#!/usr/bin/env python
"""roboclaw_node.py

ROS node implementeing differencitl drive using the RoboClaw motor controller.
"""

from math import pi, cos, sin

import diagnostic_msgs
import diagnostic_updater
import roboclaw_driver.roboclaw_driver as roboclaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"

G_INVERT_MOTOR_AXES = True
G_FLIP_LEFT_RIGHT_MOTORS = False  # By default M1=right motor M2=left motor


# TODO: need to find some better was of handling OSerror 11 or preventing it,
#       any ideas?

class EncoderOdom(object):
    def __init__(self, ticks_per_meter, base_width):
        self.tpm = ticks_per_meter
        self.base_width = base_width
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = rospy.Time.now()

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.tpm
        dist_right = right_ticks / self.tpm
        dist = (dist_right + dist_left) / 2.0

        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        # TODO: find better what to determine going straight, this means slight
        #       deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.base_width
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO: Let's find a better way to deal with this error
        if abs(enc_left - self.last_enc_left) > 20000:
            rospy.logerr(
                "Ignoring left encoder jump: cur {}, last {}".format(
                    enc_left, self.last_enc_left))
        elif abs(enc_right - self.last_enc_right) > 20000:
            rospy.logerr(
                "Ignoring right encoder jump: cur {}, last {}".format(
                    enc_right, self.last_enc_right))
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self.publish_odom(
                self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        br = tf.TransformBroadcaster()
        br.sendTransform((cur_x, cur_y, 0),
                         tf.transformations.quaternion_from_euler(
                         0, 0, cur_theta),
                         current_time,
                         "base_footprint",
                         "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_footprint'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)


class Node(object):
    def __init__(self):
        self.errors = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        rospy.init_node("roboclaw_node")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baud", "115200"))

        self.address = int(rospy.get_param("~address", "128"))
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")

        # TODO need someway to check if address is correct
        try:
            roboclaw.open_port(dev_name, baud_rate)
        except IOError as err:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(err)
            rospy.signal_shutdown("Could not connect to Roboclaw")

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.
                         FunctionDiagnosticTask("Vitals", self.check_vitals))

        try:
            version = roboclaw.read_version(self.address)
        except IOError as err:
            rospy.logwarn("Problem getting roboclaw version")
            rospy.logdebug(err)

        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
        else:
            rospy.logdebug(repr(version[1]))

        roboclaw.speed_m1m2(self.address, 0, 0)
        roboclaw.rst_enc(self.address)

        self.max_speed = float(rospy.get_param("~max_speed", "2.0"))
        self.tpm = float(rospy.get_param("~ticks_per_meter", "4342.2"))
        self.base_width = float(rospy.get_param("~base_width", "0.315"))

        self.encodm = EncoderOdom(self.tpm, self.base_width)
        self.last_set_speed_time = rospy.get_rostime()

        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        rospy.sleep(1)

        rospy.logdebug("dev {}".format(dev_name))
        rospy.logdebug("baud {}".format(baud_rate))
        rospy.logdebug("address {}".format(self.address))
        rospy.logdebug("max_speed {}".format(self.max_speed))
        rospy.logdebug("ticks_per_meter {}".format(self.tpm))
        rospy.logdebug("base_width {}".format(self.base_width))

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(10)
        while not rospy.is_shutdown():

            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 1:
                rospy.logdebug("Did not get comand for 1 second, stopping")
                try:
                    roboclaw.fw_m1(self.address, 0)
                    roboclaw.fw_m2(self.address, 0)
                except OSError as err:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(err)

            # TODO: need find solution to the OSError11 looks like sync problem
            #       with serial
            status1, enc1, crc1 = None, None, None
            status2, enc2, crc2 = None, None, None

            try:
                status1, enc1, crc1 = roboclaw.get_enc_m1(self.address)
            except ValueError:
                pass
            except OSError as err:
                rospy.logwarn("ReadEncM1 OSError: %d", err.errno)
                rospy.logdebug(err)

            try:
                status2, enc2, crc2 = roboclaw.get_enc_m2(self.address)
            except ValueError:
                pass
            except OSError as err:
                rospy.logwarn("ReadEncM2 OSError: %d", err.errno)
                rospy.logdebug(err)

        try:
            if G_INVERT_MOTOR_AXES:
                enc1 = -enc1
                enc2 = -enc2

            if G_FLIP_LEFT_RIGHT_MOTORS:
                enc1_t = enc1
                enc2_t = enc2

                enc2 = enc1_t
                enc1 = enc2_t

                rospy.logdebug("Encoders {} {}".format(enc1, enc2))
                # update_publish expects enc_left enc_right
                self.encodm.update_publish(enc2, enc1)

                self.updater.update()
        except Exception as err:
            print "Problems reading encoders: {}".format(err)
            r_time.sleep()

    def cmd_vel_callback(self, twist):
        self.last_set_speed_time = rospy.get_rostime()

        linear_x = twist.linear.x
        if linear_x > self.max_speed:
            linear_x = self.max_speed
        if linear_x < -self.max_speed:
            linear_x = -self.max_speed

        vr = linear_x + twist.angular.z * self.base_width / 2.0  # m/s
        vl = linear_x - twist.angular.z * self.base_width / 2.0

        if G_INVERT_MOTOR_AXES:
            vr = -vr
            vl = -vl

        if G_FLIP_LEFT_RIGHT_MOTORS:
            vr_t = vr
            vl_t = vl
            vr = vl_t
            vl = vr_t

        vr_ticks = int(vr * self.tpm)  # ticks/s
        vl_ticks = int(vl * self.tpm)

        rospy.logdebug("vr_ticks:{} vl_ticks: {}".format(vr_ticks, vl_ticks))

        try:
            # This is a hack way to keep a poorly tuned PID from making noise
            # at speed 0
            if vr_ticks is 0 and vl_ticks is 0:
                roboclaw.fw_m1(self.address, 0)
                roboclaw.fw_m2(self.address, 0)
            else:
                roboclaw.speed_m1m2(self.address, vr_ticks, vl_ticks)
        except OSError as err:
            rospy.logwarn("speed_m1m2 OSError: %d", err.errno)
            rospy.logdebug(err)

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = roboclaw.get_err(self.address)[1]
        except OSError as err:
            rospy.logwarn("Diagnostics OSError: {}".format(err.errno))
            rospy.logdebug(err)
            return
        state, message = self.errors[status]
        stat.summary(state, message)
        try:
            stat.add(
                "Main Batt V:",
                float(roboclaw.get_main_bat_volt(self.address)[1] / 10))
            stat.add(
                "Logic Batt V:",
                float(roboclaw.get_logic_bat_volt(self.address)[1] / 10))
            stat.add(
                "Temp1 C:",
                float(roboclaw.get_temp(self.address)[1] / 10))
            stat.add(
                "Temp2 C:",
                float(roboclaw.get_temp_2(self.address)[1] / 10))
        except OSError as err:
            rospy.logwarn("Diagnostics OSError: {}".format(err.errno))
            rospy.logdebug(err)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            roboclaw.fw_m1(self.address, 0)
            roboclaw.fw_m2(self.address, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.fw_m1(self.address, 0)
                roboclaw.fw_m2(self.address, 0)
            except OSError as err:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(err)


if __name__ == "__main__":
    try:
        NODE = Node()
        NODE.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
