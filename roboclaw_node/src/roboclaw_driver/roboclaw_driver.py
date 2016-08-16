"""roboclaw_driver.py

Implements roboclaw communication protocol and commands.
"""

import random
import time
import serial

_TRYSTIMEOUT = 3
_CRC = 0
_PORT = None


class Cmd(object):
    """Command Enums"""
    M1FORWARD = 0
    M1BACKWARD = 1
    SETMINMB = 2
    SETMAXMB = 3
    M2FORWARD = 4
    M2BACKWARD = 5
    M17BIT = 6
    M27BIT = 7
    MIXEDFORWARD = 8
    MIXEDBACKWARD = 9
    MIXEDRIGHT = 10
    MIXEDLEFT = 11
    MIXEDFB = 12
    MIXEDLR = 13
    GETM1ENC = 16
    GETM2ENC = 17
    GETM1SPEED = 18
    GETM2SPEED = 19
    RESETENC = 20
    GETVERSION = 21
    SETM1ENCCOUNT = 22
    SETM2ENCCOUNT = 23
    GETMBATT = 24
    GETLBATT = 25
    SETMINLB = 26
    SETMAXLB = 27
    SETM1PID = 28
    SETM2PID = 29
    GETM1ISPEED = 30
    GETM2ISPEED = 31
    M1DUTY = 32
    M2DUTY = 33
    MIXEDDUTY = 34
    M1SPEED = 35
    M2SPEED = 36
    MIXEDSPEED = 37
    M1SPEEDACCEL = 38
    M2SPEEDACCEL = 39
    MIXEDSPEEDACCEL = 40
    M1SPEEDDIST = 41
    M2SPEEDDIST = 42
    MIXEDSPEEDDIST = 43
    M1SPEEDACCELDIST = 44
    M2SPEEDACCELDIST = 45
    MIXEDSPEEDACCELDIST = 46
    GETBUFFERS = 47
    GETPWMS = 48
    GETCURRENTS = 49
    MIXEDSPEED2ACCEL = 50
    MIXEDSPEED2ACCELDIST = 51
    M1DUTYACCEL = 52
    M2DUTYACCEL = 53
    MIXEDDUTYACCEL = 54
    READM1PID = 55
    READM2PID = 56
    SETMAINVOLTAGES = 57
    SETLOGICVOLTAGES = 58
    GETMINMAXMAINVOLTAGES = 59
    GETMINMAXLOGICVOLTAGES = 60
    SETM1POSPID = 61
    SETM2POSPID = 62
    READM1POSPID = 63
    READM2POSPID = 64
    M1SPEEDACCELDECCELPOS = 65
    M2SPEEDACCELDECCELPOS = 66
    MIXEDSPEEDACCELDECCELPOS = 67
    SETM1DEFAULTACCEL = 68
    SETM2DEFAULTACCEL = 69
    SETPINFUNCTIONS = 74
    GETPINFUNCTIONS = 75
    SETDEADBAND = 76
    GETDEADBAND = 77
    RESTOREDEFAULTS = 80
    GETTEMP = 82
    GETTEMP2 = 83
    GETERROR = 90
    GETENCODERMODE = 91
    SETM1ENCODERMODE = 92
    SETM2ENCODERMODE = 93
    WRITENVM = 94
    READNVM = 95
    SETCONFIG = 98
    GETCONFIG = 99
    SETM1MAXCURRENT = 133
    SETM2MAXCURRENT = 134
    GETM1MAXCURRENT = 135
    GETM2MAXCURRENT = 136
    SETPWMMODE = 148
    GETPWMMODE = 149
    FLAGBOOTLOADER = 255


# Private Functions


def crc_clear():
    global _CRC
    _CRC = 0
    return


def crc_update(data):
    global _CRC
    _CRC ^= data << 8
    for bit in range(0, 8):
        if (_CRC & 0x8000) == 0x8000:
            _CRC = ((_CRC << 1) ^ 0x1021)
        else:
            _CRC <<= 1
    return


def _send_cmd(addr, command):
    crc_clear()
    crc_update(addr)
    _PORT.write(chr(addr))
    crc_update(command)
    _PORT.write(chr(command))
    return


def _read_cksum_word():
    data = _PORT.read(2)
    if len(data) == 2:
        crc = (ord(data[0]) << 8) | ord(data[1])
        return 1, crc
    return 0, 0


def _read_byte():
    data = _PORT.read(1)
    if len(data):
        val = ord(data)
        crc_update(val)
        return 1, val
    return 0, 0


def _read_word():
    val1 = _read_byte()
    if val1[0]:
        val2 = _read_byte()
        if val2[0]:
            return 1, val1[1] << 8 | val2[1]
    return 0, 0


def _readlong():
    val1 = _read_byte()
    if val1[0]:
        val2 = _read_byte()
        if val2[0]:
            val3 = _read_byte()
            if val3[0]:
                val4 = _read_byte()
                if val4[0]:
                    return 1, val1[1] << 24 | val2[1] << 16 | val3[1] << 8 | val4[1]
    return 0, 0


def _readslong():
    val = _readlong()
    if val[0]:
        if val[1] & 0x80000000:
            return val[0], val[1] - 0x100000000
        return val[0], val[1]
    return 0, 0


def _write_byte(val):
    crc_update(val & 0xFF)
    _PORT.write(chr(val & 0xFF))


def _write_sbyte(val):
    _write_byte(val)


def _write_word(val):
    _write_byte((val >> 8) & 0xFF)
    _write_byte(val & 0xFF)


def _write_sword(val):
    _write_word(val)


def _write_long(val):
    _write_byte((val >> 24) & 0xFF)
    _write_byte((val >> 16) & 0xFF)
    _write_byte((val >> 8) & 0xFF)
    _write_byte(val & 0xFF)


def _write_slong(val):
    _write_long(val)


def _read1(addr, cmd):
    global _CRC
    trys = _TRYSTIMEOUT
    while 1:
        _PORT.flushInput()
        _send_cmd(addr, cmd)
        val1 = _read_byte()
        if val1[0]:
            crc = _read_cksum_word()
            if crc[0]:
                if _CRC & 0xFFFF != crc[1] & 0xFFFF:
                    return 0, 0
                return 1, val1[1]
        trys -= 1
        if trys == 0:
            break
    return 0, 0


def _read2(addr, cmd):
    global _CRC
    trys = _TRYSTIMEOUT
    while 1:
        _PORT.flushInput()
        _send_cmd(addr, cmd)
        val1 = _read_word()
        if val1[0]:
            crc = _read_cksum_word()
            if crc[0]:
                if _CRC & 0xFFFF != crc[1] & 0xFFFF:
                    return 0, 0
                return 1, val1[1]
        trys -= 1
        if trys == 0:
            break
    return 0, 0


def _read4(addr, cmd):
    global _CRC
    trys = _TRYSTIMEOUT
    while 1:
        _PORT.flushInput()
        _send_cmd(addr, cmd)
        val1 = _readlong()
        if val1[0]:
            crc = _read_cksum_word()
            if crc[0]:
                if _CRC & 0xFFFF != crc[1] & 0xFFFF:
                    return 0, 0
                return 1, val1[1]
        trys -= 1
        if trys == 0:
            break
    return 0, 0


def _read4_1(addr, cmd):
    global _CRC
    trys = _TRYSTIMEOUT
    while 1:
        _PORT.flushInput()
        _send_cmd(addr, cmd)
        val1 = _readslong()
        if val1[0]:
            val2 = _read_byte()
            if val2[0]:
                crc = _read_cksum_word()
                if crc[0]:
                    if _CRC & 0xFFFF != crc[1] & 0xFFFF:
                        return 0, 0
                    return 1, val1[1], val2[1]
        trys -= 1
        if trys == 0:
            break
    return 0, 0, 0


def _read_n(addr, cmd, args):
    global _CRC
    trys = _TRYSTIMEOUT
    while 1:
        _PORT.flushInput()
        trys -= 1
        if trys == 0:
            break
        failed = False
        _send_cmd(addr, cmd)
        data = [1, ]
        for i in range(0, args):
            val = _readlong()
            if val[0] == 0:
                failed = True
                break
            data.append(val[1])
        if failed:
            continue
        crc = _read_cksum_word()
        if crc[0]:
            if _CRC & 0xFFFF == crc[1] & 0xFFFF:
                return data
    return 0, 0, 0, 0, 0


def _write_cksum():
    global _CRC
    _write_word(_CRC & 0xFFFF)
    val = _read_byte()
    if val[0]:
        return True
    return False


def _write_0(addr, cmd):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write1(addr, cmd, val):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_byte(val)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write11(addr, cmd, val1, val2):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_byte(val1)
        _write_byte(val2)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write111(addr, cmd, val1, val2, val3):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_byte(val1)
        _write_byte(val2)
        _write_byte(val3)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write2(addr, cmd, val):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_word(val)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_s2(addr, cmd, val):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_sword(val)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write22(addr, cmd, val1, val2):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_word(val1)
        _write_word(val2)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_s22(addr, cmd, val1, val2):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_sword(val1)
        _write_word(val2)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_s2s2(addr, cmd, val1, val2):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_sword(val1)
        _write_sword(val2)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_s24(addr, cmd, val1, val2):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_sword(val1)
        _write_long(val2)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _writes_24s24(addr, cmd, val1, val2, val3, val4):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_sword(val1)
        _write_long(val2)
        _write_sword(val3)
        _write_long(val4)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_4(addr, cmd, val):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_long(val)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_s4(addr, cmd, val):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_slong(val)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_44(addr, cmd, val1, val2):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_long(val1)
        _write_long(val2)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_4s4(addr, cmd, val1, val2):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_long(val1)
        _write_slong(val2)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_s4s4(addr, cmd, val1, val2):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_slong(val1)
        _write_slong(val2)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_441(addr, cmd, val1, val2, val3):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_long(val1)
        _write_long(val2)
        _write_byte(val3)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_s441(addr, cmd, val1, val2, val3):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_slong(val1)
        _write_long(val2)
        _write_byte(val3)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_4s4s4(addr, cmd, val1, val2, val3):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_long(val1)
        _write_slong(val2)
        _write_slong(val3)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_4s441(addr, cmd, val1, val2, val3, val4):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_long(val1)
        _write_slong(val2)
        _write_long(val3)
        _write_byte(val4)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_4444(addr, cmd, val1, val2, val3, val4):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_long(val1)
        _write_long(val2)
        _write_long(val3)
        _write_long(val4)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_4s44s4(addr, cmd, val1, val2, val3, val4):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_long(val1)
        _write_slong(val2)
        _write_long(val3)
        _write_slong(val4)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_44441(addr, cmd, val1, val2, val3, val4, val5):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_long(val1)
        _write_long(val2)
        _write_long(val3)
        _write_long(val4)
        _write_byte(val5)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_s44s441(addr, cmd, val1, val2, val3, val4, val5):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_slong(val1)
        _write_long(val2)
        _write_slong(val3)
        _write_long(val4)
        _write_byte(val5)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_4s44s441(addr, cmd, val1, val2, val3, val4, val5, val6):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_long(val1)
        _write_slong(val2)
        _write_long(val3)
        _write_slong(val4)
        _write_long(val5)
        _write_byte(val6)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_4s444s441(addr, cmd, val1, val2, val3, val4, val5, val6, val7):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_long(val1)
        _write_slong(val2)
        _write_long(val3)
        _write_long(val4)
        _write_slong(val5)
        _write_long(val6)
        _write_byte(val7)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_4444444(addr, cmd, val1, val2, val3, val4, val5, val6, val7):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_long(val1)
        _write_long(val2)
        _write_long(val3)
        _write_long(val4)
        _write_long(val5)
        _write_long(val6)
        _write_long(val7)
        if _write_cksum():
            return True
        trys -= 1
    return False


def _write_444444441(addr,
                     cmd,
                     val1,
                     val2,
                     val3,
                     val4,
                     val5,
                     val6,
                     val7,
                     val8,
                     val9):
    trys = _TRYSTIMEOUT
    while trys:
        _send_cmd(addr, cmd)
        _write_long(val1)
        _write_long(val2)
        _write_long(val3)
        _write_long(val4)
        _write_long(val5)
        _write_long(val6)
        _write_long(val7)
        _write_long(val8)
        _write_byte(val9)
        if _write_cksum():
            return True
        trys -= 1
    return False


# User accessible functions

def send_rand_data(cnt):
    for val in range(0, cnt):
        byte = random.getrandbits(8)
        _PORT.write(chr(byte))
    return


def fw_m1(addr, val):
    return _write1(addr, Cmd.M1FORWARD, val)


def bk_m1(addr, val):
    return _write1(addr, Cmd.M1BACKWARD, val)


def set_min_volt_main_bat(addr, val):
    return _write1(addr, Cmd.SETMINMB, val)


def set_max_volt_main_bat(addr, val):
    return _write1(addr, Cmd.SETMAXMB, val)


def fw_m2(addr, val):
    return _write1(addr, Cmd.M2FORWARD, val)


def bk_m2(addr, val):
    return _write1(addr, Cmd.M2BACKWARD, val)


def fw_bk_m1(addr, val):
    return _write1(addr, Cmd.M17BIT, val)


def fw_bk_m2(addr, val):
    return _write1(addr, Cmd.M27BIT, val)


def fw_mixed(addr, val):
    return _write1(addr, Cmd.MIXEDFORWARD, val)


def bk_mixed(addr, val):
    return _write1(addr, Cmd.MIXEDBACKWARD, val)


def turn_r_mixed(addr, val):
    return _write1(addr, Cmd.MIXEDRIGHT, val)


def turn_l_mixed(addr, val):
    return _write1(addr, Cmd.MIXEDLEFT, val)


def fb_mixed(addr, val):
    return _write1(addr, Cmd.MIXEDFB, val)


def lr_mixed(addr, val):
    return _write1(addr, Cmd.MIXEDLR, val)


def get_enc_m1(addr):
    return _read4_1(addr, Cmd.GETM1ENC)


def get_enc_m2(addr):
    return _read4_1(addr, Cmd.GETM2ENC)


def get_speed_m1(addr):
    return _read4_1(addr, Cmd.GETM1SPEED)


def get_speed_m2(addr):
    return _read4_1(addr, Cmd.GETM2SPEED)


def rst_enc(addr):
    return _write_0(addr, Cmd.RESETENC)


def get_version(addr):
    global _CRC
    trys = _TRYSTIMEOUT
    while 1:
        _PORT.flushInput()
        _send_cmd(addr, Cmd.GETVERSION)
        str = ""
        passed = True
        for i in range(0, 48):
            data = _PORT.read(1)
            if len(data):
                val = ord(data)
                crc_update(val)
                if val == 0:
                    break
                str += data[0]
            else:
                passed = False
                break
        if passed:
            crc = _read_cksum_word()
            if crc[0]:
                if _CRC & 0xFFFF == crc[1] & 0xFFFF:
                    return 1, str
                else:
                    time.sleep(0.01)
        trys -= 1
        if trys == 0:
            break
    return 0, 0


def set_enc_m1(addr, cnt):
    return _write_4(addr, Cmd.SETM1ENCCOUNT, cnt)


def set_enc_m2(addr, cnt):
    return _write_4(addr, Cmd.SETM2ENCCOUNT, cnt)


def get_main_bat_volt(addr):
    return _read2(addr, Cmd.GETMBATT)


def get_logic_bat_volt(addr, ):
    return _read2(addr, Cmd.GETLBATT)


def set_min_volt_logic_bat(addr, val):
    return _write1(addr, Cmd.SETMINLB, val)


def set_max_volt_logic_bat(addr, val):
    return _write1(addr, Cmd.SETMAXLB, val)


def set_m1_vel_pid(addr, p, i, d, qpps):
    return _write_4444(
        addr,
        Cmd.SETM1PID,
        long(d * 65536),
        long(p * 65536),
        long(i * 65536),
        qpps)


def set_m2_vel_pid(addr, p, i, d, qpps):
    return _write_4444(
        addr,
        Cmd.SETM2PID,
        long(d * 65536),
        long(p * 65536),
        long(i * 65536),
        qpps)


def get_i_speed_m1(addr):
    return _read4_1(addr, Cmd.GETM1ISPEED)


def get_i_speed_m2(addr):
    return _read4_1(addr, Cmd.GETM2ISPEED)


def duty_m1(addr, val):
    return _write_s2(addr, Cmd.M1DUTY, val)


def duty_m2(addr, val):
    return _write_s2(addr, Cmd.M2DUTY, val)


def duty_m1m2(addr, m1_dty, m2_dty):
    return _write_s2s2(addr, Cmd.MIXEDDUTY, m1_dty, m2_dty)


def speed_m1(addr, val):
    return _write_s4(addr, Cmd.M1SPEED, val)


def speed_m2(addr, val):
    return _write_s4(addr, Cmd.M2SPEED, val)


def speed_m1m2(addr, m1_speed, m2_speed):
    return _write_s4s4(addr, Cmd.MIXEDSPEED, m1_speed, m2_speed)


def speed_accel_m1(addr, accel, speed):
    return _write_4s4(addr, Cmd.M1SPEEDACCEL, accel, speed)


def speed_accel_m2(addr, accel, speed):
    return _write_4s4(addr, Cmd.M2SPEEDACCEL, accel, speed)


def speed_accel_m1m2(addr, accel, speed1, speed2):
    return _write_4s4s4(addr, Cmd.MIXEDSPEEDACCEL, accel, speed1, speed2)


def speed_dist_m1(addr, speed, distance, buffer_val):
    return _write_s441(addr, Cmd.M1SPEEDDIST, speed, distance, buffer_val)


def speed_dist_m2(addr, speed, distance, buffer_val):
    return _write_s441(addr, Cmd.M2SPEEDDIST, speed, distance, buffer_val)


def speed_dist_m1m2(addr, speed1, distance1, speed2, distance2, buffer_val):
    return _write_s44s441(
        addr,
        Cmd.MIXEDSPEEDDIST,
        speed1,
        distance1,
        speed2,
        distance2,
        buffer_val)


def speed_accel_dist_m1(addr, accel, speed, distance, buffer_val):
    return _write_4s441(
        addr, Cmd.M1SPEEDACCELDIST, accel, speed, distance, buffer_val)


def speed_accel_dist_m2(addr, accel, speed, distance, buffer_val):
    return _write_4s441(
        addr, Cmd.M2SPEEDACCELDIST, accel, speed, distance, buffer_val)


def speed_accel_dist_m1m2(addr,
                          accel,
                          speed1,
                          distance1,
                          speed2,
                          distance2,
                          buffer_val):
    return _write_4s44s441(
        addr,
        Cmd.MIXEDSPEED2ACCELDIST,
        accel,
        speed1,
        distance1,
        speed2,
        distance2,
        buffer_val)


def get_buffers(addr):
    val = _read2(addr, Cmd.GETBUFFERS)
    if val[0]:
        return 1, val[1] >> 8, val[1] & 0xFF
    return 0, 0, 0


def get_pwms(addr):
    val = _read4(addr, Cmd.GETPWMS)
    if val[0]:
        pwm1 = val[1] >> 16
        pwm2 = val[1] & 0xFFFF
        if pwm1 & 0x8000:
            pwm1 -= 0x10000
        if pwm2 & 0x8000:
            pwm2 -= 0x10000
        return 1, pwm1, pwm2
    return 0, 0, 0


def get_currents(addr):
    val = _read4(addr, Cmd.GETCURRENTS)
    if val[0]:
        cur1 = val[1] >> 16
        cur2 = val[1] & 0xFFFF
        if cur1 & 0x8000:
            cur1 -= 0x10000
        if cur2 & 0x8000:
            cur2 -= 0x10000
        return 1, cur1, cur2
    return 0, 0, 0


def speed_accel_m1m2_2(addr, accel_1, speed_1, accel_2, speed_2):
    return _write_4s44s4(
        addr, Cmd.MIXEDSPEED2ACCEL, accel_1, speed_1, accel_2, speed_2)


def speed_accel_dist_1m2_2(addr,
                           accel1,
                           speed1,
                           distance1,
                           accel2,
                           speed2,
                           distance2,
                           buffer_val):
    return _write_4s444s441(
        addr,
        Cmd.MIXEDSPEED2ACCELDIST,
        accel1,
        speed1,
        distance1,
        accel2,
        speed2,
        distance2,
        buffer_val)


def duty_accel_m1(addr, accel, duty):
    return _write_s24(addr, Cmd.M1DUTYACCEL, duty, accel)


def duty_accel_m2(addr, accel, duty):
    return _write_s24(addr, Cmd.M2DUTYACCEL, duty, accel)


def duty_accel_m1m2(addr, accel1, duty1, accel2, duty2):
    return _writes_24s24(
        addr, Cmd.MIXEDDUTYACCEL, duty1, accel1, duty2, accel2)


def get_m1_vel_pid(addr):
    data = _read_n(addr, Cmd.READM1PID, 4)
    if data[0]:
        data[1] /= 65536.0
        data[2] /= 65536.0
        data[3] /= 65536.0
        return data
    return 0, 0, 0, 0, 0


def get_m2_vel_pid(addr):
    data = _read_n(addr, Cmd.READM2PID, 4)
    if data[0]:
        data[1] /= 65536.0
        data[2] /= 65536.0
        data[3] /= 65536.0
        return data
    return 0, 0, 0, 0, 0


def set_main_volts(addr, min_val, max_val):
    return _write22(addr, Cmd.SETMAINVOLTAGES, min_val, max_val)


def set_logic_volts(addr, min_val, max_val):
    return _write22(addr, Cmd.SETLOGICVOLTAGES, min_val, max_val)


def get_min_max_main_volts(addr):
    val = _read4(addr, Cmd.GETMINMAXMAINVOLTAGES)
    if val[0]:
        min_val = val[1] >> 16
        max_val = val[1] & 0xFFFF
        return 1, min_val, max_val
    return 0, 0, 0


def get_min_max_logic_volts(addr):
    val = _read4(addr, Cmd.GETMINMAXLOGICVOLTAGES)
    if val[0]:
        min_val = val[1] >> 16
        max_val = val[1] & 0xFFFF
        return 1, min_val, max_val
    return 0, 0, 0


def set_m1_pos_pid(addr, kp, ki, kd, kimax, deadzone, min_val, max_val):
    return _write_4444444(
        addr,
        Cmd.SETM1POSPID,
        long(kd * 1024),
        long(kp * 1024),
        long(ki * 1024),
        kimax,
        deadzone,
        min_val,
        max_val)


def set_m2_pos_pid(addr, kp, ki, kd, kimax, deadzone, min_val, max_val):
    return _write_4444444(
        addr,
        Cmd.SETM2POSPID,
        long(kd * 1024),
        long(kp * 1024),
        long(ki * 1024),
        kimax,
        deadzone,
        min_val,
        max_val)


def get_m1_pos_pid(addr):
    data = _read_n(addr, Cmd.READM1POSPID, 7)
    if data[0]:
        data[0] /= 1024.0
        data[1] /= 1024.0
        data[2] /= 1024.0
        return data
    return 0, 0, 0, 0, 0, 0, 0, 0


def get_m2_pos_pid(addr):
    data = _read_n(addr, Cmd.READM2POSPID, 7)
    if data[0]:
        data[0] /= 1024.0
        data[1] /= 1024.0
        data[2] /= 1024.0
        return data
    return 0, 0, 0, 0, 0, 0, 0, 0


def speed_accel_deccel_pos_m1(addr,
                              accel,
                              speed,
                              deccel,
                              position,
                              buffer_val):
    return _write_44441(
        addr,
        Cmd.M1SPEEDACCELDECCELPOS,
        accel,
        speed,
        deccel,
        position,
        buffer_val)


def speed_accel_deccel_pos_m2(addr,
                              accel,
                              speed,
                              deccel,
                              position,
                              buffer_val):
    return _write_44441(
        addr,
        Cmd.M2SPEEDACCELDECCELPOS,
        accel,
        speed,
        deccel,
        position,
        buffer_val)


def speed_accel_deccel_pos_m1m2(addr,
                                accel1,
                                speed1,
                                deccel1,
                                position1,
                                accel2,
                                speed2,
                                deccel2,
                                position2,
                                buffer_val):
    return _write_444444441(
        addr,
        Cmd.MIXEDSPEEDACCELDECCELPOS,
        accel1,
        speed1,
        deccel1,
        position1,
        accel2,
        speed2,
        deccel2,
        position2,
        buffer_val)


def set_m1_defaultaccel(addr, accel):
    return _write_4(addr, Cmd.SETM1DEFAULTACCEL, accel)


def set_m2_defaultaccel(addr, accel):
    return _write_4(addr, Cmd.SETM2DEFAULTACCEL, accel)


def set_pin_fcts(addr, s3_mode, s4_mode, s5_mode):
    return _write111(addr, Cmd.SETPINFUNCTIONS, s3_mode, s4_mode, s5_mode)


def get_pin_fcts(addr):
    global _CRC
    trys = _TRYSTIMEOUT
    while trys > 0:
        _send_cmd(addr, Cmd.GETPINFUNCTIONS)
        val1 = _read_byte()
        if val1[0]:
            val2 = _read_byte()
            val3 = _read_byte()
            crc = _read_cksum_word()
            if crc[0]:
                if _CRC & 0xFFFF != crc[1] & 0xFFFF:
                    return 0, 0
                return 1, val1[1], val2[1], val3[1]
        trys -= 1
    return 0, 0


def set_deadband(addr, min_val, max_val):
    return _write11(addr, Cmd.SETDEADBAND, min_val, max_val)


def get_deadband(addr):
    val = _read2(addr, Cmd.GETDEADBAND)
    if val[0]:
        return 1, val[1] >> 8, val[1] & 0xFF
    return 0, 0, 0


def restore_defaults(addr):
    """
    Warning(TTL Serial): Baudrate will change if not already set to 38400.
    Communications will be lost
    """
    return _write_0(addr, Cmd.RESTOREDEFAULTS)


def get_temp(addr):
    return _read2(addr, Cmd.GETTEMP)


def get_temp_2(addr):
    return _read2(addr, Cmd.GETTEMP2)


def get_error(addr):
    return _read2(addr, Cmd.GETERROR)


def get_encoder_modes(addr):
    val = _read2(addr, Cmd.GETENCODERMODE)
    if val[0]:
        return 1, val[1] >> 8, val[1] & 0xFF
    return 0, 0, 0


def set_m1_encoder_mode(addr, mode):
    return _write1(addr, Cmd.SETM1ENCODERMODE, mode)


def set_m2_encoder_mode(addr, mode):
    return _write1(addr, Cmd.SETM2ENCODERMODE, mode)


def set_nvm(addr):
    """Saves active settings to NVM."""
    return _write_4(addr, Cmd.WRITENVM, 0xE22EAB7A)


def get_nvm(addr):
    """
    restores settings from NVM
    Warning(TTL Serial): If baudrate changes or the control mode changes
    communications will be lost
    """
    return _write_0(addr, Cmd.READNVM)


def set_config(addr, config):
    """
    Warning(TTL Serial): If control mode is changed from packet serial mode
    when setting config communications will be lost!
    Warning(TTL Serial): If baudrate of packet serial mode is changed
    communications will be lost!
    """
    return _write2(addr, Cmd.SETCONFIG, config)


def get_config(addr):
    return _read2(addr, Cmd.GETCONFIG)


def set_m1_max_current(addr, max):
    return _write_44(addr, Cmd.SETM1MAXCURRENT, max, 0)


def set_m2_max_current(addr, max):
    return _write_44(addr, Cmd.SETM2MAXCURRENT, max, 0)


def get_m1_max_current(addr):
    data = _read_n(addr, Cmd.GETM1MAXCURRENT, 2)
    if data[0]:
        return 1, data[1]
    return 0, 0


def get_m2_max_current(addr):
    data = _read_n(addr, Cmd.GETM2MAXCURRENT, 2)
    if data[0]:
        return 1, data[1]
    return 0, 0


def set_pwm_mode(addr, mode):
    return _write1(addr, Cmd.SETPWMMODE, mode)


def read_pwm_ode(addr):
    return _read1(addr, Cmd.GETPWMMODE)


def open_port(comport, rate):
    """Open serial port."""
    global _PORT
    _PORT = serial.Serial(comport, baudrate=rate,
                          timeout=0.1, interCharTimeout=0.01)
    return
