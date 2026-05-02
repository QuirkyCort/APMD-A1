from micropython import const
import struct
import time

VERSION = (const(1), const(1), const(1))

VERSION_REGISTER = const(0x00)
RESET_REGISTER = const(0x01)
SAVE_SETTINGS_REGISTER = const(0x02)
SPEED_PID_REGISTER = const(0x10)
POSITION_PID_REGISTER = const(0x11)
PWM_PERIOD_REGISTER = const(0x12)
STOP_MODE_REGISTER = const(0x13)
DC_REGISTER = const(0x14)
TARGET_SPEED_REGISTER = const(0x15)
SPEED_REGISTER = const(0x16)
STEPS_REGISTER = const(0x17)
CLEAR_STEPS_REGISTER = const(0x18)
TARGET_POSITION_REGISTER = const(0x19)
RUN_TO_POSITION_REGISTER = const(0x1A)
MOTOR_STATUS_REGISTER = const(0x1B)
SERVO_FREQ_REGISTER = const(0x20)
SERVO_DC_REGISTER = const(0x21)
SERVO_RUN_TO_DC_REGISTER = const(0x22)

MOTOR_STOPPED = const(1)
MOTOR_RUNNING = const(2)

SLEEP_DURATION = const(10)

MOTOR_DC_MAX = 1000

SERVO_CHANNELS = 4
SERVO_DC_MAX = 16383
SERVO_DEFAULT_FREQ = 50
SERVO_DEFAULT_US_LOW = 500
SERVO_DEFAULT_US_HIGH = 2500
SERVO_DC_TOLERANCE = 10 # About 1 deg

class APMD_A1:
    def __init__(self, i2c, addr=0x56, version_check=True):
        self.i2c = i2c
        self.addr = addr
        self.header = bytearray(2)
        self.servo_freq = [SERVO_DEFAULT_FREQ] * SERVO_CHANNELS
        self.servo_us_low = SERVO_DEFAULT_US_LOW
        self.servo_us_high = SERVO_DEFAULT_US_HIGH
        if version_check:
            version = self.get_version()
            if version != VERSION:
                msg = 'Incompatible firmware version ' + str(version[0]) + '.' + str(version[1]) + '.' + str(version[2]) + '. '
                msg += 'This driver only support version ' + str(VERSION[0]) + '.' + str(VERSION[1]) + '.' + str(VERSION[2]) + '.'
                raise RuntimeError(msg)
        self.reset()

    def read(self, reg, channel, size):
        self.header[0] = reg
        self.header[1] = channel
        self.i2c.writeto(self.addr, self.header, False)
        return self.i2c.readfrom(self.addr, size)

    def write(self, reg, channel, data):
        self.header[0] = reg
        self.header[1] = channel
        vec = [self.header, data]
        self.i2c.writevto(self.addr, vec)

    def get_version(self):
        return struct.unpack('<BBB', self.read(VERSION_REGISTER, 0, 3))
    
    def reset(self):
        self.write(RESET_REGISTER, 0, b'\x03')

    def factory_reset(self):
        self.write(RESET_REGISTER, 0, b'\x01')
        
    def save_settings(self):
        self.write(SAVE_SETTINGS_REGISTER, 0, b'\x01')
        
    def get_speed_pid(self, channel):
        return struct.unpack('<fffff', self.read(SPEED_PID_REGISTER,channel, 20))

    def set_speed_pid(self, channel, values):
        data = struct.pack('<fffff', *values)
        self.write(SPEED_PID_REGISTER, channel, data)

    def get_position_pid(self, channel):
        return struct.unpack('<fffff', self.read(POSITION_PID_REGISTER, channel, 20))

    def set_position_pid(self, channel, values):
        data = struct.pack('<fffff', *values)
        self.write(POSITION_PID_REGISTER, channel, data)
        
    def get_pwm_period(self, channel):
        return struct.unpack('<H', self.read(PWM_PERIOD_REGISTER, channel, 2))[0]

    def set_pwm_period(self, channel, period):
        data = struct.pack('<H', int(period))
        self.write(PWM_PERIOD_REGISTER, channel, data)
        
    def get_stop_mode(self, channel):
        return struct.unpack('<B', self.read(STOP_MODE_REGISTER, channel, 1))[0]

    def set_stop_mode(self, channel, mode):
        data = struct.pack('<B', mode)
        self.write(STOP_MODE_REGISTER, channel, data)
        
    def get_dc(self, channel):
        return struct.unpack('<h', self.read(DC_REGISTER, channel, 2))[0]

    def set_dc(self, channel, dc):
        data = struct.pack('<h', int(dc))
        self.write(DC_REGISTER, channel, data)

    def get_target_speed(self, channel):
        return struct.unpack('<f', self.read(TARGET_SPEED_REGISTER, channel, 4))[0]

    def set_target_speed(self, channel, speed):
        data = struct.pack('<f', speed)
        self.write(TARGET_SPEED_REGISTER, channel, data)
        
    def get_speed(self, channel):
        return struct.unpack('<h', self.read(SPEED_REGISTER, channel, 2))[0]

    def get_steps(self, channel):
        return struct.unpack('<i', self.read(STEPS_REGISTER, channel, 4))[0]
    
    def clear_steps(self, channel):
        self.write(CLEAR_STEPS_REGISTER, channel, b'\x01')

    def get_target_position(self, channel):
        return struct.unpack('<f', self.read(TARGET_POSITION_REGISTER, channel, 4))[0]

    def set_target_position(self, channel, position, wait=True):
        data = struct.pack('<f', position)
        self.write(TARGET_POSITION_REGISTER, channel, data)
        if wait:
            self.wait_until_stop(channel)
    
    def run_to_position(self, channel, position, speed, relative=0, wait=True):
        data = struct.pack('<BfH', relative, position, int(speed))
        self.write(RUN_TO_POSITION_REGISTER, channel, data)
        if wait:
            self.wait_until_stop(channel)
        
    def get_motor_status(self, channel):
        return struct.unpack('<B', self.read(MOTOR_STATUS_REGISTER, channel, 1))[0]
        
    def wait_until_stop(self, channel):
        while True:
            if self.get_motor_status(channel) == MOTOR_STOPPED:
                break
            time.sleep_ms(SLEEP_DURATION)
        
    def get_servo_freq(self, channel):
        freq = struct.unpack('<L', self.read(SERVO_FREQ_REGISTER, channel, 4))[0]
        self.servo_freq[channel] = freq
        return freq

    def set_servo_freq(self, channel, freq):
        data = struct.pack('<L', int(freq))
        self.write(SERVO_FREQ_REGISTER, channel, data)
        self.servo_freq[channel] = freq

    def set_servo_us_limits(self, low, high):
        self.servo_us_low = low
        self.servo_us_high = high
    
    def get_servo_dc(self, channel):
        return struct.unpack('<H', self.read(SERVO_DC_REGISTER, channel, 2))[0]

    def set_servo_dc(self, channel, dc):
        data = struct.pack('<H', int(dc))
        self.write(SERVO_DC_REGISTER, channel, data)

    def get_servo_percent(self, channel):
        dc = self.get_servo_dc(channel)
        return dc / SERVO_DC_MAX * 100.0

    def set_servo_percent(self, channel, percent):
        dc = percent / 100.0 * SERVO_DC_MAX
        self.set_servo_dc(channel, dc)

    def get_servo_us(self, channel):
        dc = self.get_servo_dc(channel)
        period = 1000000 / self.servo_freq[channel]
        return dc / SERVO_DC_MAX * period
    
    def set_servo_us(self, channel, us):
        period = 1000000 / self.servo_freq[channel]
        dc = us / period * SERVO_DC_MAX
        self.set_servo_dc(channel, dc)
    
    def get_servo_deg(self, channel):
        us = self.get_servo_us(channel)
        return (us - self.servo_us_low) / (self.servo_us_high - self.servo_us_low) * 180
    
    def set_servo_deg(self, channel, deg):
        us = deg / 180 * (self.servo_us_high - self.servo_us_low) + self.servo_us_low
        self.set_servo_us(channel, us)

    def run_servo_to_dc(self, channel, dc, speed, wait=True):
        data = struct.pack('<Hf', int(dc), speed)
        self.write(SERVO_RUN_TO_DC_REGISTER, channel, data)
        if wait:
            while True:
                if abs(self.get_servo_dc(channel) - dc) < SERVO_DC_TOLERANCE:
                    break

    def run_servo_to_percent(self, channel, percent, speed, wait=True):
        dc = percent / 100.0 * SERVO_DC_MAX
        speed = speed / 100.0 * SERVO_DC_MAX
        self.run_servo_to_dc(channel, dc, speed, wait=wait)

    def run_servo_to_us(self, channel, us, speed, wait=True):
        period = 1000000 / self.servo_freq[channel]
        dc = us / period * SERVO_DC_MAX
        speed = speed / period * SERVO_DC_MAX
        self.run_servo_to_dc(channel, dc, speed, wait=wait)

    def run_servo_to_deg(self, channel, deg, speed, wait=True):
        us = deg / 180 * (self.servo_us_high - self.servo_us_low) + self.servo_us_low
        speed = speed / 180 * (self.servo_us_high - self.servo_us_low)
        self.run_servo_to_us(channel, us, speed, wait=wait)
