import os
import RPi.GPIO as GPIO
import time
import random
import sys
from colour import Color
from spidev import SpiDev
from enum import IntEnum, Enum

class LoopTestException(Exception):
    pass

class Pins(IntEnum):
    POWER = 5
    LE = 0
    SCLK_DOUBLE = 25
    MISO = 9
    MOSI = 10
    SCK = 11

class LE_Position(IntEnum):
    GLOBAL_LATCH = 3
    READ_CONFIG = 5
    EN_ERROR_DETECTION = 7
    READ_ERROR_CODE = 9
    WRITE_CONFIG = 11

class SoftSPI:
    def __init__(self):
        pass

    def open(self):
        GPIO.setup((Pins.MOSI, Pins.SCK), GPIO.OUT)
        GPIO.setup(Pins.MISO, GPIO.IN)

    def writebytes(self, data, latch_position=0):
        self.open()
        bitcount = len(data) * 8
        for byte in data:
            byte_string = format(byte, '0>8b')
            for j, bit in enumerate(byte_string):
                gpio_state = GPIO.HIGH if bit == '1' else GPIO.LOW
                GPIO.output(Pins.SCK, GPIO.LOW)
                GPIO.output(Pins.MOSI, gpio_state)
                if latch_position == bitcount:
                    GPIO.output(Pins.LE, GPIO.HIGH)
                GPIO.output(Pins.SCK, GPIO.HIGH)
                bitcount -= 1
        GPIO.output(Pins.LE, GPIO.LOW)
        self.close()

    def xfer(self, data, latch_position=0):
        self.open()
        result = []
        bitcount = len(data) * 8
        for byte in data:
            read_byte = 0
            byte_string = format(byte, '0>8b')
            for j, bit in enumerate(byte_string):
                gpio_state = GPIO.HIGH if bit == '1' else GPIO.LOW
                GPIO.output(Pins.SCK, GPIO.LOW)
                GPIO.output(Pins.MOSI, gpio_state)
                read_byte |= (GPIO.input(Pins.MISO) << (7-j))
                if latch_position == bitcount:
                    GPIO.output(Pins.LE, GPIO.HIGH)
                GPIO.output(Pins.SCK, GPIO.HIGH)
                bitcount -= 1
            result.append(read_byte)
        GPIO.output(Pins.LE, GPIO.LOW)
        self.close()
        return result

    def close(self):
        GPIO.cleanup([Pins.MISO, Pins.MOSI, Pins.SCK])

class SPI:
    def __init__(self):
        self.spidev = SpiDev(0,0)
        self.spidev.max_speed_hz = 100
        self.spidev.mode = 0
        self.spidev.no_cs = True

        self.edge_count = 0
        self.latch_position = 0


    def read(self, byte_count):
        self.spidev.open(0,0)
        data = self.spidev.readbytes(byte_count)
        self.spidev.close()
        return data

    def write(self, data):
        self.spidev.open(0, 0)
        self.spidev.writebytes(data)
        self.spidev.close()

    def xfer2(self, values):
        self.spidev.open(0,0)
        result = self.spidev.xfer2(values)
        self.spidev.close()
        return result

class Driver:
    PWM_SYNC_AUTO = 0
    PWM_SYNC_MANUAL = 1
    PWM_16BIT = 0
    PWM_12BIT = 1

    def __init__(self):
        self.timeout_alert = False
        self.thermal_protection = False
        self.current_gain = int('10101011', 2)
        self.pwm_sync_mode = Driver.PWM_SYNC_AUTO
        self.pwm_counting_mode = 0
        self.pwm_bitcount = Driver.PWM_16BIT
        self.thermal_error = False
        self.parity_error = False

    def __eq__(self, other):
        if not isinstance(other, Driver):
            return NotImplemented
        return self.timeout_alert == other.timeout_alert and self.thermal_protection == other.thermal_protection \
               and self.current_gain == other.current_gain  and self.pwm_sync_mode == other.pwm_sync_mode \
               and self.pwm_counting_mode == other.pwm_counting_mode and self.pwm_bitcount == other.pwm_bitcount

    def __ne__(self, other):
        return not self.__eq__(other)


    def set_data(self, data):
        bin_data = format(data[0], '0>8b')
        bin_data += format(data[1], '0>8b')
        self.timeout_alert = False if bin_data[15] == '0' else True
        self.thermal_protection = False if bin_data[14] == '0' else True
        self.current_gain = int(bin_data[6:14], 2)
        self.pwm_sync_mode = Driver.PWM_SYNC_AUTO if bin_data[5] == '0' else Driver.PWM_SYNC_MANUAL
        self.pwm_counting_mode = int(bin_data[3:5], 2)
        self.pwm_bitcount = Driver.PWM_16BIT if bin_data[2] == '0' else Driver.PWM_12BIT
        self.thermal_error = False if bin_data[1] == '0' else True
        self.parity_error = False if bin_data[0] == '0' else True

    def get_data(self):
        bin_data = ''
        bin_data += '1' if self.parity_error else '0'
        bin_data += '1' if self.thermal_error else '0'
        bin_data += '1' if self.pwm_bitcount == Driver.PWM_12BIT else '0'
        bin_data += '1' if self.pwm_sync_mode == Driver.PWM_SYNC_MANUAL else '0'
        bin_data += format(self.current_gain, '0>8b')
        bin_data += '1' if self.thermal_protection else '0'
        bin_data += '1' if self.timeout_alert else '0'
        return bin_data



class Controller:
    def __init__(self):
        self.power_pin = 5
        self.isPowered = False
        self.spi = SPI()
        self.soft_spi = SoftSPI()
        self.drivers = [Driver()] * 12

        random.seed()

    def configure(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.cleanup()
        GPIO.setup(channel=Pins.POWER, direction=GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(channel=Pins.LE, direction=GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Pins.SCLK_DOUBLE, GPIO.IN, GPIO.PUD_UP)

    def check_driver_config_writing(self): # TODO: Doesn't work
        check_data = [('red', 0), ('green', 100), ('blue', 200)]
        for color, value in check_data:
            for driver in self.__get_driver(color):
                driver.current_gain = value
        drivers_value = [int(driver.get_data()) for driver in self.drivers]
        self.write_drivers(drivers_value, LE_Position.WRITE_CONFIG)
        for driver in self.drivers:
            driver.current_gain = -1
        data = self.read_drivers(LE_Position.READ_CONFIG)
        for i, driver in enumerate(self.drivers):
            driver.set_data([data[i * 2], data[i * 2 + 1]])
        for color, value in check_data:
            for i, driver in enumerate(self.__get_driver(color)):
                if driver.current_gain != value:
                    self.error('Driver {0} N {1} write config failed. Expected: {2}; Found: {3}'
                               .format(color, i, value, driver.current_gain))

    def togglePower(self, state):
        self.isPowered = state
        GPIO.output(Pins.POWER, not state)

    def checkLoop(self, loop_offset=24):
        print ('Checking loop')
        input = [x for x in range(0, 256)]
        output = self.soft_spi.xfer(list(input))
        for i, j in zip(range(0, 256), range(loop_offset, 256)):
            if input[i] != output[j]:
                print ('Input {}'.format(input))
                print ('Output: {}'.format(output))
                print('input[{}] != output[{}]; {} != {}'.format(i, j, input[i], output[j]))
                self.error('Loop test failed')
                return False
        return True

    def checkDefaults(self):
        print ('Checking drivers default settings')
        defaults = self.read_drivers(LE_Position.READ_CONFIG)
        default_driver = Driver()
        for i, driver in enumerate(self.drivers):
            driver.set_data([defaults[i * 2], defaults[i * 2 + 1]])
            if driver != default_driver:
                self.error('Driver {0} settings are not in default state. Data: {1}'.format(i, driver.get_data()))

    def __get_driver(self, color = '', number = -1): #TODO: check it
        if number < -1 or number > 3:
            return None
        color_offset = 0
        if color == 'green':
            color_offset = 4
        elif color == 'red':
            color_offset = 8
        if number == -1:
            return self.drivers[color_offset : color_offset + 4]
        return self.drivers[color_offset + number]

    def read_drivers(self, latch_position=0):
        self.soft_spi.xfer([0, 0], latch_position)
        data = self.soft_spi.xfer([0]*24, 0)
        return data

    def write_drivers(self, data, latch_position=0):
        self.soft_spi.writebytes(data, latch_position)

    def error(self, message):
        print('ERROR: ' + message)
        self.togglePower(False)
        GPIO.cleanup()
        sys.exit(1)


if __name__ == '__main__':
    print('Cube application started')
    print('Operating system: {}'.format(os.name))
    controller = Controller()
    controller.configure()
    controller.togglePower(True)
    time.sleep(1)
    controller.checkLoop()
    controller.checkDefaults()
    controller.check_driver_config_writing()
    controller.togglePower(False)
