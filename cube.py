import os
import RPi.GPIO as GPIO
import time
import random
import sys
from spidev import SpiDev
from enum import IntEnum

class LoopTestException(Exception):
    pass

class Pins(IntEnum):
    POWER = 5
    LE = 0
    SCLK_DOUBLE = 25

class SPI:
    def __init__(self):
        self.spidev = SpiDev(0,0)
        self.spidev.max_speed_hz = 100
        self.spidev.mode = 0
        self.spidev.no_cs = True

        self.edge_count = 0
        self.latch_position = 0

    def read(self, byte_count, latch_position=0):
        if latch_position > 0:
            self.edge_count = 0
            self.latch_position = latch_position
            GPIO.output(Pins.LE, False)
            GPIO.add_event_detect(Pins.SCLK_DOUBLE, GPIO.FALLING, callback=self.__sclk_edge_callback__)
        self.spidev.open(0,0)
        data = self.spidev.readbytes(byte_count)
        self.spidev.close()
        if latch_position > 0:
            GPIO.output(Pins.LE, False)
            GPIO.remove_event_detect(Pins.SCLK_DOUBLE)
        return data

    def __sclk_edge_callback__(self):
        self.edge_count += 1
        print ('Callback')
        if self.edge_count == self.latch_position:
            GPIO.output(Pins.LE, True)




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

    def set_data(self, data):
        bin_data = bin(data)[2:]
        self.timeout_alert = False if bin_data[0] == '0' else True
        self.thermal_protection = False if bin_data[1] == '0' else True
        self.current_gain = int(bin_data[2:10], 2)
        self.pwm_sync_mode = Driver.PWM_SYNC_AUTO if bin_data[10] == '0' else Driver.PWM_SYNC_MANUAL
        self.pwm_counting_mode = int(bin_data[11:13], 2)
        self.pwm_bitcount = Driver.PWM_16BIT if bin_data[13] == '0' else Driver.PWM_12BIT
        self.thermal_error = False if bin_data[14] == '0' else True
        self.parity_error = False if bin_data[15] == '0' else True

    def get_data(self):
        bin_data = ''
        bin_data += '1' if self.timeout_alert else '0'
        bin_data += '1' if self.thermal_protection else '0'
        bin_data += bin(self.current_gain)[2:]
        bin_data += '1' if self.pwm_sync_mode == Driver.PWM_SYNC_MANUAL else '0'
        bin_data += '1' if self.pwm_bitcount == Driver.PWM_12BIT else '0'
        bin_data += '1' if self.thermal_error else '0'
        bin_data += '1' if self.parity_error else '0'


class Controller:
    def __init__(self):
        self.power_pin = 5
        self.isPowered = False
        self.spi = SPI()
        random.seed()

    def configure(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.cleanup()
        GPIO.setup(channel=Pins.POWER, direction=GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(channel=Pins.LE, direction=GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(Pins.SCLK_DOUBLE, GPIO.IN, GPIO.PUD_UP)

    def togglePower(self, state):
        self.isPowered = state
        GPIO.output(Pins.POWER, not state)

    def checkLoop(self, loop_offset=24):
        print ('Checking loop')
        input = [random.randint(0, 255) for x in range(0, 256)]
        self.spi.spidev.open(0,0)
        print ('Input {}'.format(input))
        output = self.spi.spidev.xfer2(list(input))
        print ('Output: {}'.format(output))
        self.spi.spidev.close()
        for i, j in zip(range(0, 256), range(loop_offset, 256)):
            if input[i] != output[j]:
                print('input[{}] != output[{}]; {} != {}'.format(i, j, input[i], output[j]))
                return False
        return True

    def checkDefaults(self):
        print ('Checking defaults')
        self.spi.read(2, 11)
        defaults = self.spi.read(24)
        print ('Defaults: ', defaults)


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
    time.sleep(2)
    if not controller.checkLoop(): controller.error('Loop test failed')
    controller.checkDefaults()
    controller.togglePower(False)
