import sensor, image, time
from fpioa_manager import fm
from board import board_info
from Maix import GPIO
import time
from machine import Timer,PWM

board_pin_map = {
      'BOOT_KEY': 16,
      'LED_R': 13,
      'LED_G': 12,
      'LED_B': 14,
      'WIFI_TX': 6,
      'WIFI_RX': 7,
      'WIFI_EN': 8,
      'MIC0_WS': 19,
      'MIC0_DATA': 20,
      'MIC0_BCK': 18,
      'I2S_WS': 33,
      'I2S_DA': 34,
      'I2S_BCK': 35,
      'ESP32_CS': 25,
      'ESP32_RST': 8,
      'ESP32_RDY': 9,
      'ESP32_MOSI': 28,
      'ESP32_MISO': 26,
      'ESP32_SCLK': 27,
      'PIN0':4,
      'PIN1':5,
      'PIN2':21,
      'PIN3':22,
      'PIN4':23,
      'PIN5':24,
      'PIN6':32,
      'PIN7':15,
      'PIN8':14,
      'PIN9':13,
      'PIN10':12,
      'PIN11':11,
      'PIN12':10,
      'PIN13':3,
  }
board_info.load(board_pin_map)

_faceParameter = (1.889, 2.5245, 2.9465, 3.94056, 3.99987, 5.3658, 5.155437, 6.92275, 6.718375, 9.01025)

class Motor:
    """
    A class used to init a motor

    '''
    Attributes
    ----------
    index: int, Motor's index (1 / 2)


    Methods
    -------
    MotorRun: int direction (0 / 1), int speed
        Motor move with direction and speed
    """

    def __init__(self, index):
        """
        Parameters
        ----------
        index : int, Motor's index (either 1 or 2)
        """

        # if motor 1, uses TIMER0 and 10, 12 pins
        if index == 1:
            self.timer = Timer.TIMER0
            self.motor_dir_pin    = 10
            self.motor_speed_pin  = 12
            # register pin, reserved GPIOHS25 for motor 1 dir
            fm.register(self.motor_dir_pin, fm.fpioa.GPIOHS25)
            self.direct = GPIO(GPIO.GPIOHS25, GPIO.OUT)

        # else motor 2, uses TIMER1 and 3, 11 pins
        else:
            self.timer = Timer.TIMER1
            self.motor_dir_pin    = 3
            self.motor_speed_pin  = 11
            # register pin, reserved GPIOHS26 for motor 2 dir
            fm.register(self.motor_dir_pin, fm.fpioa.GPIOHS26)
            self.direct = GPIO(GPIO.GPIOHS26, GPIO.OUT)

        # set timer and PWM for speed
        self.tim = Timer(self.timer, Timer.CHANNEL0, mode=Timer.MODE_PWM)
        self.speed = PWM(self.tim, freq=50, duty=0, pin=self.motor_speed_pin)

    def MotorRun(self, direction, speed):
        self.direct.value(direction)
        self.speed.duty(speed)
