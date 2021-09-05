import machine
import digitalio
import board
from pyb import Timer, Pin

IP_ADDRESS = '192.168.1.43'

def bbInitCode(caller_class):
    caller_class.peltierPower = digitalio.DigitalInOut(board.B14)
    caller_class.peltierPower.switch_to_output(value = False)

    caller_class.tim4 = Timer(4, freq=10000)
    caller_class.light1 = caller_class.tim4.channel(3, Timer.PWM, pin=Pin('B8')),
    caller_class.light2 = caller_class.tim4.channel(4, Timer.PWM, pin=Pin('B9'))
    caller_class.setLight1(0)
    caller_class.setLight2(0)

def escInitCode(caller_class):
    caller_class.tim2 = Timer(2, freq=400)
    caller_class.tim8 = Timer(8, freq=400)

    caller_class.thrusters = [
        caller_class.tim2.channel(1, Timer.PWM, pin=Pin('A0')),
        caller_class.tim2.channel(2, Timer.PWM, pin=Pin('A1')),
        caller_class.tim2.channel(3, Timer.PWM, pin=Pin('A2')),
        caller_class.tim2.channel(4, Timer.PWM, pin=Pin('A3')),
        caller_class.tim8.channel(1, Timer.PWM, pin=Pin('C6')),
        caller_class.tim8.channel(2, Timer.PWM, pin=Pin('C7')),
        caller_class.tim8.channel(3, Timer.PWM, pin=Pin('C8')),
        caller_class.tim8.channel(4, Timer.PWM, pin=Pin('C9'))
    ]