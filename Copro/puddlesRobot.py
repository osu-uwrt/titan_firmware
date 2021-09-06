import machine
from pyb import Timer, Pin

ROBOT_NAME = 'Puddles'
IP_ADDRESS = '192.168.1.42'

def bbInitCode(caller_class):
    # Although the jetson power is no longer a command, it must be enabled so that the jetson will get power
    caller_class.jetsonPower = machine.Pin('C3', machine.Pin.OUT, value=1)

    caller_class.peltierPower = machine.Pin('C1', machine.Pin.OUT, value=0)

def escInitCode(caller_class):
    caller_class.tim4 = Timer(4, freq=400)
    caller_class.tim12 = Timer(12, freq=400)
    caller_class.tim2 = Timer(2, freq=400)

    caller_class.thrusters = [
        caller_class.tim4.channel(3, Timer.PWM, pin=Pin('B8')),
        caller_class.tim4.channel(4, Timer.PWM, pin=Pin('B9')),
        caller_class.tim12.channel(1, Timer.PWM, pin=Pin('B14')),
        caller_class.tim12.channel(2, Timer.PWM, pin=Pin('B15')),
        caller_class.tim2.channel(1, Timer.PWM, pin=Pin('A0')),
        caller_class.tim2.channel(2, Timer.PWM, pin=Pin('A1')),
        caller_class.tim2.channel(3, Timer.PWM, pin=Pin('A2')),
        caller_class.tim2.channel(4, Timer.PWM, pin=Pin('A3'))
    ]