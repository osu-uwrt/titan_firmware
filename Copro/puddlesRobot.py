import digitalio
import digitalio
import pwmio
import board

ROBOT_NAME = 'Puddles'
IP_ADDRESS = '192.168.1.42'

def bbInitCode(caller_class):
    # Although the jetson power is no longer a command, it must be enabled so that the jetson will get power
    caller_class.jetsonPower = digitalio.DigitalInOut(board.GP11)
    caller_class.jetsonPower.switch_to_output(value = True)
    

    caller_class.peltierPower = digitalio.DigitalInOut(board.GP15)
    caller_class.jetsonPower.switch_to_output(value = False)


def escInitCode(caller_class):
   

    caller_class.thrusters = [
        pwmio.PWMOut(board.GP7, frequency=10000, duty_cycle=0),
        pwmio.PWMOut(board.GP5, frequency=10000, duty_cycle=0),
        pwmio.PWMOut(board.GP6, frequency=10000, duty_cycle=0),
        pwmio.PWMOut(board.GP14, frequency=10000, duty_cycle=0),
        pwmio.PWMOut(board.GP13, frequency=10000, duty_cycle=0),
        pwmio.PWMOut(board.GP12, frequency=10000, duty_cycle=0),
        pwmio.PWMOut(board.GP11, frequency=10000, duty_cycle=0),
        pwmio.PWMOut(board.GP10, frequency=10000, duty_cycle=0)
    ]