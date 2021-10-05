
import digitalio
import board
import pwmio

ROBOT_NAME = 'Tempest'
IP_ADDRESS = '192.168.1.43'

def bbInitCode(caller_class):
    caller_class.peltierPower = digitalio.DigitalInOut(board.GP28)
    caller_class.peltierPower.switch_to_output(value = False)

    #caller_class.light1 = pwmio.PWMOut(board.GP21, frequency=10000, duty_cycle=0)
    #caller_class.light2 = pwmio.PWMOut(board.GP19, frequency=10000, duty_cycle=0)
    #caller_class.setLight1(0)
    #caller_class.setLight2(0)

def escInitCode(caller_class):
   

    caller_class.thrusters = [
        pwmio.PWMOut(board.A3, frequency=400, duty_cycle=0),  # Actually GP29, but circuitpython is stupid
        pwmio.PWMOut(board.GP24, frequency=400, duty_cycle=0),
        pwmio.PWMOut(board.GP22, frequency=400, duty_cycle=0),
        pwmio.PWMOut(board.GP20, frequency=400, duty_cycle=0),
        pwmio.PWMOut(board.GP18, frequency=400, duty_cycle=0),
        pwmio.PWMOut(board.GP16, frequency=400, duty_cycle=0),
        pwmio.PWMOut(board.GP14, frequency=400, duty_cycle=0),
        pwmio.PWMOut(board.GP19, frequency=400, duty_cycle=0),
    ]