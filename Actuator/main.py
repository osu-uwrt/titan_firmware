try:
    import hal
    onCopro = True
except ImportError:
    onCopro = False
    import halSimulated as hal
    import traceback

import sys
import commands


def mainLoop():
    try:
        while True:
            # Read data from i2c, process it, then respond
            # Loop infinitely. This is the only thing that it can do due to limitations in micropython
            # If it is not actively monitoring the i2c bus it might miss a packet
            
            command = hal.coproComm.receive_command()
            responseVal = commands.processCommand(command)
            hal.coproComm.send_response(responseVal)

    except Exception as exc:
        hal.raiseFault()
        if not onCopro:
            traceback.print_exc()
            print(exc)
        else:
            sys.print_exception(exc)

if __name__ == "__main__":
    mainLoop()