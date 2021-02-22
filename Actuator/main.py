def mainLoop():
    try:
        import hal
        onActuator = True
    except ImportError:
        onActuator = False
        import halSimulated as hal
        import traceback

    import sys
    import commands

    while True:
        try:
                # Read data from i2c, process it, then respond
                # Loop infinitely. This is the only thing that it can do due to limitations in micropython
                # If it is not actively monitoring the i2c bus it might miss a packet
                
                command = hal.coproComm.receive_command()
                responseVal = commands.processCommand(command)
                hal.coproComm.send_response(responseVal)
        except Exception as exc:
            hal.raiseFault()
            if not onActuator:
                traceback.print_exc()
                print(exc)

                # Exit if error is from socket
                if isinstance(exc, OSError):
                    if exc.errno == 10038 or exc.errno == 10054:
                        break
                if isinstance(exc, hal.I2C.ProtocolError):
                    break

            else:
                print("Error during main program loop")
                sys.print_exception(exc)

if __name__ == "__main__":
    try:
        mainLoop()
    finally:
        # Always turn on fault led since the code shouldn't get this far
        # If it is running in a simulator, it will just print an error
        try:
            print("Program unexpectedly terminated")
            from pyb import LED
            faultLed = LED(1)
            faultLed.on()
        except:
            print("Failed to turn on fault led at program termination")
