import socket
import array
import sys
import struct
from time import sleep, time

argv = sys.argv
#argv = ["--local", "--fulltest"]

def dispatchCommandAndWait(args_array: list[int], enforceSuccess=False, debug=False) -> bytes:
    args_array.insert(0, len(args_array) + 1)
    byte_array = array.array('B', args_array).tobytes()
    if debug:
        print('Sending: {}'.format(byte_array))
    s.sendall(byte_array)
    resp = s.recv(1024)
    #resp = [r for r in resp]
    if debug:
        print('Response: {}'.format(resp))

    if len(resp) == 0:
        raise RuntimeError("Protocol Error: Zero Length Packet Received, check for errors in copro creating the response")
    
    resp_length = resp[0]
    
    if len(resp) != resp_length:
        # Note: This could occur from tcp packets arriving in segments
        # This wasn't taken into account here, but the actual code used on the robot is able to handle this
        # If this is becoming a problem, this code could be modified to handle it as well
        raise RuntimeError("Unexpected Length Packet: {0}, told {1}".format(len(resp), resp_length))

    data = resp[1:]

    if len(data) == 0:
        raise RuntimeError("Protocol Error: Empty packet received, check copro log for errors")

    if enforceSuccess:
        if len(data) != 1:
            raise RuntimeError("Command returned multi-byte packet when a success value was expected")

        if data[0] != 0 and data[0] != 1:
            raise RuntimeError("Command returned byte {0} when a success value was expected".format(data[0]))

        if data[0] == 0:
            raise RuntimeError("Command failed on a command where a success was expected")

    return data

def enforceBooleanData(data: bytes) -> bool:
    if len(data) != 1:
        raise RuntimeError("Unexpected boolean data of length {0}".format(len(data)))

    if data[0] != 0 and data[0] != 1:
        raise RuntimeError("Unexpected boolean data of {0}".format(data[0]))

    return bool(data[0])

def formatPinState(state: bool) -> str:
    if state:
        return "HIGH"
    else:
        return "LOW"

if '-h' in argv or 'help' in argv:
    print('Run this program to test individual commands with different arguments.')
    print('Use the --local flag to connect to the simulator.')
    print('Use the --fulltest flag to run a full test of all the commands.')
    print('Argument lists accept a list of comma separated bytes.')
    print('Ex: args -> 1, 2, 100, 254, 255')
    exit()
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if '--local' in argv:
        print('Connecting to the simulator')
        s.connect(('127.0.0.1', 50000))
        print('Connected')
    else:
        print('Connecting to robot. Use the --local flag to connect to the simulator')
        s.connect(('192.168.1.42', 50000))
        print('Connected')
except KeyboardInterrupt:
    print('')
    exit()

if '--fulltest' in argv:
    test_delay = 0
    print("NOTICE: This test will test EVERY command, including thrusters and powering on/off all devices")
    print("This code will also terminate on any failure, so make sure you are able to kill the robot if it does")
    print("IF YOU DO NOT WANT TO DO THIS. KILL THIS CODE NOW and manually test the commands you want")
    print("If the robot is in a state it can be tested (or running on the simulator), press enter to continue")
    input()

    #################################################
    print("Testing moboPower: Command #0")

    prevMoboState = enforceBooleanData(dispatchCommandAndWait([0]))
    print("Current Mobo Power State:", formatPinState(prevMoboState))
    
    newMoboState = not prevMoboState
    print("Setting Mobo State to:", formatPinState(newMoboState))
    dispatchCommandAndWait([0, bool(newMoboState)], enforceSuccess=True)

    checkedMoboState = enforceBooleanData(dispatchCommandAndWait([0]))
    assert checkedMoboState == newMoboState

    sleep(test_delay)
    
    print("Returning Mobo Power to previous state")
    dispatchCommandAndWait([0, bool(prevMoboState)], enforceSuccess=True)

    checkedMoboState = enforceBooleanData(dispatchCommandAndWait([0]))
    assert checkedMoboState == prevMoboState
    
    sleep(test_delay)

    #################################################
    print()
    print("Testing lightingPower: Command #1")

    prevLightingValues = dispatchCommandAndWait([1])
    assert len(prevLightingValues) == 2 and prevLightingValues[0] <= 100 and prevLightingValues[1] <= 100
    print("Current Lighting Brightness - L0: {0}; L1: {1}".format(prevLightingValues[0], prevLightingValues[1]))

    testLightingValues = [25, 50]
    print("Setting Lighting Brightness to L0: {0}; L1: {1}".format(testLightingValues[0], testLightingValues[1]))
    dispatchCommandAndWait([1] + testLightingValues, enforceSuccess=True)

    checkedLightningValues = dispatchCommandAndWait([1])
    assert checkedLightningValues == bytearray(testLightingValues)

    sleep(test_delay)

    print("Returning lightning to previous state")
    dispatchCommandAndWait([1] + list(prevLightingValues), enforceSuccess=True)

    checkedLightningValues = dispatchCommandAndWait([1])
    assert checkedLightningValues == prevLightingValues
    
    sleep(test_delay)

    #################################################
    print()
    print("Testing peltierPower: Command #3")

    prevPeltierState = enforceBooleanData(dispatchCommandAndWait([3]))
    print("Current Peltier Power State:", formatPinState(prevPeltierState))
    
    newPeltierState = not prevPeltierState
    print("Setting Peltier State to:", formatPinState(newPeltierState))
    dispatchCommandAndWait([3, bool(newPeltierState)], enforceSuccess=True)

    checkedPeltierState = enforceBooleanData(dispatchCommandAndWait([3]))
    assert checkedPeltierState == newPeltierState

    sleep(test_delay)
    
    print("Returning Peltier Power to previous state")
    dispatchCommandAndWait([3, bool(prevPeltierState)], enforceSuccess=True)

    checkedPeltierState = enforceBooleanData(dispatchCommandAndWait([3]))
    assert checkedPeltierState == prevPeltierState

    sleep(test_delay)

    #################################################
    print()
    print("Testing getBatVolts: Command #4")

    batteryData = dispatchCommandAndWait([4])
    assert len(batteryData) == 4
    portVoltage = (batteryData[0] << 8 | batteryData[1]) / 100
    stbdVoltage = (batteryData[2] << 8 | batteryData[3]) / 100

    print("Port Voltage: {0}V; Starboard Voltage: {1}V".format(portVoltage, stbdVoltage))
    
    sleep(test_delay)

    #################################################
    print()
    print("Testing getBatCurrents: Command #5")

    batteryData = dispatchCommandAndWait([5])
    assert len(batteryData) == 4
    portCurrent = (batteryData[0] << 8 | batteryData[1]) / 100
    stbdCurrent = (batteryData[2] << 8 | batteryData[3]) / 100

    print("Port Current: {0}A; Starboard Current: {1}A".format(portCurrent, stbdCurrent))

    sleep(test_delay)

    #################################################
    print()
    print("Testing getTemperature: Command #6")

    temperatureData = dispatchCommandAndWait([6])
    assert len(temperatureData) == 2
    temperatureReading = (temperatureData[0] << 8 | temperatureData[1]) / 10

    print("Temperature: {0} C".format(temperatureReading))

    sleep(test_delay)

    #################################################
    print()
    print("Testing thrusterEnable: Command #2")

    prevThrusterEnState = enforceBooleanData(dispatchCommandAndWait([2]))
    if prevThrusterEnState:
        stateReadable = "Enabled"
    else:
        stateReadable = "Disabled"
    print("Previous Thruster State:", stateReadable)

    print("Enabling thrusters for thruster testing")
    dispatchCommandAndWait([2, 1], enforceSuccess=True)

    checkedThrusterEnState = enforceBooleanData(dispatchCommandAndWait([2]))
    assert checkedThrusterEnState  # Make sure the thrusters were enabled

    sleep(test_delay)

    #################################################
    print()
    print("Testing thrusterForce: Command #7")

    prevThrusterState = dispatchCommandAndWait([7])
    assert len(prevThrusterState) == 16
    prevThrusterStateDecoded = list(struct.unpack("!8H", prevThrusterState))
    print("Previous Thruster State:", prevThrusterStateDecoded)

    thrusterPower = 0.10
    print("Setting all thrusters to {0}% power".format(thrusterPower*100))
    haltWidth = 1525  # 1500 us is halted with 25 us deadband
    fullWidth = 1900
    thrusterWidth = int((fullWidth-haltWidth) * thrusterPower + haltWidth)
    testThrusterState = struct.pack("!8H", *tuple([thrusterWidth]*8))
    dispatchCommandAndWait([7] + list(testThrusterState), enforceSuccess=True)

    checkedThrusterState = dispatchCommandAndWait([7])
    assert checkedThrusterState == testThrusterState

    sleep(test_delay)

    print("Returning thrusters to previous state")
    dispatchCommandAndWait([7] + list(prevThrusterState), enforceSuccess=True)

    checkedThrusterState = dispatchCommandAndWait([7])
    assert checkedThrusterState == prevThrusterState

    print("Returning thruster enable to previous state")
    dispatchCommandAndWait([2, int(prevThrusterEnState)], enforceSuccess=True)

    checkedThrusterEnState = enforceBooleanData(dispatchCommandAndWait([2]))
    assert checkedThrusterEnState == prevThrusterEnState

    sleep(test_delay)

    #################################################
    print()
    print("Testing switches: Command #10")

    switchStateEncoded = dispatchCommandAndWait([10])
    assert len(switchStateEncoded) == 1
    switchStateEncoded = switchStateEncoded[0]
    assert switchStateEncoded <= 0b11

    auxSwitchStateReadable = formatPinState(switchStateEncoded & 1)
    killSwitchStateReadable = formatPinState((switchStateEncoded >> 1) & 1)

    print("Kill Switch State: {0}; Aux Switch State: {1}".format(killSwitchStateReadable, auxSwitchStateReadable))

    sleep(test_delay)

    #################################################
    print()
    print("Testing depth: Command #11")

    depthEncoded = dispatchCommandAndWait([11])
    assert len(depthEncoded) == 3
    depth = (depthEncoded[0] << 16 | depthEncoded[1] << 8 | depthEncoded[2]) / 100000

    print("Depth Sensor Reading: {0}".format(depth))

    sleep(test_delay)

    #################################################
    print()
    print("Testing getThrusterCurrents: Command #12")

    thrusterCurrentEncoded = dispatchCommandAndWait([12])
    assert len(thrusterCurrentEncoded) == 8
    thrusterCurrent = [i / 25 for i in thrusterCurrentEncoded]
    print("Thruster Currents:", thrusterCurrent)

    sleep(test_delay)

    #################################################
    print()
    print("Testing actuator: Command #16")
    print("Test Not Yet Implemented")
    # TODO: Implement this

    sleep(test_delay)

    #################################################
    print()
    print("Testing latency_check: Command #17")

    start_time = time()
    dispatchCommandAndWait([17], enforceSuccess=True)
    end_time = time()

    print("Latency: {0}ms".format(round((end_time - start_time) * 1000)))

    sleep(test_delay)

    #################################################
    print()
    print("Testing memory_check: Command #18")

    memoryUsageEncoded = dispatchCommandAndWait([18])
    assert len(memoryUsageEncoded) == 2
    memoryUsageValue = ((memoryUsageEncoded[0] << 8) | memoryUsageEncoded[1]) / ((1<<16) - 1)

    print("Memory Usage: {0}%".format(round(memoryUsageValue * 100)))

    sleep(test_delay)

    #################################################
    print()
    print("Testing temp_threshold: Command #19")

    tempThresholdEncoded = dispatchCommandAndWait([19])
    assert len(tempThresholdEncoded) == 1
    tempThreshold = tempThresholdEncoded[0]

    print("Current Temperature Threshold: {0} C".format(tempThreshold))

    sleep(test_delay)

    #################################################
    print()
    print("Testing twelvePower: Command #13")

    prev12VRailState = enforceBooleanData(dispatchCommandAndWait([13]))
    print("Current 12V Rail Power:", formatPinState(prev12VRailState))

    test12VRailState = not prev12VRailState
    print("Setting 12V Rail Power:", formatPinState(test12VRailState))
    dispatchCommandAndWait([13, int(test12VRailState)], enforceSuccess=True)

    checked12VRailState = enforceBooleanData(dispatchCommandAndWait([13]))
    assert checked12VRailState == test12VRailState

    sleep(test_delay)

    print("Returning 12V Rail Power to previous state")
    dispatchCommandAndWait([13, int(prev12VRailState)], enforceSuccess=True)

    checked12VRailState = enforceBooleanData(dispatchCommandAndWait([13]))
    assert checked12VRailState == prev12VRailState

    sleep(test_delay)

    #################################################
    print()
    print("Testing fiveReset: Command #14")

    print("Sending command to toggle 5V rail")
    dispatchCommandAndWait([14], enforceSuccess=True)
    print("Done")

    sleep(test_delay)

    #################################################
    print()
    print("Testing reset: Command #15")

    # Can't dispatch and wait since the copro will never respond
    s.sendall(bytearray([2, 15]))
    
    print('Since the device was reset, the connection will just be dropped')

else:
    while True:
        try:
            command = int(input("command number -> "))
            args = input("args -> ")
            args = str(args).replace(" ", "").split(", ")
            if "" in args:
                args.remove("")
            args_array = [int(arg) for arg in args]
            args_array.insert(0, len(args_array) + 2)
            args_array.insert(1, command)
            byte_array = array.array('B', args_array).tobytes()
            print(byte_array)
            s.sendall(byte_array)
            resp = s.recv(1024)
            resp = [r for r in resp]
            print('The response is: {}'.format(resp))

        except (KeyboardInterrupt, Exception) as e:
            print(e)
            break    
    print('Closing')
    s.sendall(bytearray([0]))

s.close()