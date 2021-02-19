try:
    import hal
except ImportError:
    import halSimulated as hal

import array

cmdFailure = 0
cmdSuccess = 1

torpedoStateFiring = 0
torpedoStateIdle = 1

class TorpedoSystem:
    torpedoTimings = None

    def __init__(self):
        self.torpedoTimings = []
        for i in range(len(hal.torpedo.torpedoGates)):
            self.torpedoTimings.append([])
            for j in range(len(hal.torpedo.coils) + 1):
                self.torpedoTimings[i].append([-1, -1])
    
    def verifyTorpedoTimings(self, needs_all_timings) -> bool:
        minimumTimerResolutionUS = int((1 / hal.torpedo.TIMER_FREQUENCY) * 1000000)

        # Check the coil timings
        for torpedoNum in range(len(hal.torpedo.torpedoGates)):
            for coilNum in range(len(hal.torpedo.coils)):
                # Skip if the timings aren't filled in yet and that's allowed
                if self.torpedoTimings[torpedoNum][coilNum][0] == -1 and self.torpedoTimings[torpedoNum][coilNum][1] == -1 and not needs_all_timings:
                    continue

                # Make sure that the values aren't negative
                if self.torpedoTimings[torpedoNum][coilNum][0] < 0 or self.torpedoTimings[torpedoNum][coilNum][1] < 0:
                    print("Timing values negative {0}:{1}".format(torpedoNum, coilNum))
                    return False

                # Make sure the coil active timing is valid
                activeDurationUS = self.torpedoTimings[torpedoNum][coilNum][1] - self.torpedoTimings[torpedoNum][coilNum][0]
                if activeDurationUS % minimumTimerResolutionUS != 0:
                    print("Unaligned Duration {0}:{1}".format(torpedoNum, coilNum))
                    return False
                
                activeDuration = activeDurationUS // minimumTimerResolutionUS

                if not hal.torpedo.check_timing_valid(activeDuration, False):
                    print("Too Large Duration {0}:{1}".format(torpedoNum, coilNum))
                    return False

                # Make sure that the pause timing is valid
                if coilNum != 0:
                    if self.torpedoTimings[torpedoNum][coilNum - 1][0] != -1:
                        pauseDurationUS = self.torpedoTimings[torpedoNum][coilNum][0] - self.torpedoTimings[torpedoNum][coilNum-1][1]

                        if pauseDurationUS % minimumTimerResolutionUS != 0:
                            print("Unaligned Pause Duration {0}:{1}".format(torpedoNum, coilNum))
                            return False
                        
                        pauseDuration = pauseDurationUS // minimumTimerResolutionUS

                        if not hal.torpedo.check_timing_valid(pauseDuration, True):
                            print("Too large pause {0}:{1}".format(torpedoNum, coilNum))
                            return False

        # Check the standby timing is valid
        standbyCoilNum = len(hal.torpedo.coils)
        if (self.torpedoTimings[torpedoNum][standbyCoilNum][0] != -1 and self.torpedoTimings[torpedoNum][standbyCoilNum][0] != -1) or needs_all_timings:
            if self.torpedoTimings[torpedoNum][standbyCoilNum][0] < 0 or self.torpedoTimings[torpedoNum][standbyCoilNum][1] < 0:
                return False
            
            if self.torpedoTimings[torpedoNum][standbyCoilNum][1] % minimumTimerResolutionUS != 0:
                return False

            standbyDuration = self.torpedoTimings[torpedoNum][standbyCoilNum][1] // minimumTimerResolutionUS

            if not hal.torpedo.check_timing_valid(standbyDuration, False):
                return False
        
        return True

    def set_torpedo_timing(self, args: bytes) -> int:
        """
        Arguments:
        Command bit 4: Torpedo ID
        Command bits 0-3: Coil ID
        Byte 1: Start Time Upper 8 bits
        Byte 2: Start Time Lower 8 bits
        Byte 3: End Time Upper 8 bits
        Byte 4: End Time Lower 8 bits
        """

        if len(args) != 5:
            return cmdFailure
    
        torpedoId = (args[0] >> 4) & 1
        coilId = args[0] & 0xF

        if torpedoId >= len(hal.torpedo.torpedoGates):
            return cmdFailure
        
        # Allow one extra for standby timings
        if coilId > len(hal.torpedo.coils):
            return cmdFailure

        startTime = (args[1] << 8) + args[2]
        endTime = (args[3] << 8) + args[4]

        oldTorpedoStart = self.torpedoTimings[torpedoId][coilId][0]
        oldTorpedoEnd = self.torpedoTimings[torpedoId][coilId][1]

        self.torpedoTimings[torpedoId][coilId][0] = startTime * 10
        self.torpedoTimings[torpedoId][coilId][1] = endTime * 10

        if not self.verifyTorpedoTimings(False):
            self.torpedoTimings[torpedoId][coilId][0] = oldTorpedoStart
            self.torpedoTimings[torpedoId][coilId][1] = oldTorpedoEnd
            return cmdFailure

        return cmdSuccess

    def get_torpedo_status(self, args: bytes) -> int:
        """
        Arguments: Unused
        Returns whether the torpedos are firing or idle as torpedoState type
        """
        if hal.torpedo.torpedo_active():
            return torpedoStateFiring
        else:
            return torpedoStateIdle

    def arm_torpedo(self, args: bytes) -> int:
        """
        Arguments:
        Command bit 4: If the torpedos should arm (1) or disarm (0)
        Command bits 0-3: Unused
        """
        
        if len(args) != 1:
            return cmdFailure
        
        should_arm = bool((args[0] >> 4) & 1)

        if hal.torpedo.arm_torpedo(should_arm):
            return cmdSuccess
        else:
            return cmdFailure
    
    def fire_torpedo(self, args: bytes) -> int:
        """
        Arguments:
        Command bit 4: The torpedo to fire
        Command bits 0-3: Unused
        """

        if len(args) != 1:
            return cmdFailure
    
        torpedoId = (args[0] >> 4) & 1

        if not self.verifyTorpedoTimings(True):
            print("Torpedo Timing Verify Failure")
            return cmdFailure
        
        minimumTimerResolutionUS = int((1 / hal.torpedo.TIMER_FREQUENCY) * 1000000)

        activeDurations = array.array('L')
        pauseDurations = array.array('L')
        
        for coilNum in range(len(hal.torpedo.coils)):
            activeDurationUS = self.torpedoTimings[torpedoId][coilNum][1] - self.torpedoTimings[torpedoId][coilNum][0]
            activeDurations.append(activeDurationUS // minimumTimerResolutionUS)

            if coilNum != 0:
                pauseDurationUS = self.torpedoTimings[torpedoId][coilNum][0] - self.torpedoTimings[torpedoId][coilNum-1][1]
                
                pauseDurations.append(pauseDurationUS // minimumTimerResolutionUS)
        
        standbyDuration = self.torpedoTimings[torpedoId][len(hal.torpedo.coils)][1] // minimumTimerResolutionUS

        if hal.torpedo.fire_torpedo(torpedoId, standbyDuration, activeDurations, pauseDurations):
            return cmdSuccess
        else:
            print("Failed to fire: Hal callback failure")
            return cmdFailure

torpedoController = TorpedoSystem()

def reset_board(args: bytes) -> int:
    """
    Arguments: Unused
    """
    hal.restart_mcu()

    # If we get here something went wrong
    return False

def release_marker(args: bytes) -> int:
    """
    Arguments:
    Command bit 4: Marker to release
    Command bits 0-3: Unused
    """
    if len(args) != 1:
        return cmdFailure
    
    dropperId = (args[0] >> 4) & 1

    if hal.marker.drop_marker(dropperId):
        return cmdSuccess
    else:
        return cmdFailure

def set_gripper_pwm(args: bytes) -> int:
    """
    Arguments:
    Command bits 0-4: Unused
    Byte 1: PWM us Upper 8 bits
    Byte 2: PWM us Lower 8 bits
    """
    if len(args) != 3:
        return cmdFailure
    
    pwm_us = args[1] * 256 + args[2]

    if hal.gripper.set_gripper_pwm(pwm_us):
        return cmdSuccess
    else:
        return cmdFailure

def get_fault_status(args: bytes) -> int:
    return int(hal.faultPresent)

"""
All command functions use the same format:
One argument contining bytes of incoming message
Expects byte of response to be returned (< 256)
"""
commandList = [
    torpedoController.set_torpedo_timing,   # ID 0
    reset_board,                            # ID 1
    get_fault_status,                       # ID 2
    torpedoController.get_torpedo_status,   # ID 3
    torpedoController.arm_torpedo,          # ID 4
    torpedoController.fire_torpedo,         # ID 5
    release_marker,                         # ID 6
    set_gripper_pwm                         # ID 7
]

def processCommand(commandData: bytes) -> int:
    # This is following the same command format as the previous actuator
    # firmware for backwards compatability. As such, only 8 commands
    # are available. If more are needed, backwards compatibility
    # will be broken with puddles's control system since commands use
    # bit 4 to pass parameters to commands

    # Only top 3 bits are used for commands
    command = commandData[0] >> 5

    # Make sure command is valid
    if command >= len(commandList):
        return cmdFailure
    if commandList[command] == None:
        return cmdFailure
    
    # Call the function and return its response
    return commandList[command](commandData)
