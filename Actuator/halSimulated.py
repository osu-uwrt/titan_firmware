import array
import os
import socket
import threading
import time

class Pin:
    registered_names = []

    def __init__(self, name, value=0):
        if name in Pin.registered_names:
            raise RuntimeError("Attempting to register pin '{0}', which has already been registered".format(name))
        Pin.registered_names.append(name)
        self.name = name
        self.state = value

    def value(self, a=None):
        if a is None:
            return self.state
        else:
            if a == 1:
                state_name = "HIGH"
            elif a == 0:
                state_name = "LOW"
            else:
                raise RuntimeError("Unexpected pin state")

            print("Pin", self.name, "- Setting pin state to", state_name)
            self.state = a

faultLed = Pin("Fault LED")
faultPresent = False

def raiseFault():
    faultLed.value(1)
    
    global faultPresent
    faultPresent = True


ETIMEDOUT = 110
class I2C:
    class ProtocolError(Exception):
        pass

    """
    Simulated I2C uses a Type, Length, Value packet format, where each direciton is independent
    Types:
     - 0xA0: MOSI: Sent to start I2C tranaction with the data
     - 0xA1: MOSI: Sent to start I2C transaction from other device with length requested, expecting 0xB1 response
     - 0xA2: MOSI: Sent to acknowledge data received from 0xB1
     - 0xB0: MISO: Sent to acknowledge data from 0xA0
     - 0xB1: MISO: Sent to respond to 0xA1 request with data requested
    """
    I2C_SIMULATOR_PORT = 40123

    MOSI_TX_CONTENT = 0xA0
    MOSI_RX_REQUEST = 0xA1
    MOSI_RX_ACK = 0xA2
    MISO_RX_ACK = 0xB0
    MISO_TX_CONTENT = 0xB1

    EXPECTED_ADDR = 0x1C

    sock: socket.socket = None

    MASTER = 1
    SLAVE = 2
    def __init__(self, bus_num, mode, addr):
        # Setup socket and such
        # create an INET, STREAMing socket
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # bind the socket to a public host, and a well-known port
        self.serversocket.bind(('0.0.0.0', I2C.I2C_SIMULATOR_PORT))
        # become a server socket
        self.serversocket.listen(1)

        self.serversocket.settimeout(None)
        print("Waiting for connection to i2c simulator...")
        self.sock = self.serversocket.accept()[0]
        print("Bus Master Connected")

    def _recvall(self, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            recv_part = self.sock.recv(size - len(data))
            if len(recv_part) == 0:
                raise I2C.ProtocolError("Connection Unexpectedly Closed")
            data += recv_part
        return data

    def _sock_read_tlv(self, expected_type: int) -> bytes:
        tl_part = self._recvall(2)

        if tl_part[0] != expected_type:
            self.sock.close()
            raise I2C.ProtocolError("Unexpected Packet Type {0}, expected {1}".format(tl_part[0], expected_type))

        length = tl_part[1]
        if length != 0:
            return self._recvall(length)
        else:
            return bytearray(0)
    
    def _sock_send_tlv(self, type: int, value: bytes) -> None:
        length = 0
        if value is not None:
            length = len(value)
        header = bytearray([type, length])
        self.sock.sendall(header)
        if value is not None:
            self.sock.sendall(value)
        

    def send(self, data: bytes, timeout=5000):
        assert len(data) < 256

        self.sock.settimeout(timeout / 1000)
        try:
            request = self._sock_read_tlv(I2C.MOSI_RX_REQUEST)
            if len(request) != 1:
                raise I2C.ProtocolError("Invalid Request Packet")
            if request[0] != len(data):
                raise I2C.ProtocolError("Send data size mismatch (This is a bug in the higher level protocol)")
            self._sock_send_tlv(I2C.MISO_TX_CONTENT, data)
            ack = self._sock_read_tlv(I2C.MOSI_RX_ACK)
            if len(ack) != 0:
                raise I2C.ProtocolError("Invalid ack packet")
        except socket.timeout:
            # Make an error that is expected during normal I2C operation
            raise OSError(ETIMEDOUT)

    def recv(self, size: int, timeout=5000):
        assert size > 0 and size < 256
        
        self.sock.settimeout(timeout / 1000)
        data = None
        try:
            data = self._sock_read_tlv(I2C.MOSI_TX_CONTENT)
            if len(data) != size:
                raise I2C.ProtocolError("Data size mismatch (This is a bug in the higher level protocol)")
            self._sock_send_tlv(I2C.MISO_RX_ACK, bytearray([len(data)]))
        except socket.timeout:
            # Make an error that is expected during normal I2C operation
            raise OSError(ETIMEDOUT)
        return data

class Timer:
    class WrapperThread(threading.Thread):
        def __init__(self, id: int, parent_function):
            threading.Thread.__init__(self, name="Timer-{0}".format(id))
            self.parent_function = parent_function
        def run(self):
            self.parent_function.run_callback()

    # Thread Private Variables
    # Also can be changed in init function, since the thread is stopped
    _prescaler = None
    _period = None
    _callback = None
    _thread = None

    # Timer Constants
    # 16-bit timer
    TIMER_MAX_COUNT = 0xFFFF
    # Running on 84 MHz clock
    TIMER_FREQUENCY = 84000000

    # The values to update the prescaler and period
    # Protected by control_vars_lock
    period_queued = None

    stop_timer = False

    registered_ids = []
    def __init__(self, id: int):
        if id in Timer.registered_ids:
            raise RuntimeError("Attempting to register timer {0}, which has already been registered".format(id))
        Timer.registered_ids.append(id)
        self.name = id

        self.control_vars_lock = threading.Lock()
    
    def init(self, freq=None, prescaler=-1, period=-1, callback=None):
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("Cannot start already running timer")

        self._thread = Timer.WrapperThread(self.name, self)

        if freq is not None:
            if freq == 10:
                self._prescaler = (self.source_freq() // 100) - 1
                self._period = 10
            else:
                raise RuntimeError("The freqeuncy to prescaler conversion is not implemented")
        elif prescaler != -1 and period != -1:
            self._prescaler = prescaler
            self._period = period
        elif prescaler != -1:
            self._prescaler = prescaler
            self._period = Timer.TIMER_MAX_COUNT
        else:
            raise TypeError("must specify either freq, period, or prescaler and period")
        
        self._callback = callback

        self.stop_timer = False
        self._thread.start()
    
    def deinit(self):
        self.stop_timer = True

    def period(self, value):
        with self.control_vars_lock:
            self.period_queued = value
    
    def source_freq(self):
        return self.TIMER_FREQUENCY
    
    def counter(self, value):
        if self._thread is not None and self._thread.is_alive() and threading.currentThread() != self._thread:
            # This is because errors build up over time when using time.sleep,
            # due to the fact that the scheduler doesn't return control exactly when requested by sleep
            # To avoid this, only one time.sleep is being used, so the counter cannot be reset mid time.sleep
            raise RuntimeError("Currently counter resetting is not supported from other threads while running")
        if value != 0:
            raise RuntimeError("Cannot set counter to non-zero value with simulated timer")

        pass  # Counter is not currently implemented

    def run_callback(self):
        while not self.stop_timer:
            time.sleep(((self._prescaler + 1) / self.TIMER_FREQUENCY) * self._period)
            if self.stop_timer:
                break
            
            self._callback()

            # TODO: Check if update event occurs on roll over or at timer of zero
            with self.control_vars_lock:
                if self.period_queued is not None:
                    self._period = self.period_queued
                    self.period_queued = None
        
        self._prescaler = None
        self._period = None
        self.timer_count = 0
        self._callback = None
        self._thread = None
    
    

class PWM:
    registered_names = []

    def __init__(self, name):
        if name in PWM.registered_names:
            raise RuntimeError("Attempting to register pin '{0}', which has already been registered".format(name))
        PWM.registered_names.append(name)
        self.name = name
    state = 0

    def pulse_width_percent(self, a=None):
        if a is None:
            return self.state
        else:
            print("PWM", self.name, "- Setting pulse width to", a)
            self.state = a



class CoproCommunicator:
    # Timeout in ms
    HEADER_TIMEOUT = 5000      # Timeout when not expecting message
    TRANSACTION_TIMEOUT = 3    # Timeout when copro is expected to be there

    PACKET_MOSI_MAGIC_NIBBLE = 0b1001
    PACKET_MISO_MAGIC_NIBBLE = 0b1010
    PACKET_STATUS_SUCCESSFUL = 1
    PACKET_STATUS_CHECKSUM = 2

    def __init__(self):
        self.backplaneI2C = I2C(1, I2C.SLAVE, addr=0x1C)
        self.i2cFaultLed = Pin(2, value=0)

    def raiseBusFault(self, msg="Undefined"):
        self.i2cFaultLed.value(1)
        print("Bus fault occurred:", msg)

    def calc_crc(self, input_data: bytes, first_byte=None) -> int:
        polynomial = 0x31
        crc = 0xFF

        data = bytearray()
        if first_byte is not None:
            data.append(first_byte)
        
        data += input_data

        for byte_i in data:
            crc ^= byte_i
            crc &= 0xFF
            for bit in range(8):
                if (crc & (1<<7)) != 0:
                    crc <<= 1
                    crc &= 0xFF
                    crc ^= polynomial
                    crc &= 0xFF
                else:
                    crc <<= 1
                    crc &= 0xFF

        return crc

    def send_packet(self, status, response_data=0):
        # status MUST be at most 4-bits
        packet = bytearray([(self.PACKET_MISO_MAGIC_NIBBLE << 4) + (status & 0xF), response_data, 0])
        packet[2] = self.calc_crc(packet[:2])

        try:
            self.backplaneI2C.send(packet, self.TRANSACTION_TIMEOUT)
        except OSError as e:
            self.raiseBusFault("Packet (status: {0}) Send Failure (errno {1})".format(status, e.args[0]))

    def receive_command(self) -> bytes:
        decoded_message = None
        while decoded_message == None:
            packet_size = 0
            # This check also intentionally prevents zero length packets from being decoded
            first_byte = 0
            while packet_size == 0:
                try:
                    header = self.backplaneI2C.recv(1, timeout=self.HEADER_TIMEOUT)
                except OSError as e:
                    if e.args[0] == ETIMEDOUT:
                        continue
                    else:
                        raise

                if header[0] >> 4 != self.PACKET_MOSI_MAGIC_NIBBLE:
                    print("Invalid Header Received: {0}".format(header[0]))
                    continue

                packet_size = header[0] & 0xF
                first_byte = header[0]
            
            try:
                remaining_packet = self.backplaneI2C.recv(packet_size+1, timeout=self.TRANSACTION_TIMEOUT)
            except OSError as e:
                if e.args[0] == ETIMEDOUT:
                    print("Timeout during second receive")
                    continue
                else:
                    raise

            test_message = remaining_packet[:-1]
            calculated_crc = self.calc_crc(test_message, first_byte)

            if calculated_crc != remaining_packet[-1]:
                print("Checksum error on message {0} received, {1} calculated".format(remaining_packet[-1], calculated_crc))
                self.send_packet(self.PACKET_STATUS_CHECKSUM)
                continue

            decoded_message = test_message
        
        return decoded_message
            
    def send_response(self, response: int):
        if type(response) != int or response < 0 or response > 0xFF:
            print("Response requested with invalid data '{0}'".format(response))
            raiseFault()
        else:
            self.send_packet(self.PACKET_STATUS_SUCCESSFUL, response_data=response)

coproComm = CoproCommunicator()


class GripperController:
    def __init__(self):
        # Setup the gripper pin for PWM
        self.gripperPin = PWM("Gripper PWM")

        # Specify gripper status LED pin
        self.gripperStatusLed = Pin('Gripper Status LED', value=0)

        # Set PWM to neutral state (1500 us pulses)
        self.set_gripper_pwm(1500)
    
    def set_gripper_pwm(self, pwm_us: int) -> bool:
        if type(pwm_us) != int:
            return False

        # Convert the thrusts in us to pulse width in percentage of 400 Hz pwm pulse
        # Calculation: (Pulse Width [us]) / ((1/400 Hz) * 1000000 [us/s]) * 100 (%)
        # This leads to (Pulse Width [us]) / 25 -> Pulse Width Percent
        pulse_width_conversion = 25

        # Make sure an invalid duty cycle isn't requested
        if pwm_us / pulse_width_conversion > 100 or pwm_us / pulse_width_conversion < 0:
            return False

        # Furthermore enforce that the pulse width percentages are valid for the gripper (See gripper documentation)
        # Although these checks aren't necessarily required for operation, they were put in since the gripper led needs to know what the gripper is doing

        # Enforce signal isn't too large for pwm open signal
        if pwm_us > 1900:
            return False
        
        # Enforce signal isn't too small for pwm close signal
        if pwm_us < 1100:
            return False
        
        # Ensure that the signal is 1500 us if it is within the neutral range
        if pwm_us <= 1530 and pwm_us >= 1470 and pwm_us != 1500:
            return False

        # Light the gripper status led if the gripper is active
        if pwm_us > 1530 or pwm_us < 1470:
            self.gripperStatusLed.value(1)
        else:
            self.gripperStatusLed.value(0)
        
        # Write the pwm value
        self.gripperPin.pulse_width_percent(pwm_us / pulse_width_conversion)

        return True

gripper = GripperController()


TIMER_STAGE_DONE = 0
TIMER_STAGE_STANDBY = 1
TIMER_STAGE_ACTIVE = 2
TIMER_STAGE_PAUSE = 3
class TorpedoController:
    TIMER_FREQUENCY = 2000

    # State of torpedo system
    torpedoArmed = False
    torpedoLaunching = False

    # Data passed between fire sequence and timer callback
    torpedoDurations = None
    torpedoPauses = None
    torpedoId = None

    # Timer state values
    timerStage = TIMER_STAGE_DONE
    timerCoilNum = 0

    def __init__(self):
        # Define timer without initializing it
        # This timer will only be used for interrupts for torpedo timing
        self.tim = Timer(12)
        
        # Specify all of the pins required
        self.coils = [
            Pin('Coil 1', value=0),
            Pin('Coil 2', value=0),
            Pin('Coil 3', value=0),
            Pin('Coil 4', value=0),
            Pin('Coil 5', value=0)
        ]
        self.S_coil = Pin('B13', value=0)

        self.torpedoGates = [
            Pin('Torpedo Gate 1', value=0),
            Pin('Torpedo Gate 2', value=0)
        ]

        # Note: The torpedos charge when pin is a digital low
        self.chargePin = Pin('Charge Pin', value=1)

        self.chargeStatusLed = Pin('Charge Status LED', value=0)
        self.firingStatusLed = Pin('Firing Status LED', value=0)
    
    def arm_torpedo(self, should_arm: bool) -> bool:
        if type(should_arm) != bool:
            return False

        if should_arm:
            self.torpedoArmed = True
            self.chargeStatusLed.value(1)
            self.chargePin.value(0)
        else:
            self.chargeStatusLed.value(0)
            self.chargePin.value(1)
            self.torpedoArmed = False

        return True
    
    def torpedo_active(self) -> bool:
        return self.torpedoLaunching

    def check_timing_valid(self, timing_value: int, is_pause: bool) -> bool:
        if type(timing_value) != int:
            return False
        
        if is_pause:
            return timing_value <= 65535 and timing_value >= 0
        else:
            # Non-pauses don't have code to handle zero duration
            return timing_value <= 65535 and timing_value > 0


    def timer_callback(self):
        if self.timerStage == TIMER_STAGE_DONE:
            # This callback shouldn't be active when the timer is done
            # Note: Printing is not allowed in interrupts since it needs dynamic memory allocation, so the only error can be the fault led
            raiseFault()
        elif self.timerStage == TIMER_STAGE_STANDBY:
            # Turn off standby coils
            self.S_coil.value(0)

            # Start the firing process
            self.timerCoilNum = 0
            self.timerStage = TIMER_STAGE_ACTIVE
            # TODO: Confirm auto-reload register will update on manual counter reset
            self.tim.period(self.torpedoDurations[self.timerCoilNum])
            self.tim.counter(0)

            # Set the coils
            self.coils[self.timerCoilNum].value(1)
        elif self.timerStage == TIMER_STAGE_ACTIVE:
            # Turn off previous coil
            self.coils[self.timerCoilNum].value(0)

            # Move to the next coil
            self.timerCoilNum += 1

            # If all coils have been fired, end the firing process
            if self.timerCoilNum >= len(self.coils):
                # Turn off timer callback
                self.timerStage = TIMER_STAGE_DONE
                self.tim.deinit()
                
                # Turn off the gate to select the torpedo
                self.torpedoGates[self.torpedoId].value(0)

                # Clean up values passed to callback function
                self.torpedoId = None
                self.torpedoPauses = None
                self.torpedoDurations = None

                # Set state to not launching
                self.torpedoLaunching = False
                self.firingStatusLed.value(0)
            else:
                if self.torpedoPauses[self.timerCoilNum-1] == 0:
                    # If there is zero pause, immediately start next coil
                    self.timerStage = TIMER_STAGE_PAUSE
                    self.timer_callback()
                else:
                    # Set the timer to pause between launches
                    self.timerStage = TIMER_STAGE_PAUSE
                    self.tim.period(self.torpedoPauses[self.timerCoilNum-1])
                    self.tim.counter(0)
        elif self.timerStage == TIMER_STAGE_PAUSE:
            # Turn on the coil
            self.coils[self.timerCoilNum].value(1)

            # Set timer to delay for given duration
            self.timerStage = TIMER_STAGE_ACTIVE
            self.tim.period(self.torpedoDurations[self.timerCoilNum])
            self.tim.counter(0)

    def fire_torpedo(self, torpedo_id: int, standby_timing: int, durations: array.array, pauses: array.array) -> bool:
        if self.torpedoLaunching:
            return False

        # All durations are in microseconds

        # Make sure arguments are valid
        if type(torpedo_id) != int or torpedo_id >= len(self.torpedoGates):
            return False

        if not self.check_timing_valid(standby_timing, False):
            return False
        
        if type(durations) != array.array or len(durations) != len(self.coils):
            return False
        
        if type(pauses) != array.array or len(pauses) != len(self.coils) - 1:
            return False
        
        for duration in durations:
            if not self.check_timing_valid(duration, False):
                return False
        
        for pause in pauses:
            if not self.check_timing_valid(pause, True):
                return False

        # Set all pins to default state
        self.S_coil.value(0)
        for coil in self.coils:
            coil.value(0)
        for gate in self.torpedoGates:
            gate.value(0)

        # Set state to launching
        self.torpedoLaunching = True
        self.firingStatusLed.value(1)

        # Specify torpedo to launch
        self.torpedoGates[torpedo_id].value(1)

        # Save variables needed in callback
        self.torpedoId = torpedo_id
        self.torpedoDurations = durations
        self.torpedoPauses = pauses

        # Setup the timing system
        self.timerStage = TIMER_STAGE_STANDBY

        source_frequency = self.tim.source_freq()
        if source_frequency % self.TIMER_FREQUENCY != 0:
            print("Sounce frequency does not divide evenly with requested TIMER_FREQUENCY")
            raiseFault()
        timer_prescaler = (source_frequency // self.TIMER_FREQUENCY) - 1

        self.tim.counter(0)
        self.tim.init(prescaler=timer_prescaler, period=standby_timing, callback=self.timer_callback)

        # Start the standby coil
        self.S_coil.value(1)

        return True

torpedo = TorpedoController()

class MarkerController:
    # Dropper Duration (seconds)
    # Keep 1/duration as even as possible (must be converted to frequency)
    dropperDuration = 0.1
    
    currentDropperActive = None

    def __init__(self):
        # Define timer without initializing it
        # Note: Timer 7 is one of the two basic timers, this can only be used with interrupts
        self.tim = Timer(7)
        
        # Specify the pins for each marker dropper
        self.dropperPins = [Pin('B14', value=0), 
                            Pin('B15', value=0)]

        self.dropperStatusLed = Pin('B0', value=0)
    
    def drop_marker(self, dropperNum: int) -> bool:
        # Make sure that the dropper is valid
        if type(dropperNum) != int or dropperNum >= len(self.dropperPins):
            return False

        # Make sure that the timer isn't currently in use
        if self.currentDropperActive is not None:
            return False
        
        # Reserve the timer and specify which dropper should be turned off after interrupt
        self.currentDropperActive = dropperNum
        
        # Start the timer with calculated frequency and specify the interrupt at 0
        self.tim.counter(0)
        self.tim.init(freq=(1.0/self.dropperDuration), callback=self.end_marker_drop)
        self.drop_start_time = time.time()

        # Set specific dropper pin and status led to HIGH
        self.dropperPins[dropperNum].value(1)
        self.dropperStatusLed.value(1)

        return True

    def end_marker_drop(self):
        # Note: This function is running in an interrupt handler
        # See micropython's documentation for what is allowed in this code

        # Stop the timer (Only way of doing so in micropython)
        self.tim.deinit()

        # Set dropper pin and status led to LOW
        self.dropperPins[self.currentDropperActive].value(0)
        self.dropperStatusLed.value(0)

        print("Done in {0} ms".format((time.time() - self.drop_start_time) * 1000))

        # Unreserve the timer
        self.currentDropperActive = None

marker = MarkerController()

def restart_mcu():
    print("Machine reset requested... Exiting simulation")
    os._exit(0)