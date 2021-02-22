import socket
import random
import time
import os

########################################
## SIMULATOR DEVICE CREATION          ##
########################################

class Pin:
	registered_names = []

	def __init__(self, name, default_state = 0):
		if name in Pin.registered_names:
			raise RuntimeError("Attempting to register pin '{0}', which has already been registered".format(name))
		Pin.registered_names.append(name)
		self.name = name
		self.state = default_state

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

	def off(self):
		self.value(0)
	
	def on(self):
		self.value(1)

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


ETIMEDOUT = 110
class ActuatorI2C:
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

	def __init__(self):
		# Setup socket and such
		try:
			self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.sock.connect(("localhost", ActuatorI2C.I2C_SIMULATOR_PORT))
		except:
			print("Failed to initialize actuator connection")

	def _recvall(self, size: int) -> bytes:
		data = bytearray()
		while len(data) < size:
			recv_part = self.sock.recv(size - len(data))
			if len(recv_part) == 0:
				raise ActuatorI2C.ProtocolError("Connection Unexpectedly Closed")
			data += recv_part
		return data

	def _sock_read_tlv(self, expected_type: int) -> bytes:
		tl_part = self._recvall(2)

		if tl_part[0] != expected_type:
			self.sock.close()
			raise ActuatorI2C.ProtocolError("Unexpected Packet Type {0}, expected {1}".format(tl_part[0], expected_type))

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
		

	def send(self, data: bytes, addr=0, timeout=5000):
		assert len(data) < 256
		if addr != ActuatorI2C.EXPECTED_ADDR:
			raise RuntimeError("Only the actuators are implemented in the simulator")

		try:
			self.sock.settimeout(timeout / 1000)
			self._sock_send_tlv(ActuatorI2C.MOSI_TX_CONTENT, data)
			ack = self._sock_read_tlv(ActuatorI2C.MISO_RX_ACK)
			if len(ack) != 1 or ack[0] != len(data):
				raise ActuatorI2C.ProtocolError("Invalid ack packet")
		except socket.timeout:
			# Make an error that is expected during normal I2C operation
			raise OSError(ETIMEDOUT)
		except OSError:
			raise OSError(ETIMEDOUT)

	def recv(self, size: int, addr=0, timeout=5000):
		assert size > 0 and size < 256
		if addr != ActuatorI2C.EXPECTED_ADDR:
			raise RuntimeError("Only the actuators are implemented in the simulator")
		
		data = None
		try:
			self.sock.settimeout(timeout / 1000)
			self._sock_send_tlv(ActuatorI2C.MOSI_RX_REQUEST, bytearray([size]))
			data = self._sock_read_tlv(ActuatorI2C.MISO_TX_CONTENT)
			if len(data) != size:
				raise ActuatorI2C.ProtocolError("Invalid size response sent")
			self._sock_send_tlv(ActuatorI2C.MOSI_RX_ACK, None)
		except socket.timeout:
			# Make an error that is expected during normal I2C operation
			raise OSError(ETIMEDOUT)
		except OSError:
			raise OSError(ETIMEDOUT)
		return data

backplaneI2C = ActuatorI2C()

########################################
## FAULT HANDLING CODE                ##
########################################

faultLed = Pin("Fault LED")
faultLed.off()

PROGRAM_TERMINATED = 1
MAIN_LOOP_CRASH = 2
DEPTH_LOOP_CRASH = 3
BATTERY_CHECKER_CRASH = 4
AUTO_COOLING_CRASH = 5
BB_INIT_FAIL = 6
ESC_INIT_FAIL = 7
DEPTH_INIT_FAIL = 8
BACKPLANE_INIT_FAIL = 9
FAULT_STATE_INVALID = 10
BATT_LOW = 11
WATCHDOG_RESET = 12

# When this bit it set, the following 7 bits are the command number for fault
COMMAND_EXEC_CRASH_FLAG = (1<<7)

faultList = []
def raiseFault(faultId: int):
	faultLed.on()
	if faultId not in faultList:
		faultList.append(faultId)

def lowerFault(faultId: int):
	if faultId in faultList:
		faultList.remove(faultId)
	if len(faultList) == 0:
		faultLed.off()

########################################
## UTILITY CODE                       ##
########################################

def getTime():
    return int(time.time()*1000)

class Sensor:
	def __init__(self, collectFunction):
		self.data = 0
		self.lastCollectionTime = 0
		self.cacheDuration = 100
		self.collectFunction = collectFunction

	def value(self):
		if getTime() - self.lastCollectionTime > self.cacheDuration:
			self.collect()
		return self.data

	def collect(self):
		self.data = self.collectFunction()
		self.lastCollectionTime = getTime()


########################################
## COPRO IMPLEMENTED INTERFACES       ##
########################################

class BBBoard:
	numLights = 2
	currentLightValues = [0, 0]
	initialized = False

	def __init__(self):
		try:
			# Setup power control pins
			self.moboPower = Pin("Mobo Power")
			self.peltierPower = Pin("Peltier Power")
			self.threePower = Pin("3.3V Power")
			self.fivePower = Pin("5V Power")
			self.twelvePower = Pin("12V Power")
			
			# Setup ligting power for pwm control (For that fancy dimming)
			self.light1 = PWM("Lighting 1")
			self.light2 = PWM("Lighting 2")
			self.setLight1(0)
			self.setLight2(0)

			# Initialize adc for voltage and current reading from Battery Balancer Board
			# Nothing to do, we are simulating it
			self.initialized = True
		except Exception as e: 
			print("Error on BB init: " + str(e))
			raiseFault(BB_INIT_FAIL)

	def setLight1(self, value: int) -> bool:
		if value > 100 or value < 0:
			return False

		self.light1.pulse_width_percent(value)
		self.currentLightValues[0] = value

		return True

	def setLight2(self, value: int) -> bool:
		if value > 100 or value < 0:
			return False

		self.light2.pulse_width_percent(value)
		self.currentLightValues[1] = value

		return True

	# Callback functions, don't have access to class variables
	def getStbdCurrent():
		return random.uniform(0,35)
	def getPortCurrent():
		return random.uniform(0,35)
	def getBalancedVolt():
		return random.uniform(19,21)
	def getStbdVolt():
		return random.uniform(19,21)
	def getPortVolt():
		return random.uniform(19,21)
	def getFiveVolt():
		return random.uniform(4.9,5.1)
	def getTwelveVolt():
		return random.uniform(11.9,12.1)
	def getTemp():
		return random.uniform(0,70)

	stbdCurrent = Sensor(getStbdCurrent)
	portCurrent = Sensor(getPortCurrent)
	balancedVolt = Sensor(getBalancedVolt)
	stbdVolt = Sensor(getStbdVolt)
	portVolt = Sensor(getPortVolt)
	fiveVolt = Sensor(getFiveVolt)
	twelveVolt = Sensor(getTwelveVolt)
	temp = Sensor(getTemp)


class ActuatorBoard:
	actuatorAddress = 0x1C
	ACTUATOR_TIMEOUT = 2
	MAX_RETRIES = 5

	PACKET_MOSI_MAGIC_NIBBLE = 0b1001
	PACKET_MISO_MAGIC_NIBBLE = 0b1010
	PACKET_STATUS_SUCESSFUL = 1
	PACKET_STATUS_CHECKSUM = 2
	
	def __init__(self):
		self.initialized = True

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

	def actuators(self, args):
		if len(args) > 15:
			return [0, 0]

		header = bytearray([ActuatorBoard.PACKET_MOSI_MAGIC_NIBBLE << 4 | len(args) & 0xF])
		data = bytearray(len(args) + 1)
		data[0:len(args)] = args
		data[-1] = self.calc_crc(data[:-1], header[0])

		tries = 0
		successful = False
		cmdStatus = 0
		while tries < self.MAX_RETRIES:
			tries += 1
			try:
				backplaneI2C.send(header, ActuatorBoard.actuatorAddress, timeout=ActuatorBoard.ACTUATOR_TIMEOUT)
				backplaneI2C.send(data, ActuatorBoard.actuatorAddress, timeout=ActuatorBoard.ACTUATOR_TIMEOUT)
			except OSError as e:
				if e.args[0] == ETIMEDOUT:
					continue
				else:
					raise
			
			recv_data = None
			try:
				recv_data = backplaneI2C.recv(3, ActuatorBoard.actuatorAddress, timeout=ActuatorBoard.ACTUATOR_TIMEOUT)
			except OSError as e:
				if e.args[0] == ETIMEDOUT:
					continue
				else:
					raise
			
			if recv_data[0] >> 4 != ActuatorBoard.PACKET_MISO_MAGIC_NIBBLE:
				continue

			if self.calc_crc(recv_data[:2]) != recv_data[2]:
				continue

			status = recv_data[0] & 0xF

			if status == ActuatorBoard.PACKET_STATUS_SUCESSFUL:
				cmdStatus = recv_data[1]
				successful = True
				break
			elif status == ActuatorBoard.PACKET_STATUS_CHECKSUM:
				continue
			else:
				continue
		
		if successful:
			return [cmdStatus, tries]
		else:
			tries = 0xFF
			return [0, tries]


class ESCBoard():
	numThrusters = 8
	thrustersEnabled = 1
	currentThrusts = []  # The current pwm pulse width in microseconds
	initialized = False

	def __init__(self):
		try:
			self.thrusters = [
				PWM("Thruster heave_stbd_aft"),
				PWM("Thruster heave_stbd_fwd"),
				PWM("Thruster vector_stbd_fwd"),
				PWM("Thruster vector_stbd_aft"),
				PWM("Thruster heave_port_fwd"),
				PWM("Thruster heave_port_aft"),
				PWM("Thruster vector_port_fwd"),
				PWM("Thruster vector_port_aft"),
			]
			assert len(self.thrusters) == ESCBoard.numThrusters
			self.stopThrusters()
			# Set the time when the kill switch position is changed
			self.timeChange = 0

			# I2C init removed
			self.initialized = True
		except Exception as e:
			print("Error on ESC init: " + str(e))
			raiseFault(ESC_INIT_FAIL)

	def getCurrents():
		current_vals = []
		for i in range(8):
			current_vals.append(random.uniform(0,10))
		return current_vals

	def stopThrusters(self) -> bool:
		# Set the pwm pulse width to 1500 us (60% of a 400hz pwm signal) to put ESC in stopped position
		# See https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/ for the documentation
		self.currentThrusts = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
		for t in self.thrusters:
				t.pulse_width_percent(60)
		return True

	def setThrusters(self, thrusts) -> bool:
		if self.thrustersEnabled and Backplane.killSwitch.value() == 0 and getTime() - self.timeChange > 5000:
			if len(thrusts) != ESCBoard.numThrusters:
				return False

			# Convert the thrusts in us to pulse width in percentage of 400 Hz pwm pulse
			# Calculation: (Pulse Width [us]) / ((1/400 Hz) * 1000000 [us/s]) * 100 (%)
			# This leads to (Pulse Width [us]) / 25 -> Pulse Width Percent
			pulse_width_conversion = 25

			# Make sure an invalid duty cycle isn't requested
			for i in range(ESCBoard.numThrusters):
				if thrusts[i] / pulse_width_conversion > 100 or thrusts[i] / pulse_width_conversion < 0:
					return False
			
			for i in range(ESCBoard.numThrusters):
				value = thrusts[i] / pulse_width_conversion
				self.thrusters[i].pulse_width_percent(value)
			
			self.currentThrusts = thrusts
			return True

		return False

	def setThrusterEnable(self, enable) -> bool:
		if enable == 1 or enable == 0:
			self.thrustersEnabled = enable
			self.stopThrusters()
			return True
		return False

	currents = Sensor(getCurrents)


class DepthSensor():
	deviceAddress = 0x76
	_fluidDensity = 997
	_pressure = 0
	initialized = False

	def __init__(self):
		self.initialized = True

	def pressure(self):
		return self._pressure
		
	def temperature(self):
		degC = self._temperature / 100.0
		return degC
		
	# Depth relative to MSL pressure in given fluid density
	def depth(self):
		return (random.uniform(0, 2))


class CoproBoard():
	def restart(self):
		print("Machine reset requested... Exiting simulation")
		os._exit(0)

	def start_watchdog(self):
		pass

	def feed_watchdog(self):
		pass

	def memory_usage(self):
		"""gc.collect()
		free_memory = gc.mem_free()
		occupy_memory = gc.mem_alloc()
		total_memory = free_memory+occupy_memory
		percent_usage = free_memory/total_memory"""
		percent_usage = random.uniform(0, 1)

		return percent_usage


class BackplaneBoard():
	def __init__(self):
		try:
			self.killSwitch = Pin("Kill Switch")
			self.auxSwitch = Pin("Aux Switch")

			#self.killSwitch.irq(self.killSwitchChanged)
			self.killSwitchChanged(self.killSwitch)
		except Exception as e:
			print("Error on Backplane init: " + str(e))
			raiseFault(BACKPLANE_INIT_FAIL)

	def killSwitchChanged(self, pin):
		ESC.stopThrusters()
		ESC.timeChange = getTime()

########################################
## COPRO INTERFACES CREATION          ##
########################################

BB = BBBoard()
Actuator = ActuatorBoard()
ESC = ESCBoard()
Depth = DepthSensor()
Copro = CoproBoard()
Backplane = BackplaneBoard()
