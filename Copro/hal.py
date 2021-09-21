import board
import busio
import digitalio
import pwmio
import time
import ethernet
import uasyncio as asyncio
import errno
import gc
import microcontroller

# Uncomment the robot in use
import titanRobot as robotSpecific
#import puddlesRobot as robotSpecific

ROBOT_NAME_ENCODED = robotSpecific.ROBOT_NAME.encode()
CHIP_NAME_ENCODED = b'RP2040'

########################################
## COMMUNICATION INTERFACE CREATION   ##
########################################

def unpretty_ip(ip): return bytearray(map(lambda x: int(x), ip.split('.')))

# MAC address format
# Uses Raspberry Pi MAC address prefix: B8:27:EB
# Except: Uses xA rather than x8 for the first byte, to make it a locally administred MAC
# Then the last 3 bytes are the last 3 bytes of the microcontroller uid (which is actually
# the nor flash id, since the RP2040 doesn't have a chip id burned onto it)
# This should hopefully avoid mac address collisions since most devices will use a universal mac
# and any other RP2040s shouldn't have the same mac address, but it is NOT GAURENTEED UNIQUE
mac_address = bytearray((0xBA, 0x27, 0xEB, microcontroller.cpu.uid[5], microcontroller.cpu.uid[6], microcontroller.cpu.uid[7]))

dev = ethernet.Wiznet5K(board.GP10, board.GP11, board.GP12, board.GP13, mac_address)
dev.ifconfig(unpretty_ip(robotSpecific.IP_ADDRESS),     # IP Address
			 unpretty_ip('192.168.1.1'),                # Gateway
			 unpretty_ip('255.255.255.0'))              # Subnet Mask

commandServer = ethernet.CommandServer(dev, 2354)

class I2CMicropythonWrapper:
	def __init__(self, i2c_bus):
		self.bus = i2c_bus
	
	def send(self, data, addr):
		assert self.bus.try_lock(), "Failed to lock bus"
		self.bus.writeto(addr, data)
		self.bus.unlock()

	def recv(self, length, addr):
		assert self.bus.try_lock(), "Failed to lock bus"
		data = bytearray(length)
		self.bus.readfrom(addr, data)
		self.bus.unlock()
		return data

	def mem_read(self, bytes_to_read, addr, memaddr):
		assert self.bus.try_lock(), "Failed to lock bus"
		read_buf = bytearray(bytes_to_read)
		self.bus.writeto_then_readfrom(addr, bytearray([memaddr]), read_buf)
		self.bus.unlock()
		return read_buf

	def mem_write(self, data, addr, memaddr):
		assert self.bus.try_lock(), "Failed to lock bus"
		self.bus.writeto(addr, bytearray([memaddr]) + data)
		self.bus.unlock()

backplaneI2C_bus = busio.I2C(board.GP1, board.GP0, frequency=200000)
backplaneI2C = I2CMicropythonWrapper(backplaneI2C_bus)
robotI2C_bus = busio.I2C(board.GP7, board.GP6, frequency=200000)
robotI2C = I2CMicropythonWrapper(robotI2C_bus)


########################################
## FAULT HANDLING CODE                ##
########################################

faultLed = digitalio.DigitalInOut(board.GP4)
faultLed.switch_to_output(value= False)

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
UNEXPECTED_NETWORK_ERROR = 13
KILL_SWITCH_MONITOR_CRASH = 14

# When this bit it set, the following 7 bits are the command number for fault
COMMAND_EXEC_CRASH_FLAG = (1<<7)

faultList = []
def raiseFault(faultId: int):
	faultLed.switch_to_output(value = True)
	if faultId not in faultList:
		faultList.append(faultId)

def lowerFault(faultId: int):
	if faultId in faultList:
		faultList.remove(faultId)
	if len(faultList) == 0:
		faultLed.switch_to_output(value = True)

if microcontroller.cpu.reset_reason  == microcontroller.ResetReason.WATCHDOG:
	raiseFault(WATCHDOG_RESET)

########################################
## UTILITY CODE                       ##
########################################

def getTime():
	return int(time.monotonic() * 1000)

def getTimeDifference(end, start):
	return (end - start)

class Sensor:
	def __init__(self, collectFunction):
		self.lastCollectionTime = -1
		self.cacheDuration = 100
		self.collectFunction = collectFunction

	def value(self):
		if self.lastCollectionTime == -1 or getTimeDifference(getTime(), self.lastCollectionTime) > self.cacheDuration:
			self.collect()
		return self.data

	def collect(self):
		self.data = self.collectFunction()
		self.lastCollectionTime = getTime()


########################################
## COPRO IMPLEMENTED INTERFACES       ##
########################################

class BBBoard:
	deviceAddress = 0x2F
	numLights = 2
	currentLightValues = [0, 0]
	initialized = False

	# Devices that are initialized differently on robots
	peltierPower: digitalio.DigitalInOut # Should be initialized for both robots, but they use different pins
	# Only initialized on titan
	light1: 'pwmio.PWMOut | None' = None
	light2: 'pwmio.PWMOut | None' = None

	def __init__(self):
		try:
			# Setup power control pins
			self.moboPower = digitalio.DigitalInOut(board.GP15)
			self.moboPower.switch_to_output(value=True)
			self.threePower = digitalio.DigitalInOut(board.GP13)
			self.threePower.switch_to_output(value=True)
			self.fivePower = digitalio.DigitalInOut(board.GP24)
			self.fivePower.switch_to_output(value=True)
			self.twelvePower = digitalio.DigitalInOut(board.GP20)
			self.twelvePower.switch_to_output(value=True)

			robotSpecific.bbInitCode(self)

			# Initialize adc for voltage and current reading from Battery Balancer Board
			while robotI2C.mem_read(1, BBBoard.deviceAddress, 0x0C)[0] & 0b00000010 != 0:
				pass
			# Operational mode 0 (includes temperature) and external vref
			robotI2C.mem_write(chr(0b001), BBBoard.deviceAddress, 0x0B)
			# Set continuous conversion
			robotI2C.mem_write(chr(1), BBBoard.deviceAddress, 0x07)
			# Disable unused channels
			robotI2C.mem_write(chr(0b01100000), BBBoard.deviceAddress, 0x08)
			# Mask all interrupts
			robotI2C.mem_write(chr(0xFF), BBBoard.deviceAddress, 0x03)
			# Start ADC and disable interrupts
			robotI2C.mem_write(chr(1), BBBoard.deviceAddress, 0x00)
			self.initialized = True
		except Exception as e: 
			print("Error on BB init: " + str(e))
			raiseFault(BB_INIT_FAIL)

	def setLight1(self, value: int) -> bool:
		# Support for puddles without lights
		if self.light1 is None:
			return False

		if value > 100 or value < 0:
			return False

		self.light1.duty_cycle = int((value/100.0) * ((2**16)-1))
		self.currentLightValues[0] = value

		return True

	def setLight2(self, value: int) -> bool:
		# Support for puddles without lights
		if self.light2 is None:
			return False

		if value > 100 or value < 0:
			return False

		self.light2.duty_cycle = int((value/100.0) * ((2**16)-1))
		self.currentLightValues[1] = value

		return True

	# Callback functions, don't have access to class variables
	@staticmethod
	def getStbdCurrent():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x20)
		voltage = (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096
		return max((voltage - .33) / .066, 0)
	@staticmethod
	def getPortCurrent():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x21)
		voltage = (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096
		return max((voltage - .33) / .066, 0)
	@staticmethod
	def getBalancedVolt():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x22)
		return (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096 * (118 / 18)
	@staticmethod
	def getStbdVolt():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x23)
		return (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096 * (118 / 18)* .984
	@staticmethod
	def getPortVolt():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x24)
		return (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096 * (118 / 18)* .984
	@staticmethod
	def getFiveVolt():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x25)
		return (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096 * (118 / 18)
	@staticmethod
	def getTwelveVolt():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x26)
		return (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096
	@staticmethod
	def getTemp():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x27)
		return ((data[0] << 8) + data[1]) / 256

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

	# See actuator firmware for protocol breakdown
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
				backplaneI2C.send(header, ActuatorBoard.actuatorAddress)
				backplaneI2C.send(data, ActuatorBoard.actuatorAddress)
			except OSError as e:
				if e.args[0] == errno.ETIMEDOUT:
					# print("Send Timed Out")
					continue
				else:
					raise
			
			recv_data = None
			try:
				recv_data = backplaneI2C.recv(3, ActuatorBoard.actuatorAddress)
			except OSError as e:
				if e.args[0] == errno.ETIMEDOUT:
					# print("Receive Timed Out")
					continue
				else:
					raise
			
			if recv_data[0] >> 4 != ActuatorBoard.PACKET_MISO_MAGIC_NIBBLE:
				# print("Invalid Packet Header")
				continue

			if self.calc_crc(recv_data[:2]) != recv_data[2]:
				# print("Invalid Checksum")
				continue

			status = recv_data[0] & 0xF

			if status == ActuatorBoard.PACKET_STATUS_SUCESSFUL:
				cmdStatus = recv_data[1]
				successful = True
				break
			elif status == ActuatorBoard.PACKET_STATUS_CHECKSUM:
				# print("Actuator requested packet resend from checksum error")
				continue
			else:
				# print("Unexpected status code... Trying again")
				continue
		
		if successful:
			return [cmdStatus, tries]
		else:
			tries = 0xFF
			return [0, tries]


class ESCBoard():
	numThrusters = 8
	deviceAddress = 0x2E
	thrustersEnabled = 1
	currentThrusts: 'list[int]' = []  # The current pwm pulse width in microseconds
	initialized = False

	def __init__(self):
		try:
			self.thrusters: 'list[pwmio.PWMOut]' = []

			# Get the thruster configuration for the specific robot
			robotSpecific.escInitCode(self)

			assert len(self.thrusters) == ESCBoard.numThrusters
			self.stopThrusters()
			# Set the time when the kill switch position is changed
			self.timeChange = 0

			while backplaneI2C.mem_read(1, ESCBoard.deviceAddress, 0x0C)[0] & 0b00000010 != 0:
				pass
			# Operational mode 1 (excludes temperature) and external vref
			backplaneI2C.mem_write(chr(0b011), ESCBoard.deviceAddress, 0x0B)
			# Set continuous conversion
			backplaneI2C.mem_write(chr(1), ESCBoard.deviceAddress, 0x07)
			# Disable unused channels
			backplaneI2C.mem_write(chr(0b00000000), ESCBoard.deviceAddress, 0x08)
			# Mask all interrupts
			backplaneI2C.mem_write(chr(0xFF), ESCBoard.deviceAddress, 0x03)
			# Start ADC and disable interrupts
			backplaneI2C.mem_write(chr(1), ESCBoard.deviceAddress, 0x00)
			self.initialized = True
		except Exception as e:
			print("Error on ESC init: " + str(e))
			raiseFault(ESC_INIT_FAIL)

	@staticmethod
	def getCurrents():
		current_vals = []
		for i in range(8):
			data = backplaneI2C.mem_read(2, ESCBoard.deviceAddress, 0x20 + i)
			voltage = (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096
			current_vals.append(max((voltage - .33) / .264, 0))
		return current_vals

	def stopThrusters(self) -> bool:
		# Set the pwm pulse width to 1500 us (60% of a 400hz pwm signal) to put ESC in stopped position
		# See https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/ for the documentation
		self.currentThrusts = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
		for t in self.thrusters:
			t = pwmio.PWMOut(board.GP17, frequency=10000, duty_cycle=60)
			
		return True

	def setThrusters(self, thrusts) -> bool:
		if self.thrustersEnabled and Backplane.killSwitch.value == 0 and getTimeDifference(getTime(), self.timeChange) > 5000:
			if len(thrusts) != ESCBoard.numThrusters:
				return False

			# Convert the thrusts in us to pulse width in percentage of 400 Hz pwm pulse
			# Calculation: (Pulse Width [us]) / ((1/400 Hz) * 1000000 [us/s]) * 65535 (%)
			# This leads to (Pulse Width [us]) * 26.214 -> Pulse Width Percent
			pulse_width_conversion = 26.214

			# Make sure an invalid duty cycle isn't requested
			for i in range(ESCBoard.numThrusters):
				if int(thrusts[i] * pulse_width_conversion) > 65535 or int(thrusts[i] * pulse_width_conversion) < 0:
					return False
			
			for i in range(ESCBoard.numThrusters):
				value = int(thrusts[i] * pulse_width_conversion)
				self.thrusters[i].duty_cycle = value
			
			self.currentThrusts = thrusts
			return True

		return False

	def setThrusterEnable(self, enable) -> bool:
		if enable == 1 or enable == 0:
			self.thrustersEnabled = enable
			if not enable:
				self.stopThrusters()
			return True
		return False

	currents = Sensor(getCurrents)


class DepthSensor():
	deviceAddress = 0x76
	_fluidDensity = 997
	_pressure = 0
	surfacePressure = -1
	initialized = False

	def __init__(self):
		try:
			robotI2C.send(chr(0x1E), self.deviceAddress)
			time.sleep(0.01)
			self._C = []

			# Read calibration and crc
			for i in range(7):
				c = robotI2C.mem_read(2, self.deviceAddress, 0xA0 + 2*i)
				time.sleep(0.01)
				c = (c[0] << 8) + c[1]
				#c =  ((c & 0xFF) << 8) | (c >> 8) # SMBus is little-endian for word transfers, we need to swap MSB and LSB
				self._C.append(c)

			assert (self._C[0] & 0xF000) >> 12 == self.crc4(self._C), "PROM read error, CRC failed!"

			self.initialized = True
		except Exception as e:
			print("Error on Depth init: " + str(e))
			raiseFault(DEPTH_INIT_FAIL)

	# Cribbed from datasheet
	def crc4(self, n_prom):
		n_rem = 0

		n_prom[0] = ((n_prom[0]) & 0x0FFF)
		n_prom.append(0)

		for i in range(16):
			if i%2 == 1:
				n_rem ^= ((n_prom[i>>1]) & 0x00FF)
			else:
				n_rem ^= (n_prom[i>>1] >> 8)

			for n_bit in range(8,0,-1):
				if n_rem & 0x8000:
					n_rem = (n_rem << 1) ^ 0x3000
				else:
					n_rem = (n_rem << 1)

		n_rem = ((n_rem >> 12) & 0x000F)

		self.n_prom = n_prom
		self.n_rem = n_rem

		return n_rem ^ 0x00

	# Cribbed from datasheet
	def calculate(self):
		OFFi = 0
		SENSi = 0
		Ti = 0

		dT = self._D2-self._C[5]*256
		SENS = self._C[1]*32768+(self._C[3]*dT)/256
		OFF = self._C[2]*65536+(self._C[4]*dT)/128
		self._pressure = (self._D1*SENS/(2097152)-OFF)/(8192)

		self._temperature = 2000+dT*self._C[6]/8388608

		# Second order compensation

		if (self._temperature/100) < 20: # Low temp
			Ti = (3*dT*dT)/(8589934592)
			OFFi = (3*(self._temperature-2000)*(self._temperature-2000))/2
			SENSi = (5*(self._temperature-2000)*(self._temperature-2000))/8
			if (self._temperature/100) < -15: # Very low temp
				OFFi = OFFi+7*(self._temperature+1500)*(self._temperature+1500)
				SENSi = SENSi+4*(self._temperature+1500)*(self._temperature+1500)
		elif (self._temperature/100) >= 20: # High temp
			Ti = 2*(dT*dT)/(137438953472)
			OFFi = (1*(self._temperature-2000)*(self._temperature-2000))/16
			SENSi = 0

		OFF2 = OFF-OFFi
		SENS2 = SENS-SENSi

		self._temperature = (self._temperature-Ti)
		self._pressure = (((self._D1*SENS2)/2097152-OFF2)/8192)/10.0

	async def read(self):
		oversampling = 5

		# Request D1 conversion (temperature)
		robotI2C.send(chr(0x40 + 2*oversampling), self.deviceAddress)

		# Maximum conversion time increases linearly with oversampling
		# max time (seconds) ~= 2.2e-6(x) where x = OSR = (2^8, 2^9, ..., 2^13)
		# We use 2.5e-6 for some overhead
		await asyncio.sleep_ms(int(2.5e-3 * 2**(8+oversampling)) + 2)

		d = robotI2C.mem_read(3, self.deviceAddress, 0x00)
		self._D1 = d[0] << 16 | d[1] << 8 | d[2]

		# Request D2 conversion (pressure)
		robotI2C.send(chr(0x50 + 2*oversampling), self.deviceAddress)

		# As above
		await asyncio.sleep_ms(int(2.5e-3 * 2**(8+oversampling)) + 2)

		d = robotI2C.mem_read(3, self.deviceAddress, 0x00)
		self._D2 = d[0] << 16 | d[1] << 8 | d[2]

		self.calculate()

	def pressure(self):
		return self._pressure

	def temperature(self):
		degC = self._temperature / 100.0
		return degC

	# Depth relative to MSL pressure in given fluid density
	def depth(self):
		return ((self.pressure() - self.surfacePressure)*100)/(self._fluidDensity*9.80665)

	async def zeroDepth(self):
		await self.read()
		await self.read()

		if self.surfacePressure == -1:
			self.surfacePressure = self._pressure

		self.surfacePressure = self.surfacePressure * .7 + self._pressure * .3


class CoproBoard():
	wdt_enabled = False

	def restart(self):
		microcontroller.reset()
	def start_watchdog(self):
		if microcontroller.watchdog is not None:
			microcontroller.watchdog.timeout = 5
			self.wdt_enabled = True
	def feed_watchdog(self):
		if self.wdt_enabled and microcontroller.watchdog is not None:
			microcontroller.watchdog.feed()
	def memory_usage(self):
		gc.collect()
		free_memory = gc.mem_free()
		occupy_memory = gc.mem_alloc()
		total_memory = free_memory+occupy_memory
		percent_usage = free_memory/total_memory

		return percent_usage


class BackplaneBoard():
	def __init__(self):
		try:
			self.killSwitch = digitalio.DigitalInOut(board.GP28)
			self.killSwitch.switch_to_input(pull=digitalio.Pull.UP)
			self.auxSwitch = digitalio.DigitalInOut(board.GP27)
			self.auxSwitch.switch_to_input(pull=digitalio.Pull.UP)
		except Exception as e:
			print("Error on Backplane init: " + str(e))
			raiseFault(BACKPLANE_INIT_FAIL)

########################################
## COPRO INTERFACES CREATION          ##
########################################

BB = BBBoard()
Actuator = ActuatorBoard()
ESC = ESCBoard()
Depth = DepthSensor()
Copro = CoproBoard()
Backplane = BackplaneBoard()
