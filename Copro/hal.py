import machine
import utime as time
import network
from pyb import Timer, Pin, I2C, LED
import uasyncio as asyncio
import uerrno
import gc

########################################
## COMMUNICATION INTERFACE CREATION   ##
########################################

nic = network.WIZNET5K(machine.SPI(1), machine.Pin('A4', machine.Pin.OUT), machine.Pin('C5', machine.Pin.OUT))
nic.ifconfig(('192.168.1.43', '255.255.255.0', '192.168.1.1', '8.8.8.8'))

backplaneI2C = I2C(1, I2C.MASTER, baudrate=200000)
robotI2C = I2C(2, I2C.MASTER, baudrate=200000)


########################################
## FAULT HANDLING CODE                ##
########################################

faultLed = LED(1)
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

if machine.reset_cause() == machine.WDT_RESET:
	raiseFault(WATCHDOG_RESET)

########################################
## UTILITY CODE                       ##
########################################

def getTime():
	return time.ticks_ms()

class Sensor:
	def __init__(self, collectFunction):
		self.data = 0
		self.lastCollectionTime = 0
		self.cacheDuration = 100
		self.collectFunction = collectFunction

	def value(self):
		if time.ticks_diff(getTime(), self.lastCollectionTime) > self.cacheDuration:
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

	def __init__(self):
		try:
			# Setup power control pins
			self.moboPower = machine.Pin('C2', machine.Pin.OUT, value=1)
			self.peltierPower = machine.Pin('B14', machine.Pin.OUT, value=1)
			self.threePower = machine.Pin('C0', machine.Pin.OUT, value=1)
			self.fivePower = machine.Pin('C13', machine.Pin.OUT, value=1)
			self.twelvePower = machine.Pin('B0', machine.Pin.OUT, value=1)
			
			# Setup ligting power for pwm control (For that fancy dimming)
			self.tim4 = Timer(4, freq=400)
			self.light1 = self.tim4.channel(3, Timer.PWM, pin=Pin('B8')),
			self.light2 = self.tim4.channel(4, Timer.PWM, pin=Pin('B9'))
			self.setLight1(0)
			self.setLight2(0)

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
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x20)
		voltage = (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096
		return max((voltage - .33) / .066, 0)
	def getPortCurrent():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x21)
		voltage = (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096
		return max((voltage - .33) / .066, 0)
	def getBalancedVolt():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x22)
		return (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096 * (118 / 18)
	def getStbdVolt():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x23)
		return (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096 * (118 / 18)* .984
	def getPortVolt():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x24)
		return (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096 * (118 / 18)* .984
	def getFiveVolt():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x25)
		return (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096 * (118 / 18)
	def getTwelveVolt():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x26)
		return (((data[0] << 8) + data[1]) >> 4) * 3.3 / 4096
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
				backplaneI2C.send(header, ActuatorBoard.actuatorAddress, timeout=ActuatorBoard.ACTUATOR_TIMEOUT)
				backplaneI2C.send(data, ActuatorBoard.actuatorAddress, timeout=ActuatorBoard.ACTUATOR_TIMEOUT)
			except OSError as e:
				if e.args[0] == uerrno.ETIMEDOUT:
					# print("Send Timed Out")
					continue
				else:
					raise
			
			recv_data = None
			try:
				recv_data = backplaneI2C.recv(3, ActuatorBoard.actuatorAddress, timeout=ActuatorBoard.ACTUATOR_TIMEOUT)
			except OSError as e:
				if e.args[0] == uerrno.ETIMEDOUT:
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
	currentThrusts = []  # The current pwm pulse width in microseconds
	initialized = False

	def __init__(self):
		try:
			self.tim2 = Timer(2, freq=400)
			self.tim8 = Timer(8, freq=400)

			self.thrusters = [
				self.tim2.channel(1, Timer.PWM, pin=Pin('A0')),
				self.tim2.channel(2, Timer.PWM, pin=Pin('A1')),
				self.tim2.channel(3, Timer.PWM, pin=Pin('A2')),
				self.tim2.channel(4, Timer.PWM, pin=Pin('A3')),
				self.tim8.channel(1, Timer.PWM, pin=Pin('C6')),
				self.tim8.channel(2, Timer.PWM, pin=Pin('C7')),
				self.tim8.channel(3, Timer.PWM, pin=Pin('C8')),
				self.tim8.channel(4, Timer.PWM, pin=Pin('C9'))
			]
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
			t.pulse_width_percent(60)
		return True

	def setThrusters(self, thrusts) -> bool:
		if self.thrustersEnabled and Backplane.killSwitch.value() == 0 and time.ticks_diff(getTime(), self.timeChange) > 5000:
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
	wdt = None

	def restart(self):
		machine.reset()
	def start_watchdog(self):
		self.wdt = machine.WDT(timeout=2000)
	def feed_watchdog(self):
		if self.wdt is not None:
			self.wdt.feed()
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
			self.killSwitch = machine.Pin('B12', machine.Pin.IN, machine.Pin.PULL_UP)
			self.auxSwitch = machine.Pin('B13', machine.Pin.IN, machine.Pin.PULL_UP)

			self.killSwitch.irq(self.killSwitchChanged)
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
