import machine
import utime as time
import network
from pyb import Timer, Pin, I2C, LED
import uasyncio as asyncio
import gc

nic = network.WIZNET5K(machine.SPI(1), machine.Pin('A4', machine.Pin.OUT), machine.Pin('C5', machine.Pin.OUT))
nic.ifconfig(('192.168.1.43', '255.255.255.0', '192.168.1.1', '8.8.8.8'))

backplaneI2C = I2C(1, I2C.MASTER, baudrate=200000)
robotI2C = I2C(2, I2C.MASTER, baudrate=200000)

# Instead of directly addressing the pin, this will be using the built-in LEDs for micropython
# This has the advantage of the fault led turning on in the event of a processor exception
faultLed = LED(1)
faultLed.off()

def raiseFault():
	faultLed.on()

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



class BBBoard:
	deviceAddress = 0x2F
	numLights = 2
	currentLightValues = [0, 0]

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
		except Exception as e: 
			print("Error on BB init: " + str(e))
			raiseFault()

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
	def getTemp():
		data = robotI2C.mem_read(2, BBBoard.deviceAddress, 0x27)
		return ((data[0] << 8) + data[1]) / 256

	stbdCurrent = Sensor(getStbdCurrent)
	portCurrent = Sensor(getPortCurrent)
	balancedVolt = Sensor(getBalancedVolt)
	stbdVolt = Sensor(getStbdVolt)
	portVolt = Sensor(getPortVolt)
	temp = Sensor(getTemp)

BB = BBBoard()

class ActuatorBoard:
	actuatorAddress = 0x1C

	def __init__(self):
		pass

	def actuators(self, args):
		backplaneI2C.send(bytearray(args), ActuatorBoard.actuatorAddress)
		#data = backplaneI2C.recv(1, ConvBoard.actuatorAddress)
		return [1]#list(backplaneI2C.recv(1, ConvBoard.actuatorAddress))
	
Actuator = ActuatorBoard()

class ESCBoard():
	numThrusters = 8
	deviceAddress = 0x2E
	thrustersEnabled = 1
	currentThrusts = []  # The current pwm pulse width in microseconds

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
		except Exception as e:
			print("Error on ESC init: " + str(e))
			raiseFault()

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
		if self.thrustersEnabled and killSwitch.value() == 0 and time.ticks_diff(getTime(), self.timeChange) > 5000:
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

ESC = ESCBoard()


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
			raiseFault()

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

Depth = DepthSensor()

class CoproBoard():
    def restart(self):
        machine.reset()
#<--TODO: check The memory usage-->
    def memory_usage(self):
        gc.collect()
        free_memory = gc.mem_free()
        occupy_memory = gc.mem_alloc()
        total_memory = free_memory+occupy_memory
        percent_usage = free_memory/total_memory

        return percent_usage

Copro = CoproBoard()


killSwitch = machine.Pin('B12', machine.Pin.IN, machine.Pin.PULL_UP)
auxSwitch = machine.Pin('B13', machine.Pin.IN, machine.Pin.PULL_UP)

def killSwitchChanged(pin):
	ESC.stopThrusters()
	ESC.timeChange = getTime()

killSwitch.irq(killSwitchChanged)
killSwitchChanged(killSwitch)
