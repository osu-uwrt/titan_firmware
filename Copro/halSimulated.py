import random
import time
import os

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




faultLed = Pin("Fault LED")

def raiseFault():
	faultLed.value(1)

class BBBoard:
	numLights = 2
	currentLightValues = [0, 0]

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
		return random.uniform(0,35)
	def getPortCurrent():
		return random.uniform(0,35)
	def getBalancedVolt():
		return random.uniform(19,21)
	def getStbdVolt():
		return random.uniform(19,21)
	def getPortVolt():
		return random.uniform(19,21)
	def getTemp():
		return random.uniform(0,70)

	stbdCurrent = Sensor(getStbdCurrent)
	portCurrent = Sensor(getPortCurrent)
	balancedVolt = Sensor(getBalancedVolt)
	stbdVolt = Sensor(getStbdVolt)
	portVolt = Sensor(getPortVolt)
	temp = Sensor(getTemp)

BB = BBBoard()

class ActuatorBoard:
	def __init__(self):
		pass

	def actuators(self, args):
		print("Command requested to actuator:", list(args))
		return [1]
	
Actuator = ActuatorBoard()

class ESCBoard():
	numThrusters = 8
	thrustersEnabled = 1
	currentThrusts = []  # The current pwm pulse width in microseconds

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
		except Exception as e:
			print("Error on ESC init: " + str(e))
			raiseFault()

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
		if self.thrustersEnabled and killSwitch.value() == 0 and getTime() - self.timeChange > 5000:
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

	def pressure(self):
		return self._pressure
		
	def temperature(self):
		degC = self._temperature / 100.0
		return degC
		
	# Depth relative to MSL pressure in given fluid density
	def depth(self):
		return (random.uniform(0, 2))

Depth = DepthSensor()

class CoproBoard():
	def restart(self):
		print("Machine reset requested... Exiting simulation")
		os._exit(0)

	def memory_usage(self):
		"""gc.collect()
		free_memory = gc.mem_free()
		occupy_memory = gc.mem_alloc()
		total_memory = free_memory+occupy_memory
		percent_usage = free_memory/total_memory"""
		percent_usage = random.uniform(0, 1)

		return percent_usage

Copro = CoproBoard()


killSwitch = Pin("Kill Switch")
auxSwitch = Pin("Aux Switch")

def killSwitchChanged(pin):
	ESC.stopThrusters()
	ESC.timeChange = getTime()

killSwitchChanged(killSwitch)
