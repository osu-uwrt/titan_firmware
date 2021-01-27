import random
import time
import gc

def getTime():
    return int(time.time()*1000)

class Sensor:
	def __init__(self, getFunction):
		self.data = 0
		self.lastgetionTime = 0
		self.cacheDuration = 100
		self.getFunction = getFunction

	def value(self):
		if getTime() - self.lastgetionTime > self.cacheDuration:
			self.get()
		return self.data

	def get(self):
		self.data = self.getFunction()
		self.lastgetionTime = getTime()

class Pin:
	state = 0
	def on(self):
		pass
	def off(self):
		pass
	def value(self, a=None):
		if a is None:
			return self.state
		else:
			self.state = a



blueLed = Pin()
greenLed = Pin()

class BBBoard:
	deviceAddress = 0x1F

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

class ConvBoard:
	deviceAddress = 0x37

	moboPower = Pin()
	jetsonPower = Pin()
	peltierPower = Pin()
	threePower = Pin()
	fivePower = Pin()
	twelvePower = Pin()

	def getFiveCurrent():
		return random.uniform(0,10)
	def getThreeCurrent():
		return random.uniform(0,10)
	def getTwelveCurrent():
		return random.uniform(0,10)
	def getTwelveVolt():
		return random.uniform(0,21)
	def getFiveVolt():
		return random.uniform(0,21)
	def getThreeVolt():
		return random.uniform(0,21)
	def getTemp():
		return random.uniform(0,70)
	def actuators(self, args):
		return [1]

	fiveVolt = Sensor(getFiveVolt)
	threeVolt = Sensor(getThreeVolt)
	twelveVolt = Sensor(getTwelveVolt)
	fiveCurrent = Sensor(getFiveCurrent)
	threeCurrent = Sensor(getThreeCurrent)
	twelveCurrent = Sensor(getTwelveCurrent)
	temp = Sensor(getTemp)

Converter = ConvBoard()

class ESCBoard():
	deviceAddress = 0x2F
	thrusts = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

	def getCurrents():
		current_vals = []
		for i in range(8):
			current_vals.append(int(random.uniform(0,10)*25))
		return current_vals

	def stopThrusters(self):
		pass

	def setThrusters(self, thrusts):
		pass

	def setThrusterEnable(self, enable):
		pass

	currents = Sensor(getCurrents)

ESC = ESCBoard()

class StatusBoard():
	screenAddress = 0x78

	def write(self, text):
		print(text)

Status = StatusBoard()

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

	def memory_usage(self):
		return random.uniform(0, 1)

Copro = CoproBoard()


killSwitch = Pin()
switch1 = Pin()
switch2 = Pin()
switch3 = Pin()
switch4 = Pin()
resetSwitch = Pin()