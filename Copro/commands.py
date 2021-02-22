try:
    import hal
except:
	import halSimulated as hal

import time

def runCommand(data):
	commandNum = data.pop(0)
	try:
		if commandNum < len(commandList) and commandList[commandNum] is not None:
			response = commandList[commandNum](data)
		else:
			print("Unexpected command received:", commandNum)
			response = []
	except Exception as e:
		print("Error on command "+str(commandNum)+": " + str(e))
		hal.raiseFault(hal.COMMAND_EXEC_CRASH_FLAG + commandNum)
		response = []
	return response

def moboPower(args):
	# Args: int boolean for setting, or empty to get
	if len(args) == 1:
		if args[0] == 1 or args[0] == 0:
			hal.BB.moboPower.value(args[0])
			return [1]
		else:
			return [0]
	elif len(args) == 0:
		return [hal.BB.moboPower.value()]
	else:
		print("Unexpected argument length for moboPower")
		return []

def lightingPower(args):
	# Args: two byte array for the percentage on each light should be
	if len(args) == 2:
		if args[0] == 1:
			successful = hal.BB.setLight1(args[1])
			return [int(successful)]
		elif args[0] == 2:
			successful = hal.BB.setLight2(args[1])
			return [int(successful)]
		else:
			return [0]
	elif len(args) == 0:
		return hal.BB.currentLightValues
	else:
		print("Unexpected argument length for lightingPower")
		return []

def thrusterEnable(args):
	# Args: int boolean for setting, or empty to get
	if len(args) == 1:
		if args[0] == 1 or args[0] == 0:
			successful = hal.ESC.setThrusterEnable(args[0])
			return [int(successful)]
		else:
			return [0]
	elif len(args) == 0:
		return [hal.ESC.thrustersEnabled]
	else:
		print("Unexpected argument length for thrusterEnable")
		return []

def peltierPower(args):
	# Args: int boolean for setting, or empty to get
	if len(args) == 1:
		if args[0] == 1 or args[0] == 0:
			hal.BB.peltierPower.value(args[0])
			return [1]
		else:
			return [0]
	elif len(args) == 0:
		return [hal.BB.peltierPower.value()]
	else:
		print("Unexpected argument length for peltierPower")
		return []

def getBatVolts(args):
	if not hal.BB.initialized:
		return [0]
	try:
		portVolt = int(hal.BB.portVolt.value() * 100)
		stbdVolt = int(hal.BB.stbdVolt.value() * 100)
		balancedVolt = int(hal.BB.balancedVolt.value() * 100)
		return [1, portVolt // 256, portVolt % 256, stbdVolt // 256, stbdVolt % 256, balancedVolt // 256, balancedVolt % 256]
	except OSError:
		return [0]

def getBatCurrents(args):
	if not hal.BB.initialized:
		return [0]
	try:
		portCurrent = int(hal.BB.portCurrent.value() * 100)
		stbdCurrent = int(hal.BB.stbdCurrent.value() * 100)
		return [1, portCurrent // 256, portCurrent % 256, stbdCurrent // 256, stbdCurrent % 256]
	except OSError:
		return [0]

def getTemperature(args):
	if not hal.BB.initialized:
		return [0]
	try:
		temp = int(hal.BB.temp.value() * 10)
		return [1, temp // 256, temp % 256]
	except OSError:
		return  [0]

def thrusterForce(args):
	# Args: 8 2-byte words (MSB) for each of the thrusters, or empty to get current thruster values
	if len(args) == 16:
		values = []
		for i in range(8):
			values.append((args[2 * i] << 8) + args[2 * i + 1])
		success = hal.ESC.setThrusters(values)
		return [int(success)]
	elif len(args) == 0:
		values = []
		thrusts = hal.ESC.currentThrusts
		for i in range(8):
			values.append(thrusts[i] // 256)
			values.append(thrusts[i] % 256)
		return values
	else:  # If it wasn't empty or 8 values, return failure
		print("Unexpected argument length for thrusterForce")
		return []

"""
def logicCurrents(args):
	threeCurrent = int(hal.Converter.threeCurrent.value() * 1000)
	fiveCurrent = int(hal.Converter.fiveCurrent.value() * 1000)
	twelveCurrent = int(hal.Converter.twelveCurrent.value() * 1000)
	return [threeCurrent // 256, threeCurrent % 256, fiveCurrent // 256, fiveCurrent % 256, twelveCurrent // 256, twelveCurrent % 256, ]
"""

def logicVolts(args):
	if not hal.BB.initialized:
		return [0]
	try:
		#threeVolt = int(hal.BB.threeVolt.value() * 1000)  # 3.3V rail was removed from monitoring on this adc
		threeVolt = 0
		fiveVolt = int(hal.BB.fiveVolt.value() * 1000)
		twelveVolt = int(hal.BB.twelveVolt.value() * 500)
		return [1, threeVolt // 256, threeVolt % 256, fiveVolt // 256, fiveVolt % 256, twelveVolt // 256, twelveVolt % 256]
	except OSError:
		return [0]

def switches(args):
	data = hal.Backplane.killSwitch.value()
	data = (data << 1) + hal.Backplane.auxSwitch.value()
	return [data]

def depth(args):
	if hal.Depth.initialized:
		data = int(hal.Depth.depth()*100000)
		return [1, (data >> 16), (data >> 8) & 0xFF, data & 0xFF]
	else:
		return [0, 0, 0, 0]


def twelvePower(args):
	if len(args) == 1:
		if args[0] == 1 or args[0] == 0:
			hal.BB.twelvePower.value(args[0])
			return [1]
		else:
			return [0]
	elif len(args) == 0:
		return [hal.BB.twelvePower.value()]
	else:
		print("Unexpected argument length for twelvePower")
		return []

def fiveReset(args):
	hal.BB.fivePower.value(0)
	time.sleep(1)
	hal.BB.fivePower.value(1)
	return [1]

def getThrusterCurrents(args):
	if not hal.ESC.initialized:
		return [0]
	try:
		values = hal.ESC.currents.value()
		data = [1]
		for i in values:
			data.append(int(i*25))
		return data
	except OSError:
		return [0]

def reset(args):
	hal.Copro.restart()

def actuator(args):
	return hal.Actuator.actuators(args)

def latency_check(args):
	return [1]

def memory_check(args):
	usage = int(hal.Copro.memory_usage()*(256*256-1))
	return [usage// 256,  usage % 256]

temp = 40
def temp_threshold(args):
    global temp
    if len(args) == 1:
        temp = args[0]
    return [temp]
    
def get_fault_state(args):
	if len(hal.faultList) != 0:
		# Make sure that the fault list doesn't have an invalid message causing the connection to drop
		if len(hal.faultList) > 254:
			return [1, hal.FAULT_STATE_INVALID]
		
		for entry in hal.faultList:
			if type(entry) != int or entry < 0 or entry > 255:
				return [1, hal.FAULT_STATE_INVALID]
		return [1] + hal.faultList
	else:
		return [0]


commandList = [
	moboPower,			#0
	lightingPower,		#1  Was jetsonPower, replaced since it is no longer in use
	thrusterEnable,		#2
	peltierPower,		#3
	getBatVolts,		#4
	getBatCurrents,		#5
	getTemperature,		#6
	thrusterForce,		#7
	None,				#8  Was logicCurrents but adc was removed from board
	logicVolts,			#9
	switches,			#10
	depth,				#11
	getThrusterCurrents,#12
	twelvePower,		#13
	fiveReset,			#14
	reset,				#15
	actuator,	     	#16
	latency_check,      #17 
	memory_check,       #18 
	temp_threshold,     #19 
	get_fault_state,	#20
]
