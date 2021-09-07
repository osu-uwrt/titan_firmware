onCopro = False
try:
	import halSimulated as hal
	from time import sleep
	import traceback
	import asyncio
except ImportError:
	onCopro = True
	import hal
	import uasyncio as asyncio

import commands
import sys

CONNECTION_TIMEOUT_MS = 1500

def safetyDisable():
	"""Will disable all systems that should not be running without an active connection.
	Mainly used for thrusters, and can be forwarded to actuator board to alert it that it has lost connection.
	"""
	hal.ESC.stopThrusters()
	# TODO: Add in forwarding to ESC

async def mainLoop():
	try:
		# Enable watchdog timer
		try:
			f = open("watchdog_enable", "r")
			f.close()
			print("Enabling Watchdog Timer")
			hal.Copro.start_watchdog()
		except OSError:
			print("Disabling Watchdog Timer")
		
		# Start main loop
		while True:
			# Handle network data
			command_id, data, packet_data = hal.commandServer.next_command()
			if command_id is not None:
				resp_data = commands.runCommand(command_id, data)
				hal.commandServer.reply(bytearray(resp_data), packet_data)
			
			# TODO: Implement safety timeout!

			# Feed Watchdog
			hal.Copro.feed_watchdog()

			# Yield
			if not onCopro:
				sleep(0.01)
			else:
				await asyncio.sleep(0)
	except Exception as exc:
		if not onCopro:
			traceback.print_exc()
			print(exc)
		else:
			sys.print_exception(exc)
			hal.raiseFault(hal.MAIN_LOOP_CRASH)
		hal.commandServer.deinit()


async def depthLoop():
	try:
		await asyncio.sleep(1.0)
		if (hal.Depth.initialized):
			print("Zeroing depth")
			for _ in range(1, 20):
				await hal.Depth.read()
			for _ in range(1, 20):
				await hal.Depth.zeroDepth()
			print("Collecting depth")
			while True:
				try:
					await hal.Depth.read()
				except Exception as e:
					print("Depth error: " + str(e))
					await asyncio.sleep_ms(50)
	except Exception as exc:
		print("Depth loop error:")
		sys.print_exception(exc)
		hal.raiseFault(hal.DEPTH_LOOP_CRASH)


async def lowVolt():
	try:
		while hal.BB.portVolt.value() < 18.5 or hal.BB.stbdVolt.value() < 18.5:
			await asyncio.sleep(1.0)
		while True:
			await asyncio.sleep(1.0)
			if hal.BB.portVolt.value() < 18.5 or hal.BB.stbdVolt.value() < 18.5:
				hal.ESC.setThrusterEnable(False)
				# hal.blueLed.on()
				print("Low Battery")
				hal.raiseFault(hal.BATT_LOW)
			else:
				hal.ESC.setThrusterEnable(True)
				hal.lowerFault(hal.BATT_LOW)
	except Exception as exc:
		print("Battery Checker error:")
		sys.print_exception(exc)
		hal.raiseFault(hal.BATTERY_CHECKER_CRASH)
		
		
async def auto_cooling():
	try:
		while True:
			current_temp = hal.BB.temp.value()
			temp_thresh = commands.temp_threshold([])[0]
			if current_temp > temp_thresh:
				hal.BB.peltierPower.value(1)
			else:
				hal.BB.peltierPower.value(0)
			await asyncio.sleep(0)
	except Exception as exc:
		print ("Auto Cooling Error ")
		sys.print_exception(exc)
		hal.raiseFault(hal.AUTO_COOLING_CRASH)


loop = asyncio.get_event_loop()
loop.create_task(depthLoop())
loop.create_task(mainLoop())
loop.create_task(lowVolt())
loop.create_task(auto_cooling())
loop.run_forever()
loop.close()

# We shouldn't have gotten this far... Something went wrong
hal.raiseFault(hal.PROGRAM_TERMINATED)