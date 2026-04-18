"""
Author: Kennan Bays (Kenneract)
Created: Aug.28.2023
Updated: Aug.17.2024
Purpose: Simple program that facilitates downloading
		data and erasing flash contents from QRET Avionics
		SRAD boards using Module Data Exchange standard. Ideally packaged
		as an executable for easier distribution.
		
		This above is a lie.
"""


from serial import Serial
import struct
import serial.tools.list_ports
from time import sleep, time
import datetime
# NOTE: You must move the "rdes.py" file to this directory
from rdes import RDESDecompressor
import csv


BAUD_RATE = 115200


# Serial commands supported by Debug Mode.
CMDS = {"Erase": 		b"E",
		"Identify": 	b"I",
		"FlashInfo": 	b"F",
		"FlashDump": 	b"D",
		"QuerySensors":	b"Q"}


"""
# MDE Structure

Normally, the module will only output a single MDE message upon startup (e.g. "Send serial to enter debug"). After 5 seconds of idling, the module will continue with normal operation. If serial data is sent within this period, the module will enter Debug mode and will send another MDE message confirming this (e.g. "Entered debug mode"). All following MDE messages are responses to serial requests.

All modules must follow this behaviour.
"""


# Discovery screen:


print("\n")
print("PROGRAM VERSION: V1.1.0 (Aug.17.2024 1430)")
print("\n")
print("Searching for board - Connect an SRAD module to get started. (If an SRAD module is already connected, please disconnect and reconnect it). SRAD boards can only be put in Debug mode within 5 seconds of being powered on. If you need a USB-UART bridge, it may be useful to plug the bridge in first, THEN turn on the module.")
print("\n")

TIMEOUT = 1 #Serial timeout time

def getAvailPorts():
	"""
	Simple wrapper function which returns
	a list of available COM ports.
	"""
	return [comport.device for comport in serial.tools.list_ports.comports()]

def waitForPort():
	"""
	Waits for a new serial COM port to become available,
	returning its name. Only responds to the addition of
	new ports, not the removal of existing ones.
	
	Blocks execution until COM port is found.
	"""
	lastPorts = getAvailPorts()
	while True:
		# Scan for new ports
		sleep(0.2)
		curPorts = getAvailPorts()
		# Determine if any new ports
		changedPorts = set(curPorts) ^ set(lastPorts) #symmetric diff
		newPorts = []
		for port in changedPorts:
			# Run through all changed ports; if
			# are in current list, they are new.
			if (port in curPorts):
				newPorts.append(port)
		# Cache current ports
		lastPorts = curPorts
		# If any new ports were found, return first one
		if (len(newPorts) > 0):
			return newPorts[0]

def waitForBoard():
	"""
	Blocks until an SRAD board is connected and recognized.
	Returns the Serial object for the board, with the board
	in Debug mode.
	"""
		
	# Continuously hunt for boards until an SRAD one is found
	while True:
		# Wait for COM port
		port = waitForPort()
		# Connect to device
		sleep(0.1)
		device = Serial(port, baudrate=BAUD_RATE, timeout=TIMEOUT)
		print(f"Serial ({port}) connected; waiting for startup")
		# Wait for 20 seconds for an MDE packet
		waitTime = 20
		while (waitTime >= 0):
			waitTime-=0.25
			sleep(0.25)
			print(".", end="", flush=True)
			recv = waitForLine(device, True, 1)
			if (recv is not None):
				# Connected!
				print("OK")
				break
		if (waitTime < 0):
			print("Missed debug window - aborting connection.")
			device.close()
			return None
		# MDE received; ready to enter debug mode
		print("Attempting to enter debug mode")
		device.write(0x01)
		# Wait for 20 seconds for an MDE response
		waitTime = 20
		while (waitTime >= 0):
			waitTime-=0.25
			sleep(0.25)
			print(".", end="", flush=True)
			recv = waitForLine(device, True, 1)
			if (recv is not None):
				# Entered debug mode!
				print("OK")
				break
		if (waitTime < 0):
			print("Failed to enter debug mode - aborting connection")
			device.close()
			return None
		# Entered debug mode
		return device
	return None

def waitForLine(device, mdeOnly:bool=False, timeout=None):
	"""
	Given an SRAD Serial device, blocks until a
	string is available over serial. A full, decoded
	string is provided.
	
	if "mdeOnly" is requested, this function will ONLY
	return lines explicitly defined as Module Data
	Exchange (MDE) lines.
	
	If given a timeout, will return None if that much
	time elapses without an MDE response
	"""
	timeLeft = 0
	if (timeout is not None):
		timeLeft = timeout
	while True:
		if (timeout is not None and timeLeft <= 0):
			return None
		# Attempt to sample
		#resp = device.readline().decode().strip()
		#resp = device.readline().decode(errors="ignore").strip()
		#resp = device.readline().decode("cp1252",errors="ignore").strip()
		resp = device.readline().decode("cp1252").strip()
		# !!!!  [ MAKE SURE YOU'RE USING THE RIGHT BAUD RATE ]  !!!!
		if len(resp) > 0:
			if (mdeOnly):
				if ("[MDE]" in resp):
					return resp[5:].strip()
			else:
				return resp
		# Decrement timeout
		timeLeft -= TIMEOUT

def getBoardInfo(device):
	"""
	Given an SRAD Serial device, queries it for its name and flash details.
	
	Returns a tuple of (BoardName, FlashSize, FlashUsage).
	"""
	# Get raw data
	device.write(CMDS["Identify"])
	sleep(0.1)
	boardName = waitForLine(device, True)
	device.write(CMDS["FlashInfo"])
	sleep(0.1)
	flashInfo = waitForLine(device, True)
	# Process data
	boardName = boardName.strip()
	flashSize, flashUsed = flashInfo.split(",")
	flashUsed = int(flashUsed)
	flashSize = int(flashSize)
	# Return
	return (boardName, flashSize, flashUsed)

def eraseBoardFlash(device):
	"""
	Given an SRAD Serial device, sends the appropriate command
	to erase the flash module.
	
	Blocks until erase is complete.
	"""
	# Send erase command
	device.write(CMDS["Erase"])
	
	# Try to read confirmation
	resp = waitForLine(device, True, 2) #confirmation
	if (resp == None):
		return False

	# Wait for erase to finish
	resp = waitForLine(device, True)
	if (resp == "Complete"):
		return True
	else:
		return False

def tempRemoveTrailingEmpty(byteArr):
	"""
	Given a byte array of data from flash, removes the trailing 15 0xFF bytes.
	
	This is a constant number of bytes due to the current seek algorithm. With improved seeking (using the SessionIndex system), this won't be nessesary.
	
	ALSO REMOVES THE FIRST BYTE, ASSUMING IT IS ERRONEOUS ZERO.
	"""
	return byteArr[1:-12]

def retrieveBoardFlash(device):
	"""
	Given an SRAD Serial device, sends the command to dump
	all flash data.
	
	Participates in active data exchange - returns full bytearray
	of bytes once complete.
	
	
	NOTES FOR IMPROVING TRANSFER SPEED:
	- Changing block size from 64-1024 didn't have much effect.
		BLOCK	SPEED2(readBulkAtEnd)	SPEED3(readAsTransmit)
		128B 	107KiB/s				
		256B	116KiB/s
		512B	123KiB/s				123KiB/s
		1024B 	126KiB/s
		2048B	128KiB/s
		4096B	129KiB/s				130KiB/s
	- Stages to optimize
		- Waiting for incoming bytes
		- Reading/Decoding bytes in buffer
		- Requesting next Block
	"""
	rawPayload = bytearray()
	done = False
	# Send dump command
	device.write(CMDS["FlashDump"])
	sleep(0.1)
	# Read handshake data & confirm transfer
	device.read()
	
	blockSize = struct.unpack('H', device.read(2))[0]
	numBlocks = struct.unpack('H', device.read(2))[0]
	numRealBytes = struct.unpack('I', device.read(4))[0]

	#device.read()
	
	print(f"Device wants to send {numRealBytes}B of data in {numBlocks} x {blockSize}B blocks.")

	# ACK handshake; data starts
	sleep(0.5) #make sure board is ready to receive data
	device.write(b'N')
	print("(Sent ACK to start transfer)")
	startTime = time()

	
	# Read blocks
	curBlock = 0
	blockFails = 0
	while (curBlock < numBlocks):

		# Begin reading block data
		PRNT_INT = 10
		if ((curBlock+1)%PRNT_INT == 0):
			
			prog = (curBlock+1)/numBlocks
			elapsed = time()-startTime
			blcksLeft = numBlocks-(curBlock+1)
			minLeft = elapsed/(curBlock+1) * blcksLeft
			minLeft = minLeft/60
			
			print(f"Receiving block {curBlock+1}/{numBlocks} ({prog:.1%}, {minLeft:.1f}m left)")
		
		blck = device.read(blockSize)
		
		# Check for timeout (undersized payload)
		if (len(blck) < blockSize):
			blockFails += 1
			sleep(0.5) #make sure device is ready to receive
			device.write(b'L') #"re-send last block"
			if (blockFails > 10): return None
			print(f"\tREQUESTING RE-SEND ({blockFails}) (want {blockSize}B, got {len(blck)})")
			continue
		
		# Load BLOCK data into array
		blockFails = 0
		rawPayload.extend(blck)

		# Request next block, if any
		curBlock += 1
		if (curBlock < numBlocks):
			device.write(b'N') #"send next block"

	elapsedTime = (time() - startTime)
	if (elapsedTime==0):
		speed=0
	else:
		speed = (numRealBytes/elapsedTime)/1024 #KiB/s
	
	print(f"Finished reading {curBlock} blocks.")
	print(f"Operation transfer speed: {speed:.1f}KiB/s")
	
	# Transfer complete - slice out padded bits
	return rawPayload[:numRealBytes]


def retrieveBoardFlashOLD(device):
	"""
	Given an SRAD Serial device, sends the command to dump
	all serial data.
	
	Retrieves data, and returns the resulting array of bytes
	from flash.
	"""
	rawPayload = []
	done = False
	# Send dump command
	device.write(CMDS["FlashDump"])
	sleep(0.1)
	# Wait for dump start byte (ASCII "#")
	while (device.in_waiting == 0):
		pass
	c = device.read().decode("cp1252")
	if (c == "#"):
		print("Preparing for incoming data...")
		sleep(0.5)
		print("Reading incoming data... (may take a minute)")
		# Retrieve binary data
		rawPayload = []
		done = False

		# Capture bulk of data
		#while not done:
		#	while(device.in_waiting > 256):
		#		rawPayload.append(device.read())
		#	print(f"\t[PauseForFill, Recv={len(rawPayload)}B]")
		#	sleep(0.30)
		#	done = (device.in_waiting <= 256)
		# Capture remaining data
		#print("flushing last bit of data")
		
		while(device.in_waiting > 0):
			rawPayload.append(device.read())
			
			l = len(rawPayload)
			if l%8192==0:
				print(l)
			
			
		# Wait for ending byte (ASCII "#")
		print(f"buffer empty, retrieved {len(rawPayload)}B, waiting for last byte")
		print(rawPayload[0:64])
		sleep(0.10)
		while (device.in_waiting == 0):
			pass
		c = device.read().decode("cp1252").strip()
		print(f"read: {c}")
		if (c == "#"):
			print("Successfully read all data")

			
		print(f"Successfully downloaded {len(rawPayload)}B payload; decoding...")
		# Convert into array of bytes
		#strBytes = rawPayload.strip().strip("#").strip(",").split(",")
		#dataBytes = [int(x) for x in strBytes]
		dataBytes = rawPayload
		print(f"Decoded ASCII payload into {len(dataBytes)}B of compressed data")
		
		print(rawPayload[0:64])
		
		return dataBytes
	print("Read failed")
	return None

def writeBytesToFile(prefix, rawBytes):
	"""
	Given a prefix and the bytes, dumps to file
	"""
	dt = datetime.datetime.now().strftime("%b.%d.%Y-%H.%M.%S")
	fName = f"{prefix}_{dt}.bin"
	
	#outStr = ""
	#for item in rawBytes:
		#outStr += f"{item},"
	with open(fName, "wb") as binFile:
		# Write bytes to file
		#b = bytearray(rawBytes)
		binFile.write(rawBytes)
	print(f"Dumped {len(rawBytes)}B to file: {fName}")




def writeToCSVSensor(prefix, outTable):
	"""
	Given the table of data, saves to CSV file.
	"""
	headers = ["Time","Pres","Temp","Humidity","GasRes"]

	dt = datetime.datetime.now().strftime("%b.%d.%Y-%H.%M.%S")
	fName = f"{prefix}_{dt}.csv"

	with open(fName, mode="w", newline="") as file:
		writer = csv.writer(file)
		writer.writerow(headers)
		
		for row in outTable:
		
			#correct magnitude
			outRow = row #[row[0], row[1]/1000, row[2]/10]
			outRow = [row[0], row[1], row[2]-273, row[3]/100, row[4]]
			writer.writerow(outRow)

def decompDataSensor(rawBytes):
	# Convert to int array (?)

	# GPS

	# Attempt to uncompress data
	#	Variant=2, 1 column
	decomp = RDESDecompressor(variant=2, numCols=5, verbose=False)
	decompData = decomp.decompress(rawBytes)
	
	orig = decomp.getCompressedSize()/1024
	expa = decomp.getUncompressedSize()/1024
	ratio = orig/expa

	print(f"\tDecoded {expa:.1f}KiB from {orig:.1f}KiB (CR{ratio:.1%})")
	return decompData




def decompDataGPS(rawBytes):
	# Convert to int array (?)

	# GPS

	# Attempt to uncompress data
	#	Variant=2, 1 column
	decomp = RDESDecompressor(variant=2, numCols=7, verbose=False, signedCols=[1,2,3,4])
	decompData = decomp.decompress(rawBytes)
	
	orig = decomp.getCompressedSize()/1024
	expa = decomp.getUncompressedSize()/1024
	ratio = orig/expa

	print(f"\tDecoded {expa:.1f}KiB from {orig:.1f}KiB (CR{ratio:.1%})")
	return decompData



def writeToCSVGPS(prefix, outTable):
	"""
	Given the table of data, saves to CSV file.
	
	GPS only
	"""
	headers = ["time","Latitude","Longitude","Alt","NumSat"]

	dt = datetime.datetime.now().strftime("%b.%d.%Y-%H.%M.%S")
	fName = f"{prefix}_{dt}.csv"

	with open(fName, mode="w", newline="") as file:
		writer = csv.writer(file)
		writer.writerow(headers)
		
		for row in outTable:
		
			#correct magnitude
			outRow = row #[row[0], row[1]/1000, row[2]/10]
			outRow = [row[0], str(row[1])+"."+str(row[2]), str(row[3])+"."+str(row[4]), row[5], row[6]]
			writer.writerow(outRow)



def decompDataAltimeter(rawBytes):
	# Convert to int array (?)

	# ALTIMETER

	# Attempt to uncompress data
	#	Variant=2, 1 column
	decomp = RDESDecompressor(variant=2, numCols=11, verbose=False, signedCols=[3,4,5,6,7,8,9])
	decompData = decomp.decompress(rawBytes)
	
	orig = decomp.getCompressedSize()/1024
	expa = decomp.getUncompressedSize()/1024
	ratio = orig/expa

	print(f"\tDecoded {expa:.1f}KiB from {orig:.1f}KiB (CR{ratio:.1%})")
	return decompData



def writeToCSVAltimeter(prefix, outTable):
	"""
	Given the table of data, saves to CSV file.
	"""
	headers = ["Time","Temp","Press"]
	headers = ["mS","Pa","K","MS2","Gx","Gy","Gz","DegX","DegY","DegZ","State"]

	dt = datetime.datetime.now().strftime("%b.%d.%Y-%H.%M.%S")
	fName = f"{prefix}_{dt}.csv"

	with open(fName, mode="w", newline="") as file:
		writer = csv.writer(file)
		writer.writerow(headers)
		
		for row in outTable:
		
			#correct magnitude
			outRow = row #[row[0], row[1]/1000, row[2]/10]
			outRow = [row[0], row[1], row[2]/1000,row[3]/10000, row[4]/10000, row[5]/10000,row[6]/10000, row[7]/10000, row[8]/10000, row[9]/10000, row[10]]
			writer.writerow(outRow)



print("Waiting for SRAD device...")
device = waitForBoard()
sleep(0.2)
name, size, used = getBoardInfo(device)

print("")
print("SRAD Device Found!")
print(f"Port is {device.port}")
print(f"Device identity is {name}")
print(f"Table is {size}B in size, {used}B used. ({used/size:.1%})")


while True:

	print("\n## COMMANDS ##")
	print("1. Dump flash data to file(s)")
	print("2. Erase flash (WARNING - DO NOT RUN IF REAL DATA ON BOARD)")
	print("3. Exit")
	cmd = input("ENTER COMMAND:").strip()
	print("\n")
	
	if (cmd == "1"):
		print("Dumping flash...")
		raw = retrieveBoardFlash(device)
		raw = tempRemoveTrailingEmpty(raw)
		print("Writing BIN to file...")
		writeBytesToFile(name, raw)
		print("Decoding with RDES2...")		
		
		if ("gps" in name.lower()):
			out = decompDataGPS(raw)
		elif ("sensor" in name.lower()):
			out = decompDataSensor(raw)
		else:
			out = decompDataAltimeter(raw)
		
		print("Saving decoded data...")
		
		if ("gps" in name.lower()):
			writeToCSVGPS(name, out)
		elif ("sensor" in name.lower()):
			writeToCSVSensor(name, out)
		else:
			writeToCSVAltimeter(name, out)

		print("Dump complete.")
		
	elif (cmd == "2"):
		print("CONFIRM FLASH ERASE.")
		print("TYPE THE FOLLOWING EXACTLY: Confirm")
		conf = input(">")
		if (conf == "Confirm"):
			print("Erasing flash... (~45s)")
			eraseBoardFlash(device)
			print("Erase complete!")
			print("YOU SHOULD RESTART THE BOARD NOW; OTHER COMMANDS MAY NOT WORK")
	
	elif (cmd == "3"):
		break
