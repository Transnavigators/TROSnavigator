import serial
#/dev/ttyUSB1
ser = serial.Serial(
    port='COM6',
    baudrate=115200,
	timeout=0)
	
def move(radius,angle):
	global ser
	#Move
	moveCmd = "\xEE\x21" + radius + angle
	ser.write(moveCmd)

def setSpeed(speec,accel,ac):
	global ser
	#Set speed, accel, centrip accel
	setCmd = "\xEE\x11" + speed + accel + ac
	ser.write(setCmd)

def stopNow():
	global ser
	#Stop Now
	stopCmd = "\xEE\x01"
	ser.write(stopCmd)

def stopLater():
	global ser
	#Stop Later
	stopLaterCmd = "\xEE\x00"
	ser.write(stopLaterCmd)

def checkBattery():
	global ser
	#Check Battery
	batteryCmd = "\xEE\x30"
	ser.write(batteryCmd)

	

while True:
	data = ser.readline()
	if(data != 0):
		print(data.decode("utf-8"))
	
	