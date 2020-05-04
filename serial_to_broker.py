import serial
import threading
import paho.mqtt.client as mqtt
import time
import sys
from serial.tools import list_ports

sys.stdout.flush()




brokerIP = "192.168.1.11"

runScript = True


countingModel = "0"
selectedModel = "0"
isReStart = False
isReStop = False
isReReset = False
isReRelease = False
sendingDataToArduino = False


####### MQTT ##########################
#def on_connect(client,userdata,rc):
def on_connect(client, userdata, flags, rc):
    print("Connected with result code:"+str(rc))
    # subscribe for all devices of user
    mqttClient.subscribe("cmd/#")

# gives message from device
def on_message(client,userdata,msg):
	#print("Topic"+msg.topic + "\nMessage:" + msg.payload.decode("utf-8") )
	global countingModel
	global selectedModel
	global isReStart
	global isReStop
	global isReReset
	global isReRelease
	
	topic=msg.topic
	message=msg.payload.decode("utf-8")

	if(topic == "cmd/update/counting"):
		countingModel = message


	if(topic == "cmd/update/model"):
		selectedModel = message;

	if(topic == "cmd/command"):
		if(message == "start"):
			isReStart = True
		if(message == "stop"):
			isReStop = True
		if(message == "release"):
			isReReset = True
			isReRelease = True    
    

def loopMqtt():
	while runScript:
		mqttClient.loop()
		time.sleep(0.05)
	
mqttClient = mqtt.Client(client_id='esp', clean_session=False)
mqttClient.on_connect = on_connect
mqttClient.on_message = on_message	
mqttClient.connect(host=brokerIP, port=1883,keepalive=60)
mqttClient.loop_start()
#mqttThread = threading.Thread(target=loopMqtt, args=())
#mqttThread.start()


ser = None
#this will store the line
bufferSerial = []

def processMessage(message):
	global sendingDataToArduino
	subitems = message.split(":")
	if(subitems[0] == "r"):
		if(not sendingDataToArduino):
			sendingDataToArduino = True
			sendStatusToSerial()
			time.sleep(0.01)
			sendingDataToArduino = False
		#pass
	elif(subitems[0] == "s"):
		sendDataToBroker(subitems[1:3])
	elif(subitems[0] == "i"):
		print("starting")

def sendStatusToSerial():
	global countingModel
	global selectedModel
	global isReStart
	global isReStop
	global isReReset
	global isReRelease

	isReStartValue="0"
	isReResetValue="0"
	isReReleaseValue="0"
	isReStopValue="0"
	
	if(isReStart):
		isReStartValue="1"
	
	if(isReReset):
		isReResetValue="1"
		
	if(isReRelease):
		isReReleaseValue="1"
		
	if(isReStop):
		isReStopValue="1"
	
	
	ser.write("<".encode());
	ser.write(isReStartValue.encode());
	ser.write(":".encode());
	ser.write(isReResetValue.encode());
	ser.write(":".encode());
	ser.write(isReReleaseValue.encode());
	ser.write(":".encode());
	ser.write(isReStopValue.encode());
	ser.write(":".encode());
	ser.write(selectedModel.encode());
	ser.write(":".encode());
	ser.write(countingModel.encode());
	ser.write(":>".encode());
	
	countingModel = "0"
	selectedModel = "0"
	isReStart = False
	isReStop = False
	isReReset = False
	isReRelease = False

def sendDataToBroker(item):
	topic = item[0]
	message = item[1]
	#print(topic + " : " + message)
	mqttClient.publish(topic,message)
	#print(topic+":"+message)
	
def getListPorts():
	ports = list_ports.comports()
	subPorts = []
	for port, desc, hwid in sorted(ports):
			if(port.startswith("/dev/ttyACM")):
				subPorts.append(port)
			if(port.startswith("/dev/ttyUSB")):
				subPorts.append(port)
	return subPorts
	
def connectToSerial():
	global ser
	serialPorts = getListPorts()
	
	for port in serialPorts:
		try:
			ser = serial.Serial(
				port=port,\
				baudrate=115200,\
				parity=serial.PARITY_NONE,\
				stopbits=serial.STOPBITS_ONE,\
				bytesize=serial.EIGHTBITS,\
					timeout=1, \
				 xonxoff=0, \
				rtscts=0)

			ser.setDTR(False)
			ser.setRTS(False)
			ser.flush()
			print("connected to: " + ser.portstr)
			break
			#time.sleep(1)
			#ser.setDTR(True)
		except Exception as e:
			pass

	
	
connectToSerial()
try:
	recvInProgress = False
	startMarker = '<'
	endMarker = '>'
    
	while runScript:
		try:
			for c in ser.read():
				try:
					c = chr(c)  ##For python3
				except:
					pass
				if(recvInProgress):
					if(c == endMarker):
						threading.Thread(target=processMessage, args=(''.join(bufferSerial),)).start()
						bufferSerial = []
						recvInProgress = False
					else:
						bufferSerial.append(c)
				else:
					if(c == startMarker):
						recvInProgress = True
		except serial.serialutil.SerialException:
			print("serialError")
			connectToSerial()
			time.sleep(5)
			#raise Exception("Sorry, no numbers below zero")
except Exception as e:
	print(e)
	runScript = False
finally:
	ser.close() 
	runScript = False
	mqttClient.disconnect()
	mqttClient.loop_stop()
