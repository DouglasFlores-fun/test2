import serial
import threading
import paho.mqtt.client as mqtt
import time
import sys
sys.stdout.flush()

runScript = True


countingModel = "0"
selectedModel = "0"
isReStart = False
isReStop = False
isReReset = False
isReRelease = False

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
mqttClient.connect(host='192.168.1.11', port=1883,keepalive=60)
mqttClient.loop_start()
#mqttThread = threading.Thread(target=loopMqtt, args=())
#mqttThread.start()



ser = serial.Serial(
    port='/dev/ttyACM0',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=1)

print("connected to: " + ser.portstr)

#this will store the line
bufferSerial = []




def processMessage(message):
	subitems = message.split(":")
	if(subitems[0] == "r"):
		sendStatusToSerial()
	elif(subitems[0] == "s"):
		sendDataToBroker(subitems[1:3])

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
	mqttClient.publish(topic,message)
	#print(topic+":"+message)
	

try:
	recvInProgress = False
	startMarker = '<'
	endMarker = '>'
    
	while runScript:
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
					
except Exception as e:
	print(e)
	runScript = False
finally:
	ser.close() 
	runScript = False
	mqttClient.disconnect()
	mqttClient.loop_stop()