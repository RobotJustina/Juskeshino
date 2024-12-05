import time
from roboclaw_3 import Roboclaw

#Windows comport name
rc = Roboclaw("/dev/justinaRC30",115200)
#Linux comport name
#rc = Roboclaw("/dev/ttyACM0",115200)


def displayspeed():
	enc1 = rc.ReadEncM1(address)
	enc2 = rc.ReadEncM2(address)


	print("Encoder1:"),
	print(enc1)
	#print ("Encoder2:")
	#print(enc2)


rc.Open()
address = 0x80

version = rc.ReadVersion(address)
if version[0]==False:
	print ("GETVERSION Failed")
else:
	print(repr(version[1]))

while(1):
	displayspeed()
	time.sleep(0.1)
