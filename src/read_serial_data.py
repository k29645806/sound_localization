#!/usr/bin/env python

import threading
import serial
from cStringIO import StringIO
from std_msgs.msg import String
import time
import rospy

class SerialDataGateway(object):
	'''
	Helper class for receiving lines from a serial port
	'''
	def __init__(self, port="/dev/ttyACM0", baudrate=2400):
		'''
		Initializes the receiver class.
		port: The serial port to listen to.
		receivedLineHandler: The function to call when a line was received.
		'''
		self._Port = port
		self._Baudrate = baudrate
		self._KeepRunning = False
		self.serialReadData = ""
		self.f_readRenew = False

	def Start(self):
		try:
			self._Serial = serial.Serial(port=self._Port, baudrate=self._Baudrate, timeout=0)
		except:
			rospy.loginfo("SERIAL PORT Start Error")
			raise
		self._KeepRunning = True
		self._ReceiverThread = threading.Thread(target=self.Listen)
		self._ReceiverThread.setDaemon(True)
		self._ReceiverThread.start()

	def Stop(self):
		rospy.loginfo("Stopping serial gateway")
		self._KeepRunning = False
		time.sleep(.1)
		try:
			self._Serial.close()
		except:
			rospy.loginfo("SERIAL PORT Stop Error")
			raise

	def Listen(self):
		stringIO = StringIO()
		self._Serial.flushInput()
		start_time = time.time()
		while self._KeepRunning:
			try:
				data = self._Serial.read()
			except:
				rospy.loginfo("SERIAL PORT Listen Error")
				raise
			if (data == '\n'):
				self.serialReadData = stringIO.getvalue()
				self.f_readRenew = True
				rospy.loginfo(self.serialReadData)
				stringIO.close()
				stringIO = StringIO()
			else:
				stringIO.write(data)

	def Write(self, data):
		#AttributeError: 'SerialDataGateway' object has no attribute '_Serial'
		try:
			# print("serial write : " + data)
			self._Serial.write(data)
		except AttributeError:
			rospy.loginfo("SERIAL PORT Write Error")
			raise

	def handleCommand(self, data):
		# print data.data
		self.Write(data.data)

	def getSerialReadData(self):
		return self.serialReadData

	def getFlagReadRenew(self):
		return self.f_readRenew

	def setFlagReadRenew(self, flag):
		self.f_readRenew = flag

if __name__ == '__main__':
	dataReceiver = SerialDataGateway("/dev/ttyACM0", 2400)
	dataReceiver.Start()
	rospy.init_node("sound_angle_serial_node")
	serialPublisher = rospy.Publisher('/serial/read', String, queue_size=10)
	rate = rospy.Rate(100)
	try:
		while not rospy.is_shutdown():
			if (dataReceiver.getFlagReadRenew()):
				serialPublisher.publish(String(dataReceiver.getSerialReadData()))
				dataReceiver.setFlagReadRenew(False)
			else:
				#serialPublisher.publish(String("-1"))
				pass
			rate.sleep()
	finally:
		dataReceiver.Stop()
