#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from numpy import median, std

class SoundLocalization(object):
    def __init__(self):
        rospy.init_node("sound_localization")
    	rospy.Subscriber('/serial/read', String, self.subSoundAngle)
        rospy.Subscriber('/recognizer', String, self.handleCmd)
        self._pubFinalAngle = rospy.Publisher('/finalAngle', String, queue_size=10)
        self._loopRate = 100
        self.rate = rospy.Rate(self._loopRate)
        self._receivedAngle = None
        self._receiveFlag = False
        self.finalSoundAngle = None
        self._counter = 0
        self._angleList = []
        self._receiveCmdFlag = False

    def subSoundAngle(self, data):
        #rospy.loginfo("Received angle: %s"%data.data)
        self._receivedAngle = int(data.data)
        self._receiveFlag = True

    def handleCmd(self, data):
        if data.data == "come":
            self._receiveCmdFlag = True

    def recognizer(self,recognizeTime=0.2):
        self._counter += 1
        # Reset after listen 0.2sec
        if self._counter > recognizeTime*self._loopRate:
            self._counter, self._angleList = 0, []
        # Add angle to list
        if self._receiveFlag:
            self._angleList.append(self._receivedAngle)
            self._receiveFlag = False
        # Calculate result
        if len(self._angleList)<5:
            self.finalSoundAngle = None
        else:
            self.finalSoundAngle = median(self._angleList)
            rospy.loginfo("Final sound angle: %s"%self.finalSoundAngle)

    def pubResult(self):
        if self._receiveCmdFlag and self.finalSoundAngle:
            self._pubFinalAngle.publish(self.finalSoundAngle)
            self._receiveCmdFlag = False

    def start(self):
        self.recognizer()
        self.pubResult()
        self.rate.sleep()


if __name__ == '__main__':
    sl = SoundLocalization()
    try:
        while not rospy.is_shutdown():
            sl.start()
    finally:
        rospy.loginfo("Stop sound localization")
