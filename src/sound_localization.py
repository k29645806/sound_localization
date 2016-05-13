#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from numpy import median, std

class SoundLocalization(object):
    def __init__(self):
        rospy.init_node("sound_localization")
    	rospy.Subscriber('/serial/read', String, self.subSoundAngle)
        self.loopRate = 100
        self.rate = rospy.Rate(self.loopRate)
        self._receivedAngle = None
        self.finalSoundAngle = -1

    def subSoundAngle(self, data):
        #rospy.loginfo("Received angle: %s"%data.data)
        self._receivedAngle = data.data

    def recognizer(self, angle, recognizeTime=0.2):
        counter, angleList = 0, []
        counter += 1
        angleList.append(angle)

        # Reset
        if counter > recognizeTime*self.loopRate:
            counter, angleList = 0, []

        # Calculate result
        if len(angleList)<5:
            self.finalSoundAngle = -1
        else:
            self.finalSoundAngle = median(angleList)
            rospy.loginfo("Final sound angle: %s"%self.finalSoundAngle)


    def start(self):
        self.recognizer(self._receivedAngle)
        self.rate.sleep()


if __name__ == '__main__':
    sl = SoundLocalization()
    try:
        while not rospy.is_shutdown():
            sl.start()
    finally:
        rospy.loginfo("Stop sound localization")
