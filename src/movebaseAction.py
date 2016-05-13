#!/usr/bin/env python
import roslib
roslib.load_manifest("sound_localization")
import rospy
import actionlib
from move_base_msgs import MoveBaseAction
from actionlib_msgs import GoalStatusArray

class NavigationClient(object):
    def __init__(self):
        rospy.init_node("navStatusCheck")
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.handle_status)

    def handle_status(self,data):
        info = data.status_list[0].text
        status = data.status_list[0].status
        rospy.liginfo(info)
        rospy.loginfo("Status:%s"%status)

if __name__=="__main__":
    navClient = NavigationClient()
    rospy.spin()
    #client = actionlib.SimpleActionClient("navStatusCheck", MoveBaseAction)
