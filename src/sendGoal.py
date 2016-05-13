#!/usr/bin/env python
import roslib
roslib.load_manifest('sound_localization')
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import cos, sin, radians

class Navigation(object):

    x, y, heading, v, w = 0.0, 0.0, 0.0, 0.0, 0.0
    navInfo = "Start sendGoal waiting for command"

    def __init__(self):
        rospy.init_node('sendGoal')
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('/finalAngle', String, self.setGoal)
        rospy.Subscriber('/odom',Odometry, self.callback_odom)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.checkStatus)

    def callback_odom(self, odom):
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        self.v = odom.twist.twist.linear.x
        self.w = odom.twist.twist.angular.z
        q1 = odom.pose.pose.orientation.x
        q2 = odom.pose.pose.orientation.y
        q3 = odom.pose.pose.orientation.z
        q4 = odom.pose.pose.orientation.w
        q = (q1, q2, q3, q4)
        e = euler_from_quaternion(q)
        yaw = e[2]   #degrees(e[2])
        self.heading = yaw

    def setGoal(self, data):
        # distance is self-defined
        dist = 3 # m
        desired_heading = self.heading + radians(int(data.data))
        desired_x = self.x + dist*cos(desired_heading)
        desired_y = self.y + dist*sin(desired_heading)

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "map"
        ps.pose.position.x = desired_x
        ps.pose.position.y = desired_y
        q = quaternion_from_euler(0.0 ,0.0 ,desired_heading)
        ps.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        self.pub.publish(ps)

    def checkStatus(self,data):
        self.navInfo = data.status_list[0].text
        self.status = data.status_list[0].status

    def printInfo(self):
        rospy.loginfo(self.navInfo)

if __name__ == '__main__':
    try:
        nav = Navigation()
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            nav.printInfo()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error rospy")
    finally:
        rospy.loginfo("Publish_nav_cmd stopped")
