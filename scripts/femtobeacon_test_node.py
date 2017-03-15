#!/usr/bin/env python
import rospy
import tf
import math
from femtobeacon_ros.msg import femtoBeaconMsg
from geometry_msgs.msg import PoseStamped

bo = 0.
co = 0.

def callback(data):
	#print "beacon", data.yaw   
	bo = data.yaw
	print 1, " ", bo
 
def callback_robot(data):
	quaternion = (
	data.pose.orientation.x,
	data.pose.orientation.y,
	data.pose.orientation.z,
	data.pose.orientation.w)

	euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw = 180.*euler[2]/math.pi
	#print "chair", yaw
	co = yaw
	print 2, " ", co

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('fbtest', anonymous=False)

    rospy.Subscriber("/femtobeacon", femtoBeaconMsg, callback)
    rospy.Subscriber("/monkeycar4/robot_pose", PoseStamped, callback_robot)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
