#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose,PoseStamped, Quaternion, Twist, Vector3, PoseWithCovarianceStamped

def odomCallback(poseupdate):
	global odom_sub_data
        odom_sub_data = PoseStamped()
	odom_sub_data = poseupdate
	#rospy.loginfo(odom_sub_data)
	#rospy.loginfo("odom_sub_data %s", odom_sub_data)
	#rospy.loginfo("poseupdate.pose %s", poseupdate.pose)
#Either class info isnt getting passed or I have to pass each attribute individually xsuub ysub
	
def main():
	global odom_sub_data
	odom_sub_data = PoseStamped()
	x = 0.0
	y = 0.0
	th = 0.0

	vx = 0.1
	vy = -0.1
	vth = 0.1


	
	rospy.init_node('vis_odom', anonymous=True)


	current_time = rospy.Time.now()
	last_time = rospy.Time.now()

	odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
	odom_sub = rospy.Subscriber("/slam_out_pose", PoseStamped, odomCallback)
	odom_broadcaster = tf.TransformBroadcaster()

	r = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		current_time = rospy.Time.now()
		# since all odometry is 6DOF we'll need a quaternion created from yaw
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
		# first, we'll publish the transform over tf
		odom_broadcaster.sendTransform(
		(x, y, 0.),
		   odom_quat,
		   current_time,
		   "base_link",
		   "odom"
		)
		
		#Get position from Hector_slam
		#rospy.loginfo(odom_sub)
		#x = odom_sub.x
		#y = odom_sub.y
		#z = 0
	
		# next, we'll publish the odometry message over ROS
		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "odom"

		# set the position
		#odom.pose.pose = Pose(Point(x, y, z), odom_sub.orientation)
		#rospy.loginfo(type(odom_sub_data.pose))
		odom.pose.pose = odom_sub_data.pose

		# set the velocity to 0
		# Note: May need to estimate this from position measurements
		odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

		rospy.loginfo(odom)
		# publish the message
		odom_pub.publish(odom)

		last_time = current_time
		r.sleep()

if __name__ == "__main__":
	main()
