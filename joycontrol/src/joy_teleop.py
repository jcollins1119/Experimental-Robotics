#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Joy

vl = 0
vr = 0

def joy_callback(data):
    global vl, vr
    vl = data.axes[1]
    vr = data.axes[4]
    rospy.loginfo("Left: %s", vl)
    rospy.loginfo("Right: %s", vr)

def imu_callback(data):
    pass
#    rospy.loginfo("Angular Velocity = %f", data.angular_velocity.z)

def main():
    rospy.init_node('joy_teleop', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#    rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, imu_callback)
    rospy.Subscriber("/joy", Joy, joy_callback)
    rate = rospy.Rate(10)
    vel = Twist()
    while not rospy.is_shutdown():
        vel.linear.x = vl* 100   #Must be from -400 to 400
        vel.linear.y = vr* 100   #Must be from -400 to 400
        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()
