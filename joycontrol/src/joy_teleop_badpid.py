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


"""
def pid_speed(target_speed):
    global prev_error_l, i_error_l, prev_error_r, i_error_r
    error = target_speed - odom_speed
    d_error = (error - prev_error) / dt
    i_error = i_error + error * dt
    out_speed = Kp*error + Ki*i_error + Kd*d_error
    prev_error = error
    return out_speed
"""

def main():
    rospy.init_node('joy_teleop', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#    rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, imu_callback)
    rospy.Subscriber("/joy", Joy, joy_callback)
    rate = rospy.Rate(10)
    vel = Twist()
    setspeed_l = 0
    setspeed_r = 0
    dt = .1
    while not rospy.is_shutdown():
		if abs(vl) < .02:
			setspeed_l = 0

		else:
			if (vl - setspeed_l) > .05:
				setspeed_l = setspeed_l + abs(vl)*dt
			elif (vl - setspeed_l) < -.05:
				setspeed_l = setspeed_l - abs(vl)*dt
			else:
				setspeed_l = vl

		if abs(vr) < .02:
			setspeed_r = 0
			print(setspeed_r)
		else:
			if (vr - setspeed_r) > .05:
				setspeed_r = setspeed_r + abs(vr)*dt
			elif (vr - setspeed_r) < -.05:
				setspeed_r = setspeed_r - abs(vr)*dt
			else:
				setspeed_r = vr

		vel.linear.x = setspeed_l
		vel.linear.y = setspeed_r
		vel_pub.publish(vel)
		rate.sleep()

if __name__ == "__main__":
    main()
