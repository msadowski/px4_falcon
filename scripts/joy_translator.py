#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from math import sqrt,sin,cos,acos,pi

twist_pub = rospy.Publisher('falcon/twist', TwistStamped, queue_size=10)
heading = 0.0

def joy_callback(msg):    
    x = msg.axes[2]
    y = -msg.axes[3]
    z = msg.axes[1]
    rot_z = msg.axes[4]
    
    v = sqrt(x*x+y*y)
    if v != 0.0:
        alpha = acos(x/v)
    else:
        alpha = 0.0
    
    x = v*cos(heading+alpha)
    y = v*sin(heading+alpha)
    rospy.loginfo("v: %f\nhdg: %f\n alpha: %f\n x: %f\n y: %f", v, heading, alpha, x, y)

    twist = TwistStamped()
    twist.header.stamp = rospy.Time.now()
    twist.twist.linear.x = v*cos(heading+alpha)
    twist.twist.linear.y = v*sin(heading+alpha)
    twist.twist.linear.z = z
    twist.twist.angular.z = rot_z
    twist_pub.publish(twist)
    
def hdg_callback(msg):
    global heading 
    heading = msg.data*pi/180.0

def main():
    rospy.init_node('joy_translator', anonymous=False)
    rospy.Subscriber('joy', Joy, joy_callback)
    rospy.Subscriber('mavros/global_position/compass_hdg', Float64, hdg_callback)
    rospy.spin() 


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
