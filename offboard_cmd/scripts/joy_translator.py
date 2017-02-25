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
   
    gs_x = x*sin(heading) + y*cos(heading)
    gs_y = x*cos(heading) - y*sin(heading)

    publish_twist(gs_x,gs_y,z,rot_z)

def publish_twist(x,y,z,rot_z):    
    twist = TwistStamped()
    twist.header.stamp = rospy.Time.now()
    twist.twist.linear.x = x
    twist.twist.linear.y = y 
    twist.twist.linear.z = z
    twist.twist.angular.z = rot_z
    twist_pub.publish(twist)

def get_sign(num):
    if num < 0:
        return -1
    elif num == 0:
        return 0
    else:
        return 1

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
