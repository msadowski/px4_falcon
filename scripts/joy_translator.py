#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import TwistStamped

twist_pub = rospy.Publisher('falcon/twist', TwistStamped, queue_size=10)

def joy_callback(msg):    
    x = msg.axes[2]
    y = -msg.axes[3]
    z = msg.axes[1]
    rot_z = msg.axes[4]
    

    twist = TwistStamped()
    twist.header.stamp = rospy.Time.now()
    twist.twist.linear.x = x
    twist.twist.linear.y = y
    twist.twist.linear.z = z
    twist.twist.angular.z = rot_z
    twist_pub.publish(twist)

def main():
    rospy.init_node('joy_translator', anonymous=False)
    rospy.Subscriber('joy', Joy, joy_callback)
    rospy.spin() 


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
