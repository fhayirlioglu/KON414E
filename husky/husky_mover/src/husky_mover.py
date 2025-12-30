#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_husky():
    rospy.init_node('husky_mover', anonymous=True)
    
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)
    
    vel_msg = Twist()
    vel_msg.linear.x = 1.0   
    vel_msg.angular.z = 0.5  

    rospy.loginfo("Husky hareket ediyor...")

    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_husky()
    except rospy.ROSInterruptException:
        pass
