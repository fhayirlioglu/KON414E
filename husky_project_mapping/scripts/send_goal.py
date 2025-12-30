#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():
    rospy.init_node('exact_goal_sender')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    print("Move_base sunucusu bekleniyor...")
    client.wait_for_server()

    # Hedef oluşturuluyor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    
    goal.target_pose.pose.position.x = -20.5   # X Metre
    goal.target_pose.pose.position.y = 35.3   # Y Metre
    goal.target_pose.pose.orientation.w = 0.00187 # Yonelim 
    

    print(f"Hedef gonderiliyor: X={goal.target_pose.pose.position.x}, Y={goal.target_pose.pose.position.y}")
    client.send_goal(goal)

    # Sonuc bekleniyor
    wait = client.wait_for_result()
    
    if not wait:
        rospy.logerr("Sunucu cevap vermedi!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        result = movebase_client()
        if result:
            print("HEDEFE ULASILDI! Test Basarili.")
    except rospy.ROSInterruptException:
        print("Test iptal edildi.")