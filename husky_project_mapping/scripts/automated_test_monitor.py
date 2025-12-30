#!/usr/bin/env python3

import rospy
import math
import csv
import os
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus

class NavigationMonitor:
    def __init__(self):
        rospy.init_node('nav_monitor', anonymous=True)

        self.start_time = None
        self.start_pose = None
        self.total_distance = 0.0
        self.last_pose = None
        self.is_active = False
        self.current_map = rospy.get_param('~map_name', 'unknown_map')
        
        # CSV File Setup
        self.csv_file = os.path.expanduser('~/ros_ws/nav_test_results.csv')
        self.init_csv()

        # Subscribers
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_cb)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_cb)
        rospy.Subscriber('/odometry/filtered_map', Odometry, self.odom_cb)

        rospy.loginfo("Navigation Monitor Initialized. Ready to record.")

    def init_csv(self):
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['Map', 'Status', 'Time_Duration_s', 'Distance_m', 'Timestamp'])

    def goal_cb(self, msg):
        rospy.loginfo("New Navigation Goal Detected! Starting recording...")
        self.start_time = rospy.Time.now()
        self.total_distance = 0.0
        self.is_active = True
        self.last_pose = None # Reset last pose to wait for first fresh odom

    def odom_cb(self, msg):
        if not self.is_active:
            return

        current_pose = msg.pose.pose.position

        if self.last_pose is not None:
            dx = current_pose.x - self.last_pose.x
            dy = current_pose.y - self.last_pose.y
            dist = math.sqrt(dx*dx + dy*dy)
            self.total_distance += dist
        
        self.last_pose = current_pose

    def result_cb(self, msg):
        if not self.is_active:
            return

        end_time = rospy.Time.now()
        duration = (end_time - self.start_time).to_sec()
        self.is_active = False

        status_id = msg.status.status
        status_text = msg.status.text
        
        # Translate status ID to human readable
        if status_id == GoalStatus.SUCCEEDED:
            status_str = "SUCCESS"
        elif status_id == GoalStatus.ABORTED:
            status_str = "ABORTED"
        else:
            status_str = f"OTHER({status_id})"

        rospy.loginfo(f"Goal Finished. Status: {status_str}, Time: {duration:.2f}s, Dist: {self.total_distance:.2f}m")
        
        self.log_to_csv(status_str, duration, self.total_distance)
    
    def log_to_csv(self, status, duration, distance):
        with open(self.csv_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([self.current_map, status, round(duration, 3), round(distance, 3), rospy.Time.now()])
        rospy.loginfo(f"Data saved to {self.csv_file}")

if __name__ == '__main__':
    try:
        NavigationMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
