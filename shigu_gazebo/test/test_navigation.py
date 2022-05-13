#!/usr/bin/env python

import rospy
import sys
import unittest
import rostest
from sensor_msgs.msg import LaserScan
import geometry_msgs.msg as gm
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatusArray
import csv
import logging
import unittest
import codecs
import time


class TestNavigation(unittest.TestCase):
    def setUp(self):
        self.filename = None
        self.available_ = 0
        self.seq = None
        self.goal = gm.PoseStamped()


    # wait until hear the /scan topic
    def scan_topic_listener(self):
        rospy.wait_for_message('/scan',LaserScan,timeout=100)

    # send single goal to /move_base_simple/goal and see feedback
    def test_single_goal(self):
        self.filename = str(sys.argv[1])
        self.available_ = int(sys.argv[2])
        self.output_log('Is it available:' + str(self.available_) + sys.argv[2])
        self.seq = int(sys.argv[3])
        tolerance = float(sys.argv[4])
        rospy.init_node('navigation_tester')
        self.read_from_csv(self.filename,self.seq)
        self.output_log('data read from csv')
        self.scan_topic_listener()
        self.output_log('scan topic is available:' +  str(self.goal.pose.position.x))
        self.single_send()
        self.output_log('publish complete')
        try:
            rospy.wait_for_message('/move_base/DWAPlannerROS/global_plan',Path,timeout=10)
            self.assertNotEqual(self.available_,0)
            goalStatus = GoalStatusArray()
            status = 1
            rospy.sleep(10) # avoid the first feedback
            i = 0
            while(status == 1 or status == 2):
                status = rospy.wait_for_message('/move_base/status',GoalStatusArray,timeout=10).status_list[0].status
                self.output_log('moving, status:' + str(status))
                rospy.sleep(0.1)
            if status == 3:
                self.output_log('reached!')
                amcl_pose = rospy.wait_for_message('/amcl_pose',gm.PoseWithCovarianceStamped,timeout=10)
                self.assertAlmostEqual(amcl_pose.pose.pose.position.x,self.goal.pose.position.x, delta = tolerance)
                self.assertAlmostEqual(amcl_pose.pose.pose.position.y,self.goal.pose.position.y, delta = tolerance)
                self.assertAlmostEqual(amcl_pose.pose.pose.position.z,self.goal.pose.position.z, delta = tolerance)
                self.assertAlmostEqual(amcl_pose.pose.pose.orientation.x,self.goal.pose.orientation.x, delta = tolerance)
                self.assertAlmostEqual(amcl_pose.pose.pose.orientation.y,self.goal.pose.orientation.y, delta = tolerance)
                self.assertAlmostEqual(amcl_pose.pose.pose.orientation.z,self.goal.pose.orientation.z, delta = tolerance)
                self.assertAlmostEqual(amcl_pose.pose.pose.orientation.w,self.goal.pose.orientation.w, delta = tolerance)
        except rospy.ROSException:
            self.output_log('timeout means the goal is not available or already reached')
            amcl_pose = rospy.wait_for_message('/amcl_pose',gm.PoseWithCovarianceStamped,timeout=10)
            self.output_log('get amcl_pose')
            self.output_log(str(self.goal.pose.position.x))
            self.output_log(str(amcl_pose.pose.pose.position.x))
            self.output_log(str(abs(float(amcl_pose.pose.pose.position.x)- float(self.goal.pose.position.x))))
            if(abs(float(amcl_pose.pose.pose.position.x) - float(self.goal.pose.position.x)) > tolerance):
                self.output_log('not available')
                self.assertEqual(self.available_,0)
            else:
                self.output_log('already reached')
                self.assertNotEqual(self.available_,0)
        
        
        
    def single_send(self):
        pub = rospy.Publisher('/move_base_simple/goal', gm.PoseStamped)
        self.output_log('ready to publish')
        rospy.sleep(2)
        self.output_log(str(isinstance(self.goal, gm.PoseStamped)))
        pub.publish(self.goal)
        

    def read_from_csv(self, filename, seq):
        with open(filename, 'rb') as csvfile:
            reader = csv.reader(codecs.EncodedFile(csvfile, 'utf-8', 'utf-8-sig'), delimiter=',') 
            for row in reader:
                if int(row[0]) == seq:
                    self.goal.header.frame_id = "map"
                    self.goal.pose.position.x = float(row[1])
                    self.goal.pose.position.y = float(row[2])
                    self.goal.pose.position.z = float(row[3])
                    self.goal.pose.orientation.x = float(row[4])
                    self.goal.pose.orientation.y = float(row[5])
                    self.goal.pose.orientation.z = float(row[6])
                    self.goal.pose.orientation.w = float(row[7])
                    break    

    def output_log(self, log):
        now = int(round(time.time()*1000))
        now02 = time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(now/1000))
        with open(sys.argv[5], 'a') as file:
            file.write(now02 + '  ::  ' + log + '\n')
        
if __name__ == '__main__':
    rostest.run('shigu_gazebo','navigation_test',TestNavigation,sys.argv)