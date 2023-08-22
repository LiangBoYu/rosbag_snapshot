#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import time
import json
import rospy
import yaml
from std_srvs.srv import Trigger
from rosbag_snapshot_msgs.srv import TriggerSnapshot, TriggerSnapshotRequest
from threading import Lock
from rosgraph_msgs.msg import TopicStatistics
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class callrosbag(object):
    def __init__(self):
        rospy.loginfo("callrosbag start")
        self.filename = '/home/gyro/catkin_ws/src/rosbag_snapshot/bags/short_control_instability.bag'
        self.yamlfile = '/home/gyro/catkin_ws/src/rosbag_snapshot/rosbag_snapshot/param/snapshot.yaml'
        self.subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.process_odom)
        #self.topics = ['/tf', '/scan', '/amcl_pose']
        self.queue = []
        self.size = 0
        self.lock = Lock()
        self.start_time0 = 1000

        self.main()
        
    def main(self):
        with open(self.yamlfile, 'r') as rosbag_param:      
            data = yaml.safe_load(rosbag_param) 
            self.topics = data['topics']
            #self.start_time = rospy.Time(secs=15000, nsecs=0)
            self.start_time = rospy.Time(secs=self.start_time0, nsecs=0)
            self.stop_time = rospy.Time(secs=2000, nsecs=0)
            #print("need save topics:{}".format(self.topics))

        success, message = self.call_trigger_snapshot(self.filename, self.topics, self.start_time, self.stop_time)
        if success:
            print("Service call successful.")
        else:
            print("Service call failed:", message)


    def odom_callback(self, msg):
        self.latest_odom = msg

    def process_odom(self, event):
        if self.latest_odom:
            seq = self.latest_odom.header.seq
            self.odom_nowsecs = self.latest_odom.header.stamp.secs
            nsecs = self.latest_odom.header.stamp.nsecs
            frame_id = self.latest_odom.header.frame_id

            #rospy.loginfo("odom_Sequence: %d", seq)
            #rospy.loginfo("odom_Stamp: %d.%d", self.odom_nowsecs, nsecs)
            rospy.loginfo("odom_Stamp: %d", self.odom_nowsecs)
            #rospy.loginfo("odom_Frame ID: %s", frame_id)

    def call_trigger_snapshot(self, filename, topics, start_time, stop_time):
        rospy.wait_for_service('/trigger_snapshot')
        try:
            trigger_snapshot_proxy = rospy.ServiceProxy('/trigger_snapshot', TriggerSnapshot)
            request = TriggerSnapshotRequest()
            request.filename = filename
            request.topics = topics
            request.start_time = start_time
            request.stop_time = stop_time
            response = trigger_snapshot_proxy(request)
            return response.success, response.message
        except rospy.ServiceException as e:
            return False, "Service call failed: " + str(e)
        
if __name__ == "__main__":
    rospy.init_node("call_rosbag_snapshot", anonymous=False)
    cd = callrosbag()
    rospy.spin()


#==============
# import rospy
# from rosbag_snapshot_msgs.srv import TriggerSnapshot, TriggerSnapshotRequest

# def call_trigger_snapshot(filename, topics, start_time, stop_time):
#     rospy.wait_for_service('/trigger_snapshot')
#     try:
#         trigger_snapshot_proxy = rospy.ServiceProxy('/trigger_snapshot', TriggerSnapshot)
#         request = TriggerSnapshotRequest()
#         request.filename = filename
#         request.topics = topics
#         request.start_time = start_time
#         request.stop_time = stop_time
#         response = trigger_snapshot_proxy(request)
#         return response.success, response.message
#     except rospy.ServiceException as e:
#         return False, "Service call failed: " + str(e)

# if __name__ == '__main__':
#     rospy.init_node('trigger_snapshot_caller')
    
#     # 設定要呼叫服務的參數
#     #filename = 'short_control_instability000.bag'
#     filename = '/home/gyro/catkin_ws/src/rosbag_snapshot/bags/short_control_instability.bag'
#     topics = ['/tf', '/scan', '/amcl_pose']
#     start_time = rospy.Time(secs=5600, nsecs=0)
#     stop_time = rospy.Time(secs=5700, nsecs=0)

#     # 呼叫服務並獲取回應
#     success, message = call_trigger_snapshot(filename, topics, start_time, stop_time)

#     # 輸出結果
#     if success:
#         print("Service call successful.")
#     else:
#         print("Service call failed:", message)







#======================================