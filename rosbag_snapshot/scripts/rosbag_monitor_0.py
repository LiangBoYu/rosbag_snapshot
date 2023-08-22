#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from syswork.srv import MapLocation, SysIndicatorSrv, playmp3srv
from syswork.msg import syswarn, syserr, AlarmMsg, RFIDAlarm, RFIDWarning, EncoderAlarm, RoutemapAlarm, CassetteDoorAlarm, RoutemapWarning, WarningMsg, TMModbusAlarm, SysWorkAlarm, AGVMAlarm, MotorCfgDiagAlarm
import os
import signal
import subprocess
import time
from datetime import datetime
import json
import yaml
from std_srvs.srv import Trigger
from rosbag_snapshot_msgs.srv import TriggerSnapshot, TriggerSnapshotRequest
from threading import Lock
from rosgraph_msgs.msg import TopicStatistics
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry




class rosbag_monitor(object):
    def __init__(self):
        rospy.loginfo("rosbag_monitor_v_0_1_0 start")
        self.errorsummary = ""
        self.has_alarm_flag = False
        self.latest_odom = None
        self.minus_time = 0
        self.savepath = '/home/gyro/catkin_ws/src/rosbag_snapshot/bags/'
        self.yamlfile = '/home/gyro/catkin_ws/src/rosbag_snapshot/rosbag_snapshot/param/snapshot.yaml'
        self.nowsec = rospy.get_time()
        self.syserr_sub = rospy.Subscriber("/sysindicator/error", syserr, self.syserr_cb)
        self.subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.process_odom)



    def syserr_cb(self, msg):
        self.errorsummary = msg.errorsummary
        if self.errorsummary != "" :
            self.set_minus()
            if self.has_alarm_flag == False:
                self.has_alarm_flag = True
                rospy.loginfo("ALARM_CODE is " + self.errorsummary)
                now = datetime.now()
                file_time = now.strftime("%Y%m%d_%H%M%S_")
                alarm_name = str(self.errorsummary)
                self.final_file_name = self.savepath +file_time + alarm_name + '_'
                self.set_start_time = max(self.nowsec - self.minus_time, 0) #確保set_start_time不會小於0
                self.start_time = rospy.Time(secs=self.set_start_time, nsecs=0)
                self.stop_time = rospy.Time(secs=self.nowsec, nsecs=0) 
                with open(self.yamlfile, 'r') as rosbag_param:      
                    data = yaml.safe_load(rosbag_param) 
                    self.topics = data['topics']
                print("final_file_name:{}".format(self.final_file_name))
                print("topics:{}".format(self.topics))
                print("minus_time:{}".format(self.minus_time))
                print("start_time:{}".format(self.start_time))
                print("stop_time:{}".format(self.stop_time))
                success, message = self.call_trigger_snapshot(self.final_file_name, self.topics, self.start_time, self.stop_time)
                if success:
                    print("Service call successful.")
                else:
                    print("Service call failed:", message)
            else:
                rospy.loginfo("MR in ALARM, alarm code is {} ,not reset".format(self.errorsummary))

        if self.errorsummary == "":
            rospy.loginfo("MR OK")
            self.minus_time = 0
            self.has_alarm_flag = False

    def set_minus(self):
        #set minus time
        if self.errorsummary == 'RM1058' or self.errorsummary == 'RM1041':
            self.minus_time = 1200
            #MOVE TIME OUT
        elif self.errorsummary == 'RM1015' or self.errorsummary == 'RM1047':
            self.minus_time = 120
            #emo
        elif self.errorsummary == 'RM1049':
            self.minus_time = 120
            #LEAVE NG
        elif self.errorsummary == 'TB1008':
            self.minus_time = 120
            #DOCK NG
        elif self.errorsummary == 'RM1085' or self.errorsummary == 'PJ0002':
            self.minus_time = 200
            #TM TIMEOUT
    

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

    def odom_callback(self, msg):
        self.latest_odom = msg

    def process_odom(self, event):
        if self.latest_odom:
            secs = self.latest_odom.header.stamp.secs
            nsecs = self.latest_odom.header.stamp.nsecs
            self.nowsec = secs
            #rospy.loginfo("odom_now_time: %d", self.nowsec)

if __name__ == "__main__":
    rospy.init_node("rosbag_monitor_node", anonymous=False)
    cd = rosbag_monitor()
    rospy.spin()




'''
RM1058  RM1041  (MOVE TIME OUT)	NOW-1200sec
RM1015  RM1047	(EMO)		    NOW-60sec
RM1049		    (LEAVE NG)	    NOW-60sec
TB1008		    (DOCK NG)	    NOW-60sec
RM1085	PJ0002	(TM TIMEOUT)	NOW-200sec
'''