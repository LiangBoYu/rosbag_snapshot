#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from syswork.srv import MapLocation, SysIndicatorSrv, playmp3srv
from syswork.msg import syswarn, syserr, AlarmMsg, RFIDAlarm, RFIDWarning, EncoderAlarm, RoutemapAlarm, CassetteDoorAlarm, RoutemapWarning, WarningMsg, TMModbusAlarm, SysWorkAlarm, AGVMAlarm, MotorCfgDiagAlarm
from datetime import datetime
import yaml
from rosbag_snapshot_msgs.srv import TriggerSnapshot, TriggerSnapshotRequest
from threading import Lock
from nav_msgs.msg import Odometry


class rosbag_monitor(object):
    def __init__(self):
        rospy.loginfo("rosbag_monitor_v_0_1_6 start")
        self.errorsummary = ""
        self.has_alarm_flag = False
        self.latest_odom = None
        self.minus_time = 0
        self.savepath = '/home/gyro/maintain_work_logs/rosbag/'
        self.yamlfile = '/home/gyro/catkin_ws/src/rosbag_snapshot/rosbag_snapshot/param/snapshot.yaml'
        
        # self.nowsec = rospy.Time.now()
        # self.now_seconds = self.nowsec.secs
        # self.now_nsecs = self.nowsec.nsecs

        self.syserr_sub = rospy.Subscriber("/sysindicator/error", syserr, self.syserr_cb)
        #self.subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        #self.timer = rospy.Timer(rospy.Duration(1.0), self.process_odom)
    



    # def timer(self):
    #     print("nowtime")
    #     rospy.loginfo("now_time:{}".format(self.nowsec))
    #     self.set_start_time = max(self.nowsec - self.minus_time + 180 , 0) #start為發生alarm當下前3分鐘
    #     #self.set_stop_time = self.set_start_time + 120 #stop為發生alarm後2分鐘
    #     self.set_stop_time = self.set_start_time + 120
    #     rospy.loginfo("start_time:{}".format(self.start_time))
    #     rospy.loginfo("stop_time:{}".format(self.stop_time))

    def syserr_cb(self, msg):

        
        self.errorsummary = msg.errorsummary
        #if self.errorsummary != "" :

        with open(self.yamlfile, 'r') as set_alarm_time:
            try:
                self.config = yaml.safe_load(set_alarm_time)
            except yaml.YAMLError as exc:
                rospy.loginfo(exc)
        
        found_match = False

        for alarm in self.config['alarm_code']:

            if self.errorsummary in alarm:
                print(alarm)
                found_match = True
                self.minus_time = alarm[self.errorsummary]
                print("minus_time is: {}".format(self.minus_time))

                if self.has_alarm_flag == False:
                    self.has_alarm_flag = True
                    rospy.loginfo("ALARM_CODE is " + self.errorsummary)
                    print("has_alarm_flag: {}".format(self.has_alarm_flag))
                    rospy.loginfo("recording")
                    now = datetime.now()
                    file_time = now.strftime("%Y%m%d_%H%M%S_")

                    #起始時間
                    #self.nowsec = rospy.get_time()
                    # self.nowsec0 = rospy.get_time()
                    # self.nowsec = rospy.Time(secs=self.nowsec0, nsecs=0)/1000000000
                    self.nowsec = int(rospy.get_time())
                    rospy.loginfo("in alarm:{}".format(self.nowsec))



                    alarm_name = str(self.errorsummary)
                    self.final_file_name = self.savepath +file_time + alarm_name + '_'

                    #self.set_start_time = max(self.nowsec - (self.minus_time) - (180) , 0) #start為發生alarm當下前3分鐘
                    self.set_start_time = self.nowsec - (self.minus_time) - (180)  #start為發生alarm當下前3分鐘
                    if self.set_start_time < self.nowsec:
                        self.set_start_time = 0

                    #self.set_stop_time = self.set_start_time + 120 #stop為發生alarm後2分鐘
                    if (self.errorsummary == 'RM1058') or (self.errorsummary == 'RM1041') :
                        self.set_stop_time = self.set_start_time + (180)
                    else :
                        self.set_stop_time = self.nowsec

                    self.start_time = rospy.Time(secs=self.set_start_time, nsecs=0)
                    self.stop_time = rospy.Time(secs=self.set_stop_time, nsecs=0) 
                    # self.start_time = self.set_start_time
                    # self.stop_time = self.set_stop_time
                    with open(self.yamlfile, 'r') as rosbag_param:      
                        data = yaml.safe_load(rosbag_param) 
                        self.topics = data['topics']
                    rospy.loginfo("final_file_name:{}".format(self.final_file_name))
                    rospy.loginfo("topics:{}".format(self.topics))
                    #rospy.loginfo("minus_time:{}".format(self.minus_time))
                    rospy.loginfo("now_time:{}".format(self.nowsec))
                    rospy.loginfo("start_time:{}".format(self.start_time))
                    rospy.loginfo("stop_time:{}".format(self.stop_time))
                    success, message = self.call_trigger_snapshot(self.final_file_name, self.topics, self.start_time, self.stop_time)
                    if success:
                        rospy.loginfo("Service call successful.")
                    else:
                        rospy.loginfo("Service call failed: {}".format(message))
                else:
                    print("has_alarm_flag: {}".format(self.has_alarm_flag))
                    rospy.loginfo("MR in ALARM, alarm code is {} ,not reset".format(self.errorsummary))


        if not found_match:
                print("has_alarm_flag: {}".format(self.has_alarm_flag))
                rospy.loginfo("doesnt need record")


            # else:
            #     print("has_alarm_flag: {}".format(self.has_alarm_flag))
            #     rospy.loginfo("doesnt need record")

        if self.errorsummary == "":
            print("MR OK")
            self.nowsec00 = rospy.get_time()
            self.nowsecshow = rospy.Time(secs=self.nowsec00, nsecs=0)
            rospy.loginfo("in OK now_time:{}".format(self.nowsecshow))
            self.minus_time = 0
            self.has_alarm_flag = False

    # def set_minus(self):
    #     with open(self.yamlfile, 'r') as set_alarm_time:
    #         try:
    #             self.config = yaml.safe_load(set_alarm_time)
    #         except yaml.YAMLError as exc:
    #             rospy.loginfo(exc)
    #     for alarm in self.config['alarm_code']:
    #         if self.errorsummary in alarm:
    #             self.minus_time = alarm[self.errorsummary]
    #             # rospy.loginfo("set:{}".format(self.errorsummary))
    #             # rospy.loginfo("set:{}".format(self.minus_time))
    #             break

    def call_trigger_snapshot(self, filename, topics, start_time, stop_time):
        print("recording")
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

    # def odom_callback(self, msg):
    #     self.latest_odom = msg

    # def process_odom(self, event):
    #     if self.latest_odom:
    #         secs = self.latest_odom.header.stamp.secs
    #         nsecs = self.latest_odom.header.stamp.nsecs
    #         self.nowsec = secs
    #         #rospy.loginfo("odom_now_time: %d", self.nowsec)

if __name__ == "__main__":
    rospy.init_node("rosbag_monitor_node", anonymous=False)
    cd = rosbag_monitor()
    rospy.spin()
