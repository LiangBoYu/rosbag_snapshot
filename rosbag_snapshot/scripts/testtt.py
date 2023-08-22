#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class AMCLPoseListener:
    def __init__(self):
        rospy.init_node('amcl_pose_listener_class', anonymous=True)

        self.subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.process_odom)


    def amcl_pose_callback(self, msg):
        seq = msg.header.seq
        secs = msg.header.stamp.secs
        nsecs = msg.header.stamp.nsecs
        frame_id = msg.header.frame_id

        #rospy.loginfo("AMCL_Sequence: %d", seq)
        rospy.loginfo("AMCL_Stamp: %d.%d", secs, nsecs)
        #rospy.loginfo("AMCL_Frame ID: %s", frame_id)

    def odom_callback(self, msg):
        self.latest_odom = msg

    def process_odom(self, event):
        if self.latest_odom:
            seq = self.latest_odom.header.seq
            secs = self.latest_odom.header.stamp.secs
            nsecs = self.latest_odom.header.stamp.nsecs
            frame_id = self.latest_odom.header.frame_id

            #rospy.loginfo("odom_Sequence: %d", seq)
            rospy.loginfo("odom_Stamp: %d.%d", secs, nsecs)
            #rospy.loginfo("odom_Frame ID: %s", frame_id)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    listener = AMCLPoseListener()
    listener.run()

