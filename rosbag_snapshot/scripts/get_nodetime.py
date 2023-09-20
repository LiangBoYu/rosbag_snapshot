#!/usr/bin/python
# -*- coding: UTF-8 -*-



import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading
import queue


class TopicStatistics:
    def __init__(self, time):
        self.time = time
        self.traffic = 0
        self.delivered_msgs = 0
        self.window_start = 0
        self.window_stop = 0

class MessageQueue:
    def __init__(self):
        self.lock = threading.Lock()
        self.queue_ = queue.Queue()

    def fillStatus(self, status):
        with self.lock:  
            if self.queue_.empty():  
                return

            status.window_start = self.queue_.queue[0].time
            status.window_stop = self.queue_.queue[-1].time

def amcl_pose_callback(msg):
    # 在這裡將接收到的amcl_pose訊息放入MessageQueue的queue_佇列中
    message_queue.queue_.put(msg)

# 初始化 ROS 節點
rospy.init_node('amcl_pose_subscriber')

# 建立MessageQueue物件
message_queue = MessageQueue()

# 訂閱amcl_pose話題，指定回呼函式為amcl_pose_callback
rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)

# 在此處持續執行，直到程式被終止
rospy.spin()

# 創建空的TopicStatistics物件，用於填充資訊
status = TopicStatistics(0)

# 調用fillStatus方法並傳入status物件
message_queue.fillStatus(status)

# 印出結果
print("Traffic:", status.traffic)
print("Delivered Messages:", status.delivered_msgs)
print("Window Start:", status.window_start)
print("Window Stop:", status.window_stop)
