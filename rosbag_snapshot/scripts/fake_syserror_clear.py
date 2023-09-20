import rospy
from syswork.srv import MapLocation, SysIndicatorSrv,playmp3srv
from syswork.msg import syswarn,syserr,AlarmMsg,RFIDAlarm,RFIDWarning,EncoderAlarm,RoutemapAlarm, CassetteDoorAlarm,RoutemapWarning,WarningMsg, TMModbusAlarm, SysWorkAlarm, AGVMAlarm, MotorCfgDiagAlarm
import os
import signal
import subprocess
import time
from datetime import datetime
from std_msgs.msg import Int32, String

class fake_syserr_publish():

    def __init__(self):
        rospy.loginfo("fake_syserr_publish")
        #rospy.Subscriber('input_number', String, self.fake_publisher)
        self.fake_syserr_pub = rospy.Publisher("/sysindicator/error",syserr,queue_size = 50)
        rospy.sleep(1)
        #pub = rospy.Publisher('output_string', String, queue_size=10)
        
        self.fake_publisher()

    def fake_publisher(self):
        print("fake_publisher")
        while not rospy.is_shutdown():
            robot_cmd = syserr()
            robot_cmd.errorsummary = ""
            self.fake_syserr_pub.publish(robot_cmd)
            rospy.loginfo("alarmcode:"+ robot_cmd.errorsummary)
            rospy.sleep(1)
                

    # def fake_publisher(self, input_num):
    #     print("fake_publisher")
    #     while not rospy.is_shutdown():

    #         robot_cmd = syserr()
    #         robot_cmd.errorsummary = ""
    #         if input_num.data == "1":
    #             robot_cmd.errorsummary = "TB1000"
    #             self.fake_syserr_pub.publish(robot_cmd)
    #             rospy.loginfo("alarmcode:"+ robot_cmd.errorsummary)
    #             #print("AAAAAA")
    #             input_num.data = ""
    #             time.sleep(1)
    #         else:
    #             print("clear")
    #             break

    # def fake_publisher(self, input_num):
    #     print("fake_publisher")
    #     robot_cmd = syserr()
    #     robot_cmd.errorsummary = ""
    #     if input_num.data == "1":
    #         print("AAAAAAA")
    #         rospy.spinOnce()
    #         #rate = rospy.Rate(1) 
    #         #robot_cmd.errorsummary = "TB1000"
    #         #self.fake_syserr_pub.publish(robot_cmd)
    #         #rospy.loginfo("alarmcode:"+ robot_cmd.errorsummary)
    #         #rate.sleep()
    #         #robot_cmd.errorsummary = ""
    #         #rospy.loginfo("clear alarmcode:"+ robot_cmd.errorsummary)
    #     elif input_num.data == "2":
    #         print("BBBBBBB")
    #         #rate = rospy.Rate(1) 
    #         #robot_cmd.errorsummary = ""
    #         #self.fake_syserr_pub.publish(robot_cmd)
    #         #rospy.loginfo(robot_cmd.errorsummary)
    #         #rate.sleep()


if __name__ == '__main__':
    try:    
        rospy.init_node('fake_syserr_publish')
        rospy.loginfo(rospy.get_name() + " start")
        ab =fake_syserr_publish()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Error in main function")






# #!/usr/bin/env python
# import rospy
# from std_msgs.msg import Int32, String

# def input_callback(input_num):
#     output_str = ""

#     if input_num.data == "1":
#         output_str = "AAA"
#     elif input_num.data == "2":
#         output_str = ""

#     rospy.loginfo(output_str)
#     pub.publish(output_str)

# if __name__ == '__main__':
#     rospy.init_node('string_publisher', anonymous=True)

#     rospy.spin()
