#!/usr/bin/env python
# export ROS_IP=132.72.12.206
# export ROS_MASTER_URI=http://132.72.12.206:11311
# 
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('cam_trial_mode', String, queue_size=1)
    rospy.init_node('talker_cam_mode', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        send_str = 'Wait'
        rospy.loginfo(send_str)
        pub.publish(send_str)
        rate.sleep()
    pub.publish('no_msg')

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
