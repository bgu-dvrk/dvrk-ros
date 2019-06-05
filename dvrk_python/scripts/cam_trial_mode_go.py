#!/usr/bin/env python
# export ROS_IP=132.72.12.206
# export ROS_MASTER_URI=http://132.72.12.206:11311
# 
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

continue_publish = True

def callback(data):
    global continue_publish
    continue_publish = False

def talker():
    pub = rospy.Publisher('cam_trial_mode', String, queue_size=1)
    rospy.init_node('talker_cam_mode', anonymous=False)
    rospy.Subscriber("/dvrk/footpedals/coag", Joy, callback)
    rate = rospy.Rate(10) # 10hz
    counter = 0
    while counter != 10:
    #while continue_publish:
	counter = counter + 1
        send_str = 'Go'
        rospy.loginfo(send_str)
        pub.publish(send_str)
        rate.sleep()
    pub.publish('no_msg')

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


