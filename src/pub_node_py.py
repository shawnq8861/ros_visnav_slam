#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def main():
    pub = rospy.Publisher('py_pub_hello', String, queue_size=1000)
    rospy.init_node('pub_node_py', anonymous=True)
    rate = rospy.Rate(2) # 2hz
    loopCount = 0
    while not rospy.is_shutdown():
        loopCount += 1
        hello_str = "python loop count = %d" % loopCount
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

main()
