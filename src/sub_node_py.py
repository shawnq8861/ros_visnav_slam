#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def sub_callback(msg):
    rospy.loginfo("subscriber received:  %s" % msg.data)

def main():
    rospy.init_node('sub_node_py', anonymous=True)
    sub = rospy.Subscriber('py_pub_hello', data_class=String,
                            queue_size=1000, callback=sub_callback)
    rospy.spin()

main()
