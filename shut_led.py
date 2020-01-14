#!/usr/bin/env python

import rospy
import signal
import sys
from threading import Thread
from std_msgs.msg import UInt8, Bool
from rubi_server.msg import RubiBool
import time

rospy.init_node("test_pub")

pub_R = rospy.Publisher("/rubi/boards/kurwotron/fields_to_board/LED_R", RubiBool, queue_size = 10, latch = True)
pub_G = rospy.Publisher("/rubi/boards/kurwotron/fields_to_board/LED_G", RubiBool, queue_size = 10, latch = True)
pub_B = rospy.Publisher("/rubi/boards/kurwotron/fields_to_board/LED_B", RubiBool, queue_size = 10, latch = True)

msg_R = RubiBool()
msg_G = RubiBool()
msg_B = RubiBool()

msg_B.data = [False]
msg_R.data = [False]
msg_G.data = [False]
pub_R.publish(msg_R)
pub_G.publish(msg_G)
pub_B.publish(msg_B)
rospy.sleep(1)
