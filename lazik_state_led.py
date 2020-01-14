#!/usr/bin/env python

import rospy
import signal
import sys
import os
from threading import Thread
from std_msgs.msg import UInt8, Bool
from rubi_server.msg import RubiBool
import time
def callback(state):
    if state.data == 0:
        msg_B.data = [True] # 0 -> manualne (domyslne) -> kolor niebieski
        msg_R.data = [False]
        msg_G.data = [False]
    if state.data == 1:
        msg_B.data = [False] # 1 -> autonomia -> kolor czerwony
        msg_R.data = [True]
        msg_G.data = [False]
    if state.data == 2:
        msg_B.data = [False] # 2 -> kolor zielony
        msg_R.data = [False]
        msg_G.data = [True]
    pub_R.publish(msg_R)
    pub_G.publish(msg_G)
    pub_B.publish(msg_B)

    print(state.data)


rospy.init_node("test_pub")

pub_R = rospy.Publisher("/rubi/boards/kurwotron/fields_to_board/LED_R", RubiBool, queue_size = 10, latch = True)
pub_G = rospy.Publisher("/rubi/boards/kurwotron/fields_to_board/LED_G", RubiBool, queue_size = 10, latch = True)
pub_B = rospy.Publisher("/rubi/boards/kurwotron/fields_to_board/LED_B", RubiBool, queue_size = 10, latch = True)

msg_R = RubiBool()
msg_G = RubiBool()
msg_B = RubiBool()

rospy.Subscriber("/lazik_state", UInt8, callback)

rospy.spin()

def hook():
    os.system("./shut_led.py")

rospy.on_shutdown(hook)
