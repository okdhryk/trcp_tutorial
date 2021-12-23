#!/usr/bin/env python

import sys

import rospy
rospy.init_node("init_arm")
from utils import *



if sys.argv[1] == 'n':
    move_arm_neutral()
elif sys.argv[1] == 'i':
    move_arm_init()
elif sys.argv[1] == 'gc':
    move_hand(0)
elif sys.argv[1] == 'go':
    move_hand(1)
