#!/usr/bin/env python

import rospy
rospy.init_node('put_obj')
from utils import *

delete_object('apple')
put_object('apple', 2.2, 4.75, 0.5)
