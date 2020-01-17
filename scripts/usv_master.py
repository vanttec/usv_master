#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose2D

class USVMaster:
    def __init__(self):
        self.dataframe_pub = rospy.Publisher('/vanttec_usv/usv_master/usv_data', String, queue_size=10)
