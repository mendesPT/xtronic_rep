#!/usr/bin/env python

# Description: util was designed to define required utilities used in the MonarchState features.


import math

from geometry_msgs.msg import *
from move_base_msgs.msg import *
from std_msgs.msg import *


def pose2pose_stamped(x, y, t):
        """Helper function for creation of a PoseStamped message"""
        pose_stamped = PoseStamped(header=Header(frame_id="/map"), pose=Pose(position=Point(x, y, 0), orientation=Quaternion(0, 0, math.sin(t/2.0), math.cos(t/2.0))))
        return pose_stamped

