#!/usr/bin/env python

import rospy

import tf2_ros
import tf2_geometry_msgs

import PyKDL

from semanticmap.msg import SemanticMapGrid
from semanticmap.msg import SemanticMapMetaData

from nav_msgs.msg import Path

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion


class ConvertSemanticMapToOccupancyGrid(object):

    def __init__(self):

        self.pub_map = 
        self.pub_map_metadata
