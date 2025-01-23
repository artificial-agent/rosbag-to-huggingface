#!/usr/bin/env python3
###############################################################################################################
"""
artificial-agent
09-25-2024
"""
"""
nav_msgs.py
"""
###############################################################################################################


###############################################################################################################
# External imports
from genpy import Message, Time
import numpy as np
# Internal Imports
from rosbag_preprocess.helpers import format_value
###############################################################################################################


###############################################################################################################
def process_odometry(msg: Message, time_stamp: Time, extra_options: dict=None) -> dict:
    return {
        "seq": msg.header.seq,
        "stamp": msg.header.stamp,
        "frame_id": msg.header.frame_id,

        "x": format_value(msg.pose.pose.position.x, 16),
        "y": format_value(msg.pose.pose.position.y, 16),
        "z": format_value(msg.pose.pose.position.z, 16),

        "qx": format_value(msg.pose.pose.orientation.x, 16),
        "qy": format_value(msg.pose.pose.orientation.y, 16),
        "qz": format_value(msg.pose.pose.orientation.z, 16),
        "qw": format_value(msg.pose.pose.orientation.w, 16),

        "vx": format_value(msg.twist.twist.linear.x, 16),
        "vy": format_value(msg.twist.twist.linear.y, 16),
        "vz": format_value(msg.twist.twist.linear.z, 16),

        "wx": format_value(msg.twist.twist.angular.x, 16),
        "wy": format_value(msg.twist.twist.angular.y, 16),
        "wz": format_value(msg.twist.twist.angular.z, 16),
    }


###############################################################################################################

# EOF