#!/usr/bin/env python3
###############################################################################################################
"""
artificial-agent
09-25-2024
"""
"""
geometry_msgs.py
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
def process_twist(msg: Message, time_stamp: Time, extra_options: dict=None) -> dict:
    return {
        "stamp": time_stamp,

        "vx": format_value(msg.linear.x, 16),
        "vy": format_value(msg.linear.y, 16),
        "vz": format_value(msg.linear.z, 16),

        "wx": format_value(msg.angular.x, 16),
        "wy": format_value(msg.angular.y, 16),
        "wz": format_value(msg.angular.z, 16),
    }


def process_twist_stamped(msg: Message, time_stamp: Time, extra_options: dict=None) -> dict:
    return {
        "seq": msg.header.seq,
        "stamp": msg.header.stamp,
        "frame_id": msg.header.frame_id,

        "vx": format_value(msg.twist.linear.x, 16),
        "vy": format_value(msg.twist.linear.y, 16),
        "vz": format_value(msg.twist.linear.z, 16),

        "wx": format_value(msg.twist.angular.x, 16),
        "wy": format_value(msg.twist.angular.y, 16),
        "wz": format_value(msg.twist.angular.z, 16),
    }


###############################################################################################################

# EOF