#!/usr/bin/env python3
###############################################################################################################
"""
artificial-agent
09-25-2024
"""
"""
data_handler.py
"""
###############################################################################################################


###############################################################################################################
# Core Imports
from typing import List, Dict
# External Imports
from genpy import Message, Time
import numpy as np
from PIL import Image
from PIL.PngImagePlugin import PngInfo

# Internal Imports
from rosbag_preprocess.nav_msgs import process_odometry
from rosbag_preprocess.geometry_msgs import process_twist
from rosbag_preprocess.sensor_msgs import process_img, process_compressed_img
###############################################################################################################


###############################################################################################################
def get_msg_cols(msg_type: str) -> List[str]:
    if msg_type == "nav_msgs/Odometry":
        return [ "id", "time", "x", "y", "z", "theta", "vx", "vy", "vz", "wx", "wy", "wz" ]

    elif msg_type == "geometry_msgs/Twist":
        return [ "id", "time", "vx", "vy", "vz", "wx", "wy", "wz" ]

    else:
        raise NotImplementedError


def process_msg_csv(msg_type: str, msg: Message, time_stamp: Time, extra_options: dict=None) -> dict:
    if msg_type == "nav_msgs/Odometry":
        formatted_msg = process_odometry(msg, time_stamp, extra_options)

    elif msg_type == "geometry_msgs/Twist":
        formatted_msg = process_twist(msg, time_stamp, extra_options)

    else:
        raise NotImplementedError

    return formatted_msg


def process_msg_img(msg_type: str, msg: Message, time_stamp: Time, extra_options: dict=None) -> Dict[str: Image.Image, str: PngInfo]:
    if msg_type == "sensor_msgs/Image":
        formatted_msg = process_img(msg, time_stamp, extra_options)

    elif msg_type == "sensor_msgs/CompressedImage":
        formatted_msg = process_compressed_img(msg, time_stamp, extra_options)

    else:
        raise NotImplementedError

    return formatted_msg


###############################################################################################################

# EOF