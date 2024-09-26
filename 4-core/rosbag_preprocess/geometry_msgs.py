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
###############################################################################################################


###############################################################################################################
def process_twist(msg: Message, time_stamp: Time, extra_options: dict=None) -> dict:
    vx = msg.linear.x
    vy = msg.linear.y
    vz = msg.linear.z

    wx = msg.angular.x
    wy = msg.angular.y
    wz = msg.angular.z

    return {
        "id": "2022-12-14-08-02-07",
        "time": time_stamp.to_sec(),
        "vx": vx,
        "vy": vy,
        "vz": vz,
        "wx": wx,
        "wy": wy,
        "wz": wz,
    }


###############################################################################################################

# EOF