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
###############################################################################################################


###############################################################################################################
def process_odometry(msg: Message, time_stamp: Time, extra_options: dict=None) -> dict:
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w

    theta = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    vz = msg.twist.twist.linear.z

    wx = msg.twist.twist.angular.x
    wy = msg.twist.twist.angular.y
    wz = msg.twist.twist.angular.z

    return {
        "seq": msg.header.seq,
        "stamp": msg.header.stamp,
        "frame_id": msg.header.frame_id,

        "x": msg.pose.pose.position.x,
        "y": msg.pose.pose.position.y,
        "z": msg.pose.pose.position.z,

        "qx": qx,
        "qy": qy,
        "qz": qz,
        "qw": qw,

        "theta": theta,

        "vx": vx,
        "vy": vy,
        "vz": vz,

        "wx": wx,
        "wy": wy,
        "wz": wz,
    }


###############################################################################################################

# EOF