import os

import numpy as np

import rosbag

from datasets import Dataset
import yaml


# Load bag list from yaml file
with open("baglist.yaml", "r") as f:
    baglist = yaml.load(f, Loader=yaml.FullLoader)


def odom_generator():
    for key, data in baglist.items():
        for bagfile in data:
            filename = os.path.basename(bagfile)
            filename = os.path.splitext(filename)[0]

            with rosbag.Bag(bagfile, "r") as bag:

                for topic, msg, t in bag.read_messages(topics=["/odom"]):
                    x = msg.pose.pose.position.x
                    y = msg.pose.pose.position.y
                    z = msg.pose.pose.position.z

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

                    yield {
                        "id": filename,
                        "terrain": key,
                        "time": t.to_sec(),
                        "x": x,
                        "y": y,
                        "z": z,
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


ds = Dataset.from_generator(odom_generator)

ds.push_to_hub("ajthor/spot_terrain_data", "odom", split="train")
# ds.to_csv("odom.csv")
