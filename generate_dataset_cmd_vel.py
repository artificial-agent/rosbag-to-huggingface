import os

import numpy as np

import rosbag

from datasets import Dataset
import yaml


# Load bag list from yaml file
with open("baglist.yaml", "r") as f:
    baglist = yaml.load(f, Loader=yaml.FullLoader)


def cmd_vel_generator():
    for key, data in baglist.items():
        for bagfile in data:
            filename = os.path.basename(bagfile)
            filename = os.path.splitext(filename)[0]

            with rosbag.Bag(bagfile, "r") as bag:

                for topic, msg, t in bag.read_messages(topics=["/cmd_vel"]):
                    vx = msg.linear.x
                    vy = msg.linear.y
                    vz = msg.linear.z

                    wx = msg.angular.x
                    wy = msg.angular.y
                    wz = msg.angular.z

                    yield {
                        "id": filename,
                        "terrain": key,
                        "time": t.to_sec(),
                        "vx": vx,
                        "vy": vy,
                        "vz": vz,
                        "wx": wx,
                        "wy": wy,
                        "wz": wz,
                    }


ds = Dataset.from_generator(cmd_vel_generator)

# ds.push_to_hub("ajthor/spot_terrain_data", "cmd_vel", split="train")
# ds.to_csv("odom.csv")
