from datasets import load_dataset

import numpy as np

import rosbag
import rospy

import yaml


ds_odometry = load_dataset("ajthor/viikd_spot_terrain", "odom")
ds_commands = load_dataset("ajthor/viikd_spot_terrain", "cmd_vel")

# Filter out the data for the first bag.
bagname = "2022-12-14-08-02-07"

ds_bag_odometry = ds_odometry.filter(lambda x: x["id"] == bagname)


ds_odometry["train"]

print("here")
