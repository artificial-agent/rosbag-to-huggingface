from datasets import Dataset, IterableDataset, Image

import numpy as np

import rosbag
import rospy

import yaml

# Import opencv tools for CompressedImage
import cv2
from cv_bridge import CvBridge

from PIL import Image

# with rosbag.Bag('hf-datasets/ros/spot_sample_bag_all_data.bag', "r") as bag:
#     for topic, msg, t in bag.read_messages():


bagpath = "hf-datasets/ros/2022-12-14-08-02-07.bag"


with rosbag.Bag(bagpath, "r") as bag:

    topics = bag.get_type_and_topic_info()[1].keys()
    types = []
    for val in bag.get_type_and_topic_info()[1].values():
        types.append(val[0])

    def odometry_generator():
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
                "id": "2022-12-14-08-02-07",
                "time": t.to_sec(),
                "x": x,
                "y": y,
                "z": z,
                "theta": theta,
                "vx": vx,
                "vy": vy,
                "vz": vz,
                "wx": wx,
                "wy": wy,
                "wz": wz,
            }

    def commands_generator():
        for topic, msg, t in bag.read_messages(topics=["/cmd_vel"]):
            vx = msg.linear.x
            vy = msg.linear.y
            vz = msg.linear.z

            wx = msg.angular.x
            wy = msg.angular.y
            wz = msg.angular.z

            yield {
                "id": "2022-12-14-08-02-07",
                "time": t.to_sec(),
                "vx": vx,
                "vy": vy,
                "vz": vz,
                "wx": wx,
                "wy": wy,
                "wz": wz,
            }

    def image_raw_generator():
        for topic, msg, t in bag.read_messages(
            topics=["/camera/rgb/image_raw/compressed"]
        ):
            bridge = CvBridge()
            cv_image = bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            image = Image.fromarray(cv_image)

            yield {
                "id": "2022-12-14-08-02-07",
                "time": t.to_sec(),
                "image": image,
            }

    # - gps
    # - odometry
    # - commands
    # - lidar
    # - camera
    # - radar

    ds = Dataset.from_generator(odometry_generator)
    ds.to_csv("hf-datasets/ros/spot_terrain/odom.csv")

    ds = Dataset.from_generator(commands_generator)
    ds.to_csv("hf-datasets/ros/spot_terrain/cmd_vel.csv")

    ds = Dataset.from_generator(commands_generator)
    ds.to_csv("hf-datasets/ros/spot_terrain/cmd_vel.csv")

    # ds = IterableDataset.from_generator(
    #     odom_generator, gen_kwargs={"gen": bag.read_messages(topics=["/odom"])}
    # )


print("done")
