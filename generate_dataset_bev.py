import os

import numpy as np

import rosbag
import rospy

from datasets import Dataset, load_dataset, concatenate_datasets, Image as HfImage
import yaml


import cv2
from cv_bridge import CvBridge

from PIL import Image
from PIL.PngImagePlugin import PngInfo


# Load bag list from yaml file
with open("baglist.yaml", "r") as f:
    baglist = yaml.load(f, Loader=yaml.FullLoader)

feature = HfImage(decode=False)


def image_raw_generator():

    count = 0

    for key, data in baglist.items():
        for bagfile in data:
            filename = os.path.basename(bagfile)
            filename = os.path.splitext(filename)[0]

            img_path = bagfile
            img_path = os.path.join(*img_path.split(os.path.sep)[2:])  # Remove the path
            img_path = os.path.splitext(img_path)[0]  # Remove the extension

            target_dir = os.path.join("bev", "train", img_path)

            bag = os.path.basename(bagfile)
            bag = os.path.splitext(bag)[0]  # Remove the extension

            with rosbag.Bag(bagfile, "r") as bag:

                for topic, msg, t in bag.read_messages(
                    topics=["/bev/single/compressed"]
                ):

                    bridge = CvBridge()
                    cv_image = bridge.compressed_imgmsg_to_cv2(
                        msg, desired_encoding="rgb8"
                    )
                    image = Image.fromarray(cv_image)

                    header = msg.header

                    img_metadata = PngInfo()
                    img_metadata.add_text("ros::seq", str(header.seq))
                    img_metadata.add_text("ros::stamp:secs", str(header.stamp.secs))
                    img_metadata.add_text("ros::stamp:nsecs", str(header.stamp.nsecs))
                    img_metadata.add_text("ros::frame_id", str(header.frame_id))

                    file_name = f"{count:08}.png"
                    count += 1

                    # img_path = bagfile
                    # img_path = os.path.splitext(img_path)[0]
                    target_dir = os.path.join("bev", "train")
                    img_path = os.path.join(target_dir, file_name)
                    # img_path = os.path.join(*img_path.split(os.path.sep)[2:])
                    # img_path = os.path.join("bev", img_path)

                    # Create the path if it doesn't exist
                    os.makedirs(target_dir, exist_ok=True)

                    # Save the image
                    image.save(img_path, "PNG", pnginfo=img_metadata)

                    yield {
                        "id": filename,
                        "terrain": key,
                        "time": t.to_sec(),
                        "file_name": file_name,
                        # "metadata": img_metadata,
                    }


metadata = Dataset.from_generator(image_raw_generator)

target_dir = os.path.join("bev", "train")
metadata.to_csv(os.path.join(target_dir, "metadata.csv"))

ds = load_dataset("imagefolder", data_dir="bev")
ds.push_to_hub("ajthor/spot_terrain_data", "bev")
