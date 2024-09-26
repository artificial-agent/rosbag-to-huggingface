#!/usr/bin/env python3
###############################################################################################################
"""
artificial-agent
09-25-2024
"""
"""
sensor_msgs.py
"""
###############################################################################################################


###############################################################################################################
# External imports
from genpy import Message, Time
import numpy as np
import cv2
from cv_bridge import CvBridge
from PIL import Image
from PIL.PngImagePlugin import PngInfo
###############################################################################################################


###############################################################################################################
cv_bridge = CvBridge()

###############################################################################################################


###############################################################################################################
def process_img(msg: Message, time_stamp: Time, extra_options: dict) -> dict:
    # So we dont have to keep instantiating
    global cv_bridge

    # Deserialize message
    header = msg.header
    image_np = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

    # Resize if necassary
    if extra_options.get("image_size", None) is not None:
        image_np = resize_image_h_w(image_np, extra_options["image_size"]["width"], extra_options["image_size"]["height"])

    # Wrap into PIL
    pil_image = Image.fromarray(image_np)

    # Add metadata
    img_metadata = PngInfo()
    img_metadata.add_text("ros::seq", str(header.seq))
    img_metadata.add_text("ros::stamp:secs", str(header.stamp.secs))
    img_metadata.add_text("ros::stamp:nsecs", str(header.stamp.nsecs))
    img_metadata.add_text("ros::frame_id", str(header.frame_id))

    return{
        "img": pil_image,
        "img_metadata": img_metadata
    }


def process_compressed_img(msg: Message, time_stamp: Time, extra_options: dict) -> dict:
    # So we dont have to keep instantiating
    global cv_bridge

    # Deserialize message
    header = msg.header
    image_np = cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="rgb8")

    # Resize if necassary
    if extra_options is not None:
        if extra_options.get("image_size", None) is not None:
            image_np = resize_image_h_w(image_np, extra_options["image_size"]["width"], extra_options["image_size"]["height"])

    # Wrap into PIL
    pil_image = Image.fromarray(image_np)

    # Add metadata
    img_metadata = PngInfo()
    img_metadata.add_text("ros::seq", str(header.seq))
    img_metadata.add_text("ros::stamp:secs", str(header.stamp.secs))
    img_metadata.add_text("ros::stamp:nsecs", str(header.stamp.nsecs))
    img_metadata.add_text("ros::frame_id", str(header.frame_id))

    return{
        "img": pil_image,
        "img_metadata": img_metadata
    }


###############################################################################################################


###############################################################################################################
def resize_image_h_w(image_np: np.ndarray, width: int, height: int) -> np.ndarray:
    if image_np.shape[1] == width and image_np.shape[0] == height:
        return image_np
    else:
        image_np = cv2.resize(image_np, (width, height))
        return image_np


###############################################################################################################

# EOF