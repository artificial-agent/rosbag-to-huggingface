#!/usr/bin/env python3
###############################################################################################################
"""
artificial-agent
09-25-2024
"""
"""
extract_bag.py
"""
"""
DESC:
    - script for extracting rosbags and converting them to hugging face datasets
"""
###############################################################################################################


###############################################################################################################
# Core Imports
import sys
import yaml
import csv
from pathlib import Path
from argparse import ArgumentParser
# External Imports
import rosbag
from tqdm import tqdm
import numpy as np
from datasets import Dataset, IterableDataset, Image
# Internal Imports
from rosbag_preprocess.data_handler import get_message_cols, process_message
###############################################################################################################


# Script setup
###############################################################################################################
def parse_cmd_line() -> dict:
    # Instantiate base argument parser
    parser = ArgumentParser(usage='USAGE: extract_bag.py')

    # Setup args
    parser.add_argument("--bagfile",    type=str,   default=f"canonical.bag",       help="Absolute location of rosbag")
    parser.add_argument("--config",     type=str,   default=f"extract_config.yaml", help="Absolute location of config file")
    parser.add_argument("--output_dir", type=str,   default=f"2-outputs",           help="Directory to ouput data products")

    # Parse args
    args = parser.parse_args()

    # Create output DIR if it does not exist
    Path(args.output_dir).mkdir(parents=True, exist_ok=True)

    # Return
    return {
        "bagfile": args.bagfile,
        "config": args.config,
        "output_dir": args.output_dir
    }


###############################################################################################################


# Extraction functions
###############################################################################################################
def extract_single(rosbag_abs_path: str, extraction_config: dict, output_dir: str) -> None:
    #! Setup
    rosbag_topics = [ topic_info["rosbag_topic"] for topic_info in extraction_config["data_schema"] ]
    hugface_names = [ topic_info["hugface_name"] for topic_info in extraction_config["data_schema"] ]
    csv_writers = {} # Dictionary to keep track of open CSV writers by topic type
    topic_msg_counts = {topic: 0 for topic in rosbag_topics} # Dictionary to keep track of the count of each topic

    #! Main Loop
    try:
        # Open bag w/ read access
        with rosbag.Bag(rosbag_abs_path, 'r') as bag:
            # Completion bar
            with tqdm(total=bag.get_message_count(), desc="Extracting topics...", file=sys.stdout) as progress_bar:
                # Loop through bag
                for topic, msg, time_stamp in bag.read_messages():
                    msg_type = msg._type

                    # Only extract specified topics
                    if topic in rosbag_topics:
                        topic_idx = topic_msg_counts[topic]
                        topic_config = [ topic_info for topic_info in extraction_config["data_schema"] if topic_info["rosbag_topic"] == topic ][0] # do this up front for speedup

                        # Conditionally extract msgs based on start, end, and throttle rate
                        if (topic_idx >= topic_config["start_idx"]) and (topic_idx < topic_config["end_idx"]) and (topic_idx % topic_config["throttle_rate"] == 0):

                            #! Switch based on output type (csv, png, pcd, etc)
                            if topic_config["output_type"] == "csv":
                                if topic not in csv_writers:
                                    # Create a new CSV file for this message type
                                    csv_file_path = Path(f'{output_dir}/{topic_config["hugface_name"]}.csv')
                                    csv_file = open(csv_file_path, 'w', newline='')

                                    # Initialize a CSV writer for this file
                                    writer = csv.DictWriter(csv_file, get_message_cols(msg_type))
                                    writer.writeheader() # Write the header (fields of the message)
                                    csv_writers[topic] = {"file": csv_file, "writer": writer, "msg_type": msg_type}

                                # Process the message and write it to the corresponding CSV
                                processed_data = process_message(msg_type, msg, time_stamp)
                                csv_writers[topic]["writer"].writerow(processed_data)

                            elif topic_config["output_type"] == "dir_of_files":
                                processed_data = process_message(msg_type, msg, time_stamp)
                                # save()


                    # Bookkeeping
                    try:
                        topic_msg_counts[topic] += 1
                    except:
                        topic_msg_counts[topic] = 1

                    progress_bar.update(1)

    finally:
        # Close all open CSV files
        for key, value in csv_writers.items():
            value["file"].close()


def extract_all(args: dict, extraction_config: dict) -> None:
    pass


###############################################################################################################


# Main
###############################################################################################################
if __name__ == "__main__":
    # Setup
    args = parse_cmd_line()

    # Load YAML config file
    with open(args["config"]) as stream:
        try:
            extraction_config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # Infer end idx
    for topic_info in extraction_config["data_schema"]:
        if topic_info["end_idx"] == -1:
            topic_info["end_idx"] = int(9E15)

    # Infer if single bag or dir of bags & extract data
    if f"{args['bagfile']}"[-4:] == ".bag" :
        extract_single(args["bagfile"], extraction_config, args["output_dir"])
    else:
        extract_all(args["bagfile"], extraction_config, args["output_dir"])


###############################################################################################################
# EOF