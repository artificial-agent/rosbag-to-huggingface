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
from pathlib import Path
from argparse import ArgumentParser
# External Imports
import numpy as np
from datasets import Dataset, IterableDataset, Image
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


# Main
###############################################################################################################
if __name__ == "__main__":
    # Setup
    args = parse_cmd_line()

    print(args)

###############################################################################################################
# EOF