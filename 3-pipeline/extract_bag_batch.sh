#!/bin/bash
##########################################################################
PREFIX="$HOME/Documents/5-public_code/rosbag-to-huggingface"
CONFIG="${PREFIX}/3-pipeline/extract_config.yaml"
##########################################################################


##########################################################################
echo "python3 extract_bag_batch.py"
python3 extract_bag_batch.py \
    --config "${CONFIG}"

##########################################################################

# EOF