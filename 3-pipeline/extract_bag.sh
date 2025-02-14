#!/bin/bash
##########################################################################
BAG_NAME="2025-01-22-16-39-06.bag"
##########################################################################


##########################################################################
PREFIX="$HOME/Documents/rosbag-to-huggingface"
BAGFILE="/Volumes/MY_VOLUME/MY_DATASET/${BAG_NAME}.bag"
CONFIG="${PREFIX}/3-pipeline/extract_config.yaml"
OUTPUT_DIR="${PREFIX}/2-outputs/${BAG_NAME}"
##########################################################################


##########################################################################
echo "python3 extract_bag.py"
python3 extract_bag.py \
    --bagfile "${BAGFILE}" \
    --config "${CONFIG}" \
    --output_dir "${OUTPUT_DIR}"

##########################################################################

# EOF