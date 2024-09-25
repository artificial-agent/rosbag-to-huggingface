#!/bin/bash

##########################################################################
PREFIX="$HOME/Documents/code/rosbag-to-huggingface"

BAGFILE="${PREFIX}/1-inputs"
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