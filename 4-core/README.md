# rosbag_preprocess

## description
helper functions to extract raw data from rosbags

## Adding new messages
1. Add an entry to the `data_schema` in `3-pipeline/extract_config.yaml`
    - Important! Specify the `output_type` of the topic; i.e. `csv`, `dir_of_imgs`, `dir_of_pcds`, etc.
2. Update the according `data_handler` in `4-core/ros_bag_preprocess/data_handler.py` based on the decided `output_type`
    - If it is a csv, you must add the expected columns
    - To "update" means to add a line to a function call to be written.
3. Write a function which extracts the message and returns a dictionary in the appropriate messages file in `4-core/ros_bag_preprocess`.
    - e.x. If I wanted to add a a `NavSatFix` message type, I would add it to the `sensor_msgs.py` file
4. Consider submitting it as a MR!