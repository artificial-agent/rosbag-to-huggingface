# rosbag-to-huggingface
Set of scripts to convert ROS bags to hugging face datasets.


## Enviroment Setup

### Option 1 - Install robostack (reccomended)
0. Consider installing a python version management tool like [pyenv](https://github.com/pyenv/pyenv)
1. Install [miniforge](https://github.com/conda-forge/miniforge)
2. Install mamba: `conda install mamba -c conda-forge`
3. Create mamba env: `mamba create -n ros_hugface python=3.11`
4. Activate env: `mamba activate ros_env`
5. Setup env:
    - Add the conda-forge channel: `conda config --env --add channels conda-forge`
    - Add the robostack channel: `conda config --env --add channels robostack-staging`
    - Remove the defaults channel: `conda config --env --remove channels defaults`
6. Install ROS noetic: `mamba install ros-noetic-desktop`
7. Install ROS dev tools: `mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep`

### Option 2 - Install ROS noetic (requires Ubuntu host OS - not reccommended unless already set up)
0. https://wiki.ros.org/noetic/Installation/Ubuntu

### Install pip deps
1. `pip install datasets opencv-python`

## Using this repo
1. The core script can be found in `3-pipeline/extract_bag.sh`
    1. Update `PREFIX`, `BAGFILE`, `CONFIG`, and `OUTPUT_DIR` as needed.
2. Topics to be extracted, and their settings are found in `3-pipeline/extract_config.yaml`
    1. Edit as needed, adding new topics and editing their extraction settings.
    2. If you would like to add a message that is not supported, you will have to add it (see below).

## Adding new messages
1. Add an entry to the `data_schema` in `3-pipeline/extract_config.yaml`
    - Important! Specify the `output_type` of the topic; i.e. `csv`, `dir_of_imgs`, `dir_of_pcds`, etc.
2. Update the according `data_handler` in `4-core/ros_bag_preprocess/data_handler.py` based on the decided `output_type`
    - If it is a csv, you must add the expected columns
    - To "update" means to add a line to a function call to be written.
3. Write a function which extracts the message and returns a dictionary in the appropriate messages file in `4-core/ros_bag_preprocess`.
    - e.x. If I wanted to add a a `NavSatFix` message type, I would add it to the `sensor_msgs.py` file
