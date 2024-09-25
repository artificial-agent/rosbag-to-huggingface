# rosbag-to-huggingface
Set of scripts to convert ROS bags to hugging face datasets.


## Enviroment Setup

### Install robostack
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

### Install pip deps
1. `pip install datasets`