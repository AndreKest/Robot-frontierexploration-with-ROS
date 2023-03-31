# Robot-frontierexploration-with-ROS
Robot frontier exploration with ROS and Gazebo

Was build for a project at my university.

- map the current available environment with Python and send the map to RVIZ for visualization
- calculate the frontiers and drive the turtlebot3 robot in GAZEBO to the frontier

------------------------------------------------------------------------------------------
### Folder structure
.<br>
├── build                   &emsp;# compiled folder with colcon<br>
├── config                  &emsp;# config folder with the configurations for Mapper and NAV2-Stack<br>
├── install                 &emsp;# setup data<br>
├── launch                  &emsp;# launch files<br>
├── log                     &emsp;# Log-files from ROS2<br>
├── map                     &emsp;# The map of the environment, must be recorded and saved before, for NAV2-Navigation<br>
├── src                     &emsp;# Own Paket (here: py_mapping)<br>
└── readme.md<br>


------------------------------------------------------------------------------------------
### Version:

Python: 3.10.6<br>
ROS2:   ROS-Humble<br>
RVIZ2:  11.2.3<br>
GAZEBO: 11.10.2<br>


------------------------------------------------------------------------------------------
### Install ROS2 and Packages

sudo apt install ros-humble-desktop-full ros-humble-gazebo-* ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* ros-humble-dynamixel-sdk python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
pip install setuptools==58.2    # otherwise i had a build error


------------------------------------------------------------------------------------------
### Start project

cd ros2_ws/                           &emsp;&emsp;# go to folder this folder<br>
build colcon                          &emsp;&emsp;# build project<br>
. install/setup.bash                  &emsp;&emsp;# start setup<br>
cd launch/                            &emsp;&emsp;# go to launch folder ./launch<br>
export TURTLEBOT3_MODEL=burger        &emsp;&emsp;# add robot for simulation<br>
ros2 launch mapping_house.launch.py   &emsp;&emsp;# starts RVIZ2, GAZEBO, und frontier_exploration


------------------------------------------------------------------------------------------
### End project

type in terminal: ctrl+c


------------------------------------------------------------------------------------------
### Note

The navigation is made with the  help of the GitHub Repository: https://github.com/SteveMacenski/nav2_rosdevday_2021<br>
The frontier calculation is implemened with the help of the GitHub Repository: https://github.com/SeanReg/nav2_wavefront_frontier_exploration<br>
The frontier calculation is implemened with the help of the Paper: https://github.com/SeanReg/nav2_wavefront_frontier_exploration