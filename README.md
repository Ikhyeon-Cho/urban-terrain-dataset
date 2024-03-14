# S2TD: Self-supervised Terrain Traversability Dataset


This is the corresponding repository to the paper "Learning Self-supervised Traversability with Navigation Experiences of Mobile Robots: A Risk-aware Self-training Approach".

**Authors:** [Ikhyeon Cho]() ([tre0430@korea.ac.kr](mailto:tre0430@korea.ac.kr)), Woojin Chung

**The work is currently Under Review. The dataset will be published upon publication of the paper.**

From [Intelligent Systems and Robotics Lab](https://www.isr.korea.ac.kr/) at Korea University, Republic of Korea.

<p align='center'>
    <img src="./config/Learned LiDAR Traversability (Urban Campus).gif" alt="demo" width="800"/>
</p>

## Table of Contents
  - [Dataset dependency](#dependency)
  - [Package Installation](#install)
  - [Access to Public Datasets](#datasets)
  - [How to Play Datasets](#usage)


## Dependency
The dataset is provided with the use of [rosbag](https://wiki.ros.org/rosbag). For more information of rosbag, see [rosbag/Tutorials](https://wiki.ros.org/rosbag/Tutorials)
- [ROS 1](https://wiki.ros.org/noetic/Installation/Ubuntu) (tested on Noetic)

To find out more about the rosbag command-line tool, see [rosbag Command-line Usage](https://wiki.ros.org/rosbag/Commandline) and [Cookbook examples](https://wiki.ros.org/rosbag/Cookbook)
## Install
Use the following commands to use the tta_dataset_player package for playing rosbag files
  ```
  cd ~/your-ros-workspace/src
  git clone https://github.com/Ikhyeon-Cho/KU-TTA-Dataset.git
  cd ..
  catkin build tta_dataset_player
  ```

## Datasets
  * Download the datasets to test the functionality of the traversability estimation module. The datasets below are recorded using the outdoor settings:
    - **Urban Campus dataset:** [[Google Drive](https://drive.google.com/drive/folders/15gFqs8lKcpjT9-ycEXlfCmOTBPYSgqBJ?usp=sharing)]
    - **Farm Road dataset:** [[Google Drive](https://drive.google.com/drive/folders/15gFqs8lKcpjT9-ycEXlfCmOTBPYSgqBJ?usp=sharing)]

  * The datasets below are recorded using the indoor settings:
    - **Innovation Hall dataset:** [[Google Drive](https://drive.google.com/drive/folders/15gFqs8lKcpjT9-ycEXlfCmOTBPYSgqBJ?usp=sharing)]


## Usage
- To be updated