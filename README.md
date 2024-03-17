# KU-TTA-Dataset

The corresponding repository to the paper *"Learning Self-supervised Traversability with Navigation Experiences of Mobile Robots: A Risk-aware Self-training Approach"*, which is accepted for publication in RA-L.

#### [[IEEE RA-L](https://ieeexplore.ieee.org/document/10468651)]  [[ArXiv]()]  [[Video]()]

**Authors:** [Ikhyeon Cho]() ([tre0430@korea.ac.kr](mailto:tre0430@korea.ac.kr)), Woojin Chung (smartrobot@korea.ac.kr)

from Intelligent Systems and Robotics (ISR) Lab at Korea University, South Korea.

<p align='center'>
    <img src="./config/Learned LiDAR Traversability (Urban Campus).gif" alt="demo" width="800"/>
</p>


 
## Dataset Overview
**Data Info:** Two-wheeled differential robot ([ISR-M3](./data/robotic_platform.pdf)) was used to collect the provided datasets. Our task of interests are: terrain mapping; ground obstacle detection; and the estimation of *robot-specific* traversability. For these purposes, a robot was equipped with a 3D LiDAR and IMU. The measured 3D point clouds capture the accurate geometry of the environment. In addition, robot poses are localized by the use of a Lidar-inertial odometry (LIO) system.

  The datasets are provided as a [rosbag](https://wiki.ros.org/rosbag) type. For more information about the rosbag, see [rosbag/Tutorials](https://wiki.ros.org/rosbag/Tutorials/Recording%20and%20playing%20back%20data). In each rosbag of our datasets, the following ROS messages are provided:
  - LiDAR measurements (`velodyne_msgs/VelodyneScan` -> see the **Note** below)
  - IMU measurements (`sensor_msgs::Imu`)
  - Robot poses (`/tf`)
  - Robot's extrinsic parameters (`/tf_static`) 

****Note:** To reduce the size of datasets, only the *packet messages* of LiDAR sensor were recorded. This means that we have to unpack the lidar packets to playback and visualize the 3D point clouds. We provide the `vlp16packet_to_pointcloud.launch` file that handles the conversion of lidar packet to point clouds.


**Environments:** We mainly provide two datasets with distinct ground surface characteristics. `Urban campus` shown in the below spans approximately 510m x 460m, with a maximum elevation change of 17m. The maximum inclination of the terrain is 14 degrees. The environment mostly consists of asphalt terrain. Some damaged roads, cobblestoned pavements, and the roads with small debris are challenging. In contrast, `Farm road` areas were typically unpaved dirt or gravel and included various low-height ground obstacles. 

The training (blue) / testing (red) trajectories of a robot are shown in the aerial images below.    

<p align='center'>
    <img src="./config/environments.jpeg" alt="demo" width="600"/>
</p>




## Download and Play
### Download
Use the following links to download the datasets. 
  - **Urban Campus Dataset:** [[Google Drive](https://drive.google.com/drive/folders/1l14o5QE8Z5ldgQ0lO2D_KLKNIQVvB2Qr?usp=sharing)]
  - **Farm Road Dataset:** [[Google Drive](https://drive.google.com/drive/folders/168ChwknIusDksEj3KLq57-kcfWv4_ghr?usp=sharing)]

### Play
We provide `tta_dataset_player` ROS package for playing the datasets with the basic `rviz` visualization settings. Please follow the instructions below to play the recorded rosbag files.

### Dependencies
In order to run `tta_dataset_player` package, please install the dependencies below:
- Ubuntu (tested on 20.04)
- [ROS 1](https://wiki.ros.org/noetic/Installation/Ubuntu) (tested on Noetic)
- `velodyne_pointcloud` (Velodyne ROS driver for unpacking LiDAR packet msgs)
- `catkin_tools` (Optional, catkin command-line tools)

Instaling the `velodyne_pointcloud` binaries by using `apt` should work through:
```
sudo apt install ros-noetic-velodyne-pointcloud
```

Instaling the `catkin_tools` package by using `apt` should work through:
```
sudo apt install python3-catkin-tools
```

### Build
Use the following commands to download and build the `tta_dataset_player` package:
  ```
  cd ~/your-ros-workspace/src
  git clone https://github.com/Ikhyeon-Cho/KU-TTA-Dataset.git
  cd ..
  catkin build tta_dataset_player   ## If not using catkin_tools, use catkin_make
  ```


### Run the player
1. Locate the downloaded rosbag files (See [Download the datasets](#download)) into `KU-TTA-Dataset/data` folder. 
2. Run the command below to play the rosbag files in `data` folder:
  ```
  ## Example: urban_campus.bag
  roslaunch tta_dataset_player urban_campus.launch  playback_speed:=4.0
  ```

### Traversability Label Generation
- TBA

## Citation
If you found this package useful, please cite our paper:

```bibtex
@article{cho2024traversability,
  title={Learning Self-Supervised Traversability With Navigation Experiences of Mobile Robots: A Risk-Aware Self-Training Approach}, 
  author={Cho, Ikhyeon and Chung, Woojin},
  journal={IEEE Robotics and Automation Letters}, 
  year={2024},
  volume={},
  number={},
  pages={},
  doi={10.1109/LRA.2024.3376148}
}
```
