# Urban Terrain Dataset

<div align="center">
    <br>
<div>

[🛠️ Installation](#get-the-data) | [🎥 Video]() | [📖 Paper](https://ieeexplore.ieee.org/document/10468651)
<br>

<div align="left">
<div>

The dataset corresponding to the paper *['Learning Self-supervised Traversability with Navigation Experiences of Mobile Robots: A Risk-aware Self-training Approach,'](https://ieeexplore.ieee.org/document/10468651)* accepted for publication in RA-L on Feb, 2024. 
<p align='center'>
    <img src="./config/Learned LiDAR Traversability (Urban Campus).gif" alt="demo" width="800"/>
</p>

Our task of interests are: **terrain mapping**; **ground obstacle detection**; and the estimation of ***'robot-specific'* traversability**.

## Why we made custom dataset
While well-known datasets like [KITTI]() provide extensive data for robotic perception, they often fall short in addressing the specific needs of learning robot-specific traversability. That is, KITTI and similar datasets such as [Cityscapes]() and [nuScenes](), are mainly designed for general applications and may not capture the unique environmental and operational challenges faced by specific robots. On the other hand, the robot's own navigation experiences provide rich contextual information that is crucial for understanding and navigating complex urban terrains.

Given a robotic platform, we collected urban terrain data from its onboard measurements and labeled them by just using a simple manual driving experience of the robot. Here are some good reasons of using onboard measurements and the robot's own navigation experiecne for the application of learning robot-specific traversability:
- **Data Scalability**: Leveraging the robot's own sensors and navigation experiences allows for the collection of large-scale datasets without the need for extensive manual data annotation efforts. This approach enables continuous and automated data gathering as the robot operates, facilitating the creation of extensive datasets that capture diverse environmental conditions and scenarios. All we have to do is to manually drive the robot in the target environment, which is typically done when constructing the map of the environment with the aid of SLAM system.
- **Robot-Environment Adaptability**: Data collected directly from the robot’s sensors ensures that the training data is highly relevant to the specific robot and its operating environment. This method allows the model to adapt to the unique characteristics of the robot, such as its locomotion capabilities and the given sensor configurations, leading to more accurate learning and predictions of traversability.

## About the dataset
<p align='left'>
    <img src="config/ISR_M3.png" height="160"/>
    <img src="config/label_generation.png" height="150"  style="margin-right: 20px;"/>
    <img src="config/label_generation.gif" height="150"/>
</p>

- **Data Format:** Our datasets are provided as the files with [rosbag](https://wiki.ros.org/rosbag) format. For more information about the rosbag, see [rosbag/Tutorials](https://wiki.ros.org/rosbag/Tutorials/Recording%20and%20playing%20back%20data) and [rosbag/API Documentation](https://docs.ros.org/en/melodic/api/rosbag/html/).

- **What's in our dataset?:**   In each file of our datasets, the following ROS messages are provided:
  - LiDAR measurements (`velodyne_msgs/VelodyneScan`)
  - IMU measurements (`sensor_msgs::Imu`)
  - Odometry pose (`/tf`)
  - Extrinsic parameters of the sensors (`/tf_static`)
> **[Note]:** To reduce the size of datasets, only the *packet messages* of LiDAR sensor were recorded. This means that we have to unpack the lidar packets for playback the recorded point cloud measurements. For the purpose, there is a `vlp16packet_to_pointcloud.launch` file that handles the conversion of lidar packet to point clouds. 
  </br>

- **Robotic Platform:** Two-wheeled differential-drive robot, [ISR-M3](https://github.com/Ikhyeon-Cho/isr_robot_ros/tree/isr_m3/ros1), was used to collect the datasets. The robot was equipped with a single 3D LiDAR and IMU. During the experiments, 3D pose of the robot was estimated by the use of a Lidar-inertial odometry (LIO) system.

- **Environments:** We mainly provide two datasets with distinct ground surface characteristics. The training (blue) / testing (red) trajectories of a robot are shown in the aerial images below.   
  - **Urban campus**: This main target environment spans approximately 510m x 460m, with a maximum elevation change of 17m. The maximum inclination of the terrain is 14 degrees. The environment mostly consists of asphalt terrain. Some damaged roads, cobblestoned pavements, and the roads with small debris are challenging.
  - **Rural farm road**: We additionally validated our approach in the unstructured environments. Farm road areas were typically unpaved dirt or gravel and included various low-height ground obstacles. 

<p align='center'>
    <img src="./config/environments.jpeg" alt="demo" width="800"/>
</p>
<p align='center'>
    <img src="./config/parking_lot.gif" alt="demo" width="400"/>
    <img src="./config/country_road.gif" alt="demo" width="400"/>
</p>



## Get the Data
### Download
Use the following links to download the datasets. 

**1. Urban Campus Dataset:** [[Google Drive](https://drive.google.com/drive/folders/1l14o5QE8Z5ldgQ0lO2D_KLKNIQVvB2Qr?usp=sharing)]
- [parking_lot.bag](https://drive.google.com/drive/folders/16df1ZbN2SjjBQvhAS9EX5fHkJ85wHgfE?usp=sharing) (-> 15 min)
- [wheelchair_ramp.bag](https://drive.google.com/drive/folders/1sk3j3hwHu3VUMTLs_DI3MVyxgch_4uQD?usp=sharing) (-> 5 min)
- [campus_south.bag](https://drive.google.com/drive/folders/1NSslJuBo6XGyjKpxsPo8HG7VAMU_hEqc?usp=sharing) (-> 40 min)
- [campus_east.bag](https://drive.google.com/drive/folders/1vKQ2pda6jSjldX7Rq-fqBiMSrf18jhfb?usp=sharing) (-> 7 min)
- [campus_north.bag](https://drive.google.com/drive/folders/1tE8ag8oqOLQ_xRGvVf8WACGu07CUyXPK?usp=sharing)
- [campus_west.bag](https://drive.google.com/drive/folders/1WeDotnT4zPD1zetEtvcp3O_dXAi7rZKO?usp=sharing)
- [campus_full.bag](https://drive.google.com/drive/folders/1URP1RMi-O8e0v7mvyIFC80K34L4J_hSt?usp=sharing) (-> about 50 min)
- [campus_full_long.bag](https://drive.google.com/drive/folders/1K8qeY7GaqmDP5BFLAzukuf1LrxLXPGok?usp=sharing) (-> 2h 18 min)
- [campus_road_camera.bag](https://drive.google.com/drive/folders/1vanOoK2dcIbze5YHum0TSm1lOO2tlqP6?usp=sharing) (-> 10 min)
- [ku_innovation_hall_4F.bag](https://drive.google.com/drive/folders/1uNsNPxqw25mWO8aBt2kjQj6W2Z6JQuL8?usp=sharing) (-> 7 min)

**2. Farm Road Dataset:** [[Google Drive](https://drive.google.com/drive/folders/168ChwknIusDksEj3KLq57-kcfWv4_ghr?usp=sharing)]
- [country_road_training.bag](https://drive.google.com/drive/folders/1WzTZGSETp41iKhOsCWsTGiAXhPxjD2Db?usp=sharing) (-> 4 min)
- [country_road_testing.bag](https://drive.google.com/drive/folders/1WzTZGSETp41iKhOsCWsTGiAXhPxjD2Db?usp=sharing) (-> 16 min)
- [greenhouse.bag](https://drive.google.com/drive/folders/15R_Qcc_lZ4OfhlLhSKGsSqk2y5_wRRcn?usp=sharing) (-> 13 min)

### Play
We provide `tta_dataset_player` ROS package for playing the datasets with the basic `rviz` visualization settings. Please follow the instructions below to play the recorded rosbag files.

### Dependencies
In order to run `tta_dataset_player` package, please install the dependencies below:
- Ubuntu (tested on 20.04)
- [ROS](https://wiki.ros.org/noetic/Installation/Ubuntu) (tested on Noetic)
- `velodyne_pointcloud` (Velodyne ROS driver for unpacking LiDAR packet msgs)

Instaling the `velodyne_pointcloud` binaries by using `apt` should work through:
```
sudo apt install ros-noetic-velodyne-pointcloud
```



### Build
We recommend to use `catkin_tools` to build the ROS packages (It is not mandatory).
Instaling the `catkin_tools` package by using `apt` should work through:
```
sudo apt install python3-catkin-tools
```

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
  ## Example: parking_lot.bag
  roslaunch tta_dataset_player parking_lot.launch  playback_speed:=4.0
  ```

<!-- ## Acknowledgement -->


## Citation
Thank you for citing [our paper](https://ieeexplore.ieee.org/document/10468651) if this helps your research projects:

```bibtex
@article{cho2024traversability,
  title={Learning Self-Supervised Traversability With Navigation Experiences of Mobile Robots: A Risk-Aware Self-Training Approach}, 
  author={Cho, Ikhyeon and Chung, Woojin},
  journal={IEEE Robotics and Automation Letters}, 
  year={2024},
  volume={9},
  number={5},
  pages={4122-4129},
  doi={10.1109/LRA.2024.3376148}
}
```