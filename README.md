# ORB-SLAM3-ROS

A ROS implementation of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0 that focuses on the ROS part.

This package uses ```catkin build```. Tested on Ubuntu 20.04.
## 1. Prerequisites
### Eigen3
```
sudo apt install libeigen3-dev
```
### Pangolin
```
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make
sudo make install
```
### OpenCV
Check the OpenCV version on your computer (required [at least 3.0](https://github.com/UZ-SLAMLab/ORB_SLAM3)):
```
python3 -c "import cv2; print(cv2.__version__)" 
```
On a freshly installed Ubuntu 20.04.4 LTS with desktop image, OpenCV 4.2.0 is already included. If a newer version is required (>= 3.0), follow [installation instruction](https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html) and change the corresponding OpenCV version in `CMakeLists.txt`

### (Optional) `hector-trajectory-server`
Install `hector-trajectory-server` to visualize the real-time trajectory of the camera/imu. Note that this real-time trajectory might not be the same as the keyframes' trajectory.
```
sudo apt install ros-[DISTRO]-hector-trajectory-server
```
## 2. Installation
```
cd ~/catkin_ws/src
git clone https://github.com/thien94/orb_slam3_ros.git
cd ../
catkin build
```

## 3. Run Examples on our dataset (RRC2)

### Monocular mode with  [`rrc2.bag`](https://iiitaphyd-my.sharepoint.com/:f:/g/personal/kyrylo_shyvam_students_iiit_ac_in/En_-qa3Dma9KiKjrWnTvYBgBuTb8-77ohZun10uEvyl8zw?e=Lq80Uf):

```
# In one terminal:
roslaunch orb_slam3_ros rrc_realsense_monocular.launch
# In another terminal:
rosbag play rrc2.bag # 
```
### RGB-D mode with  [`rrc2.bag`](https://iiitaphyd-my.sharepoint.com/:f:/g/personal/kyrylo_shyvam_students_iiit_ac_in/En_-qa3Dma9KiKjrWnTvYBgBuTb8-77ohZun10uEvyl8zw?e=Lq80Uf):
- Run the example:
```
# In one terminal:
roslaunch orb_slam3_ros rrc_realsense_rgbd.launch
# In another terminal:
rosbag play rrc2.bag
```

### Mono-inertial mode with [`rrc2.bag`]( https://iiitaphyd-my.sharepoint.com/:f:/g/personal/kyrylo_shyvam_students_iiit_ac_in/En_-qa3Dma9KiKjrWnTvYBgBuTb8-77ohZun10uEvyl8zw?e=Lq80Uf):
#### (Did not work smoothly for us, however worked with EuRoC dataset)
```
# In one terminal:
roslaunch orb_slam3_ros rrc_realsense_imu_mono.launch
# In another terminal:
rosbag play rrc2.bag
```
### RGBD-inertial mode with [`rrc2.bag`](https://iiitaphyd-my.sharepoint.com/:f:/g/personal/kyrylo_shyvam_students_iiit_ac_in/En_-qa3Dma9KiKjrWnTvYBgBuTb8-77ohZun10uEvyl8zw?e=Lq80Uf)
```
# In one terminal:
roslaunch orb_slam3_ros rrc_realsense_imu_rgbd.launch
# In another terminal:
rosbag play rrc2.bag
```
### RGB-D mode with your own Kinect Xbox 360
```
# In one terminal:
roslaunch orb_slam3_ros rrc_kinect.launch
# In another terminal:
rosbag play <name_of_your_bag>
```
### Monocular mode with your own Kinect Xbox 360
```
# In one terminal:
roslaunch orb_slam3_ros rrc_kinect_mono.launch
# In another terminal:
rosbag play <name_of_your_bag>
```
### Monocular mode with your own camera 
#### (Change intrinsic camera constants in yaml file)
```
# In one terminal:
roslaunch orb_slam3_ros rrc_mobile_mono.launch
# In another terminal:
rosbag play <name_of_your_bag>
```



### Save and load map 

The map file will have `.osa` extension, and is located in the `ROS_HOME` folder (`~/.ros/` by default).
#### Load map:
- Set the name of the map file to be loaded with `System.LoadAtlasFromFile` param in the settings file (`.yaml`).
- If the map file is not available, `System.LoadAtlasFromFile` param should be commented out otherwise there will be error.
#### Save map:
- **Option 1**: If `System.SaveAtlasToFile` is set in the settings file, the map file will be automatically saved when you kill the ros node.
- **Option 2**: You can also call the following ros service at the end of the session
```
rosservice call /orb_slam3/save_map [file_name]
```

## 4. ROS topics, params and services
### Subscribed topics
- `/camera/image_raw` for Mono(-Inertial) node
- `/camera/left/image_raw` for Stereo(-Inertial) node
- `/camera/right/image_raw` for Stereo(-Inertial) node
- `/imu` for Mono/Stereo/RGBD-Inertial node
- `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw` for RGBD node
### Published topics
- `/orb_slam3/camera_pose`, left camera pose in world frame, published at camera rate
- `/orb_slam3/body_odom`, imu-body odometry in world frame, published at camera rate
- `/orb_slam3/tracking_image`, processed image from the left camera with key points and status text
- `/orb_slam3/tracked_points`, all key points contained in the sliding window
- `/orb_slam3/all_points`, all key points in the map
- `/orb_slam3/kf_markers`, markers for all keyframes' positions
- `/tf`, with camera and imu-body poses in world frame
### Params
- `voc_file`: path to vocabulary file required by ORB-SLAM3
- `settings_file`: path to settings file required by ORB-SLAM3
- `enable_pangolin`: enable/disable ORB-SLAM3's Pangolin viewer and interface. (`true` by default)

### Services
- `rosservice call /orb_slam3/save_map [file_name]`: save the map as `[file_name].osa` in `ROS_HOME` folder.
- `rosservice call /orb_slam3/save_traj [file_name]`: save the estimated trajectory of camera and keyframes as `[file_name]_cam_traj.txt` and  `[file_name]_kf_traj.txt` in `ROS_HOME` folder.

### Docker
(Not tested completely)
Provided [Dockerfile](Dockerfile) sets up an image based a ROS noetic environment including RealSense SDK

To access a USB device (such as RealSense camera) inside docker container use:
``` bash
docker run --network host --privileged -v /dev:/dev -it [image_name]
```
## 5. Results:
### Monocular ORB-SLAM on RRC2
[![ORB-SLAM3](https://img.youtube.com/vi/oT5Hyu_RLBI/0.jpg)](https://youtu.be/oT5Hyu_RLBI)
* Video showing working of ORB-SLAM3. Here, we have 2000 ORB features and 7 levels of ORB. 
* Interestingly, ORB-SLAM3 loses way, but is able to relocalize in the last moment. 

### Monocular ORB-SLAM on RRC2 with different ORB parameters
[![ORB-SLAM3](https://img.youtube.com/vi/np4mIMJZ9ro/0.jpg)](https://youtu.be/np4mIMJZ9ro)
* Video showing working of ORB-SLAM3. Here, we have 2000 ORB features and 8 levels of ORB.
* Interestingly, ORB-SLAM3 is never lost.

### Monocular ORB-SLAM on RRC2 + Colmap for post-processing
[![ORB-SLAM3](https://img.youtube.com/vi/QnV6setiRaY/0.jpg)](https://youtu.be/QnV6setiRaY)
* We applied Colmap given the poses coming from ORB-SLAM3. Dense reconstruction coming is great for monocular method. 

### Same as previous but Poisson Mesh
[![ORB-SLAM3](https://img.youtube.com/vi/u4ZnioRhbQM/0.jpg)](https://youtu.be/u4ZnioRhbQM)
* Doing Poisson meshing on Dense Point cloud from last link.

### Results of reconstruction from normal mobile phone
#### Actual video
[![ORB-SLAM3](https://img.youtube.com/vi/4YCMZwmzOUs/0.jpg)](https://youtu.be/4YCMZwmzOUs)
#### Point cloud (ORB-SLAM3 + Colmap)
[![ORB-SLAM3](https://img.youtube.com/vi/J8unIvbHWsE/0.jpg)](https://youtu.be/J8unIvbHWsE)
#### Mesh (ORB-SLAM3 + Colmap)
[![ORB-SLAM3](https://img.youtube.com/vi/OvDghpJSHFE/0.jpg)](https://youtu.be/OvDghpJSHFE)

