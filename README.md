# VRRL Tello Mapping and Detection

This repository provides a library which can be used to deploy SLAM for the DJI Tello Drone. 
The implementation makes use of the ORB-SLAM2 algorithm to create a point
cloud map by analyzing the video feed from the drone which is communicated using
ROS, with the DJITelloPy library used to receive the frames. 


## Dependencies

1. Modified ORB_SLAM2 (with point cloud outputting) [repository](https://github.com/jzijlmans/orb_slam2_mod) (All depenencies of this library must be resolved)
2. The DJITelloPy [repository](https://github.com/damiafuentes/DJITelloPy)
3. ROS Kinetic


## Build and execution example: real-time SLAM using the drone's video feed (no motion)

1. Clone the master branch of this repository as vrrl.
   ```
   git clone https://gitlab.com/Salarios77/vrrl.git vrrl
   ```
2. Add the SLAM_ROS directory to the ROS_PACKAGE_PATH by the following line to your bashrc and restart the terminal console.
   ```shell
   export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:'PATH_TO_VRRL'/vrrl/SLAM_ROS
   ```
3. Change directory to the SLAM_ROS package folder.
   ```shell
   cd vrrl/SLAM_ROS
   ```
4. Change the set (ORB_SLAM2_DIR ...) line in CMakeLists.txt to refer to the directory where the ORB_SLAM2 library is located.
   ```shell
   set (ORB_SLAM2_DIR "PATH_TO_ORB_SLAM2/ORB_SLAM2")
   ```
5. Create a build folder and build the package. Source setup.bash.
   ```shell
   mkdir build && cd build
   cmake ..
   make
   source devel/setup.bash
   ```
6. Run the Mono node from the SLAM_ROS package which will wait for the video feed from the drone.
   ```shell
   rosrun SLAM_ROS Mono ../../Calibration/ORBvoc.txt ../../Calibration/tello.yaml
   ```
7. Open a new terminal console and change directory to vrrl/Tello. Then, initiate video recording from the drone.
   ```shell
   cd vrrl/Tello
   python FrameCapture.py
   ```

## Running point cloud clustering
1. Make sure you have a distribution of python with the right packages. Have roscore running in the background.

2. Run the code to start a listener node.
   ```shell
   python3 pc_filter.py
   ```
3. Run the ros command to convert a pcd file to a pointcloud message.
   ```shell
   rosrun pcl_ros pcd_to_pointcloud path_to_pcd
   ```

   
## Other Notes

The camera parameters and ORB-SLAM2 vocabulary can be configured by modifying the files 
under vrrl/Calibration. See the ORB_SLAM2 [repository](https://github.com/raulmur/ORB_SLAM2)
for further information.

