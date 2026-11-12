# Winterhack Revenge

**I'll fix this fxxking hiwonder bot.**

## 1. Use

1. Firstly, make sure you installed ros2 humble.
2. Clone the repository at where you preferred.
3. Compile. `cd winterhack_revenge/ros_ws/` and `colcon build`.
4. Setup environment variables. `source install/setup.bash`

## 2. Packages

---

### 2.1 oradar_lidar

#### 2.1.1 Description

The lidar driver for oradar ms200. Downloaded from [ORADAR lidar's official website](https://www.orbbec.com.cn/index/Download2025/info.html?cate=121&id=1).  

Basicly, this driver reads and handles the lidar's data from USB port. Then publishes the data to `/scan` ROS2 topic.

> **Diff to official version**
> 1. Made a few changes to fit humble version.
> 2. I applied **linear interpolation** to it to make every msg published to `/scan` topic has fixed 450 points. 
> 3. Interpolation skips large range jumps and picks the nearest raw point instead, to avoid creating floating obstacle points at object edges.

**!!! DO NOT EDIT THIS PACKAGE UNLESS IT'S NECESSARY !!!**  

#### 2.1.2 Configuration

The only thing you are adviced to change is the lidar serial port's linux device file in `/dev`. Default value is `/dev/ttyUSB0`. Change it in launch file `ms200_scan.launch.py`.

#### 2.1.3 Launch Files

- `ms200_scan.launch.py`
    - to launch all 2 nodes in 2.1.4
- `ms200_scan_view.launch.py`
    - do not use

#### 2.1.4 Nodes

- `MS200`
    - Node which read the data and publish to /scan
- `base_link_to_laser_frame` 
    - the tf publisher

#### 2.1.5 Publish Topics

- `/scan`
- `/tf`
    - the `base_link` -> `lidar_frame` part

#### 2.1.6 Subscribe Topics

None

---

### 2.2 rf2o_laser_odometry

#### 2.2.1 Description

From [rf2o_laser_odometry Github Repo](https://github.com/MAPIRlab/rf2o_laser_odometry).

This package reads and turns the lidar `/scan` data to odometry data `/odom_rf2o`.  

Why use it?  
Because slam_toolbox need odometry data to help SLAM. Our motors do not have encoder, so we use it to get odometry. The result is not particularly good. But enough to drive slam_toolbox. Consider to use multisensor fusion in the future.

#### 2.2.2 Configuration

Some topic names. Default values are enough. They are in rf2o_laser_odometry.launch.py if you want to change.

#### 2.2.3 Launch Files

- rf2o_laser_odometry.launch.py
    - launch the only node

#### 2.2.4 Nodes

- rf2o_laser_odometry

#### 2.2.5 Publish Topics

- `/odom_rf2o`
    - the odom data calc from scan data
- `/tf`
    - the `/odom` to `/base_link` part 

#### 2.2.6 Subscribe Topics

- `/scan`

---

### 2.3 hiwonder_driver

#### 2.3.1 Description

Our bot use a "Hiwonder rrc board" as a expand board of RaspberryPi. Motors, arm servos, imu are connected to this board.  
This is the ROS2 driver interacts with the rrc board.

Currently, we only read the `/cmd_vel` and drive the motor.

#### 2.3.2 Configuration

None

#### 2.3.3 Launch Files

None

#### 2.3.4 Nodes

- `hiwonder_driver`

#### 2.3.5 Publish Topics

- `/imu/data_raw`

#### 2.3.6 Subscribe Topics

- `/cmd_vel`

---

### 2.4 winterhack_bringup

#### 2.4.1 Description

The bringup package for starting the project more conveniently.

#### 2.4.2 Configuration

In winterhack_bringup/config. Some project-wide configuration.

- `nav2_params.yaml`
- `slam.yaml`

#### 2.4.3 Launch Files

- `pi.launch.py`
    > Run this launch on RaspberryPi. This file launches the nodes or other
    launch files should be launched on Pi.

- `pc.launch.py`
    > Run this on your PC.

#### 2.4.4 Nodes

None

#### 2.4.5 Publish Topics

None

#### 2.4.6 Subscribe Topics

None

---

### 2.5 landerpi_description

#### 2.5.1 Description

The robot model for our LanderPi mecanum bot. The URDF and Meshes were extract from original hiwonder repository. Modified to work with ros2_control.

#### 2.5.2 Configuration

The model is in `urdf/landerpi_mecanum.urdf`. Modify it directly to change any configuration.

#### 2.5.3 Launch Files

- `disc.launch.py`
    - load the URDF and launch `robot_state_publisher`

#### 2.5.4 Nodes

- `robot_state_publisher`

#### 2.5.5 Publish Topics

- `/robot_description`
    - the URDF of the bot
- `/tf`
    - the transforms for movable joints when joint states are received
- `/tf_static`
    - the transforms for fixed joints

#### 2.5.6 Subscribe Topics

- `/joint_states`

---

## 3. Todo or Road Map

- [x] Lidar drive
- [x] Motor drive
- [x] Slam
    > !! accuracy problem caused by odometry !!
- [x] Nav2
    > !! param opimization needed !!
- [x] Imu
- [x] Winterhack bringup launch
- [ ] Multi-sensor fusion for odometry 
- [ ] Servo driver
- [ ] move to ros_control
