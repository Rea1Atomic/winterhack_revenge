# Winterhack Revenge

**I'll fix this fxxking hiwonder bot.**

## 1. Use

1. Firstly, make sure you installed ros2 humble.
2. Clone the repository at where you preferred.
3. Compile. `cd winterhack_revenge` and `colcon build`.
4. Setup environment variables. `source install/setup.bash`



## 2. Packages

### 2.1 oradar_lidar

#### 2.1.1 Description

The lidar driver for oradar ms200. Downloaded from [ORADAR lidar's official website](https://www.orbbec.com.cn/index/Download2025/info.html?cate=121&id=1).  

Basicly, this driver reads and handles the lidar's data from USB port. Then publishes the data to `/scan` ROS2 topic.

> **Diff to official version**
> 1. Made a few changes to fit humble version.
> 2. I applied **linear interpolation** to it to make every msg published to `/scan` topic has fixed 450 points. 

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
    - the `base_link` -> `lidar_frame` part.

#### 2.1.6 Subscribe Topics

None



## 3. Todo

- [x] Lidar drive
- [x] Motor drive
- [ ] Slam
- [ ] Nav2\(depend on motor drive and slam\)
