# Winterhack Revenge

**I'll fix this fxxking hiwonder bot.**

## Use

1. Firstly, make sure you installed ros2 humble.
2. Clone the repository at where you preferred.
3. Compile. `cd winterhack_revenge` and `colcon build`.
4. Setup environment variables. `source install/setup.bash`

## Packages under winterhack_revenge/src

### oradar_lidar

The lidar driver. Downloaded from [ORADAR lidar's official website](https://www.orbbec.com.cn/index/Download2025/info.html?cate=121&id=1).
Made a few changes to fit the ros2 humble.

`ros2 launch oradar_lidar ms200_scan.launch.py`

Above command will launch a TF node and a lidar node.
TF is temporary; we will continue to handle it later.  
Lidar node will publish a /scan topic. The parameters are in src/oradar_lidar/ms200_scan.launch.py


## Todo

- [x] Lidar drive
- [x] Motor drive
- [ ] Slam
- [ ] Nav2\(depend on motor drive and slam\)
