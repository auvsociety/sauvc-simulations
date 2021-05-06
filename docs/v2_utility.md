# About v2_utility
This package contains utility ROS nodes, plugins, and other files required by **v2_control**.

## Commands
1. To view output of the location _(loc)_ sensors - IMU, depth sensors
```
rosrun v2_utility loc_sensors
```
2. To view the output of the depth sensor
```
rostopic echo /auv_v2/depth
```
3. To view the output of the IMU
```
rostopic echo /auv_v2/imu
```
4. To convert the recording of the camera output from AUV_v2's front and down facing camera in a rosbag into `.jpg` images
```
roslaunch v2_utility bag:=name_of_rosbag.bag use_frnt:=[true | false]
mv ~/.ros/frame*.jpg /path/to/v2_dataset/imgs/random_name_for_dataset
```
> Note: _name\_of\_rosbag.bag_ must be present in /path/to/v2_dataset/rosbags or else `roslaunch` will shoot an error. This is done for maintaining structure and not lose any file(s).
5. To rename the image files from `frameXXXX.jpg` to `XXXX.jpg` (where `XXXX` represents a whole number):
```
cd /path/to/v2_dataset/imgs
python rename_img.py
```
6. To obtain ground truth data
```
rostopic echo /auv_v2/gt
``` 

[Back to Home](./Home.md)