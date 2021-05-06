# Using DSO
## Installing DSO
1. Create a new folder and name it **cmake_ws**. This will be the CMake workspace. In this folder you will be placing all the non-ROS repositories.
2. Git clone this repository - [dso](https://github.com/NikolausDemmel/dso/tree/cmake) into the CMake workspace.
    1. Checkout the `cmake` branch.
    2. Follow the instructions in _README.md_ of the aforementioned repository, and build the targets.
3. Add the following line to `~/.bashrc`
```
export DSO_PATH=/path/to/cmake_ws/dso
```

## DSO and AUV_v2
- Git clone the _v2\_\*_ repositories into **auv_ws**. Make sure that the dependencies are installed.
- This [rosbag](https://drive.google.com/file/d/1_rANX3zwthq0yXkztWnRG6kZjRiXbiAf/view?usp=sharing) contains data from the front and down facing cameras placed on the AUV_v2 vehicle (Size 5 GB). You can download the rosbag and execute the following command to see the actual motion:
```
roslaunch v2_control auv_vision.launch
rosbag play sauvc_simpleRun.bag
``` 
Downloading the rosbag is not mandatory. The images present in the _v2\_dataset_ repository will be utilized (approximate size 150 MB). Please refer to the documentation of _v2\_utility_ to learn about conversion of data from rosbag to images. <br>
To obtain the results in the [video](https://youtu.be/-gvOdPLldr4), perform the following steps:
1. Make sure _dso_ repository is built in the cmake workspace.
2. Copy and paste the executable named `dso_dataset` from `/path/to/cmake_ws/dso/build/bin` to `/path/to/v2_dataset/imgs` or to the folder containing the folder of dataset images and camera calibration TXT file.
3. Execute the following command:
```
./dso_dataset files=./simpleRun_frnt calib=./cam_calib_dso.txt mode=2 preset=0
```
> _README_ of _dso_ repository explains the meaning and reason behind the above command.<br>
> Refer [here](./v2_control.md) for commands to execute DSO with AUV\_v2 simulation.

## Challenges for implementing DSO
1. The scuba divers might come in the field of view of camera. This would make the environment dynamic instead of static. (Note: DSO works well for static environment)
2. The sunlight scattering at the bottom of the SAUVC pool would also make the environment dynamic. The problem is same as the one listed in the "Underwater SLAM Scenario" page.
3. Need to develop the complete back-end.
4. No idea how real world behaviour would be due to lack of datasets.

### Known issues with DSO
1. The Z direction motion is flipped, i.e., when the bot is moving up, the estimated position in the DSO GUI is moving down and vice versa.
2. Sometimes the estimated path is erratic. Reason behind it is unknown as of now; need to perform more experiments to understand.

### TODO
#### Phase 1: Understanding DSO via simulation
- -[x] 1) Assess and list down the issues presented by the initial run of DSO over AUV_v2's simulation dataset. You may play around with the parameters in the GUI of DSO and try to find out the setting that gives the best result.
- -[x] 2) Understand and implement DSO in real-time by feeding the algorithm output of topic `/auv_v2/f_cam/image_raw`.
- -[x] 3) Understand the output of DSO.
  - -[x] 3.1) Implement heave, yaw PID loop, and PID to effort(/PWM) conversion block.
  - -[x] 3.2) Develop a program to obtain ground truth displacement from Gazebo.
  - -[x] 3.3) Find out how the real world displacement and DSO output are mapped.
  - -[x] 3.4) Understand how DSO output and DSO visualisation are mapped.

#### Phase 2: Testing DSO with real world information
- -[x] 1) Get/develop an Android application that streams camera output to a ROS node.
- -[ ] 2) Run DSO with underwater datasets obtained from various sources.
- -[x] 3) Assess the results.

> Only after the completion of Phase 2, we would decide on what needs to be done next with DSO.

## Conclusion
After performing the tests, we won't be relying on DSO or any visual odometry technique. We will be proceeding without using SLAM _(given that hydrophones are not ready yet)_

### Links to test results
[DSO Test 1](./DSO-Test-1.md)

[DSO Test 2](./DSO-Test-2.md)

[Back to Home](./Home.md)