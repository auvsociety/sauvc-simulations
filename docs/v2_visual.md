# About v2_visual
This package contains all the files that help in visualising the SAUVC-2020 swimming pool and auv_v2 vehicle. It can be independently installed.

## Commands
1.  To view a *URDF* model in RViz. 
```
roslaunch v2_visual display-rviz.launch
```
Other arguments of  ```display-rviz.launch``` :
- `model`: path to the urdf/xacro file
- `rvizconfig`: path to the .rviz file

2.  To view the pool in Gazebo
```
roslaunch v2_visual pool-display.launch
```
Other arguments of  ```pool-display.launch``` :
- `show_misc`: true | false (default: true) <br>
 If true, displays other items like gate, flare, etc within the pool.
- `show_task_balls`: true | false (default: false)<br>
	If set `true`, the task balls for Task 4 are placed on the stands. It is set as `false` by default because, the balls are in continuous contact with the stand and it takes up a lot of processing power from Gazebo resulting in reduction of real time factor. On hiding the balls, the real time factor is 1.

	[Back to Home](./Home.md)