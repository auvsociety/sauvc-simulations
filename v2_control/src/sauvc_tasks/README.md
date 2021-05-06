# sauvc_tasks

## To execute
Execute the commands in each line in separate terminals:
```
roslaunch v2_control sauvc.launch
roslaunch v2_control process_vision.launch
rosrun v2_control sauvc_tasks
```
Arguments of  **process_vision.launch** :
- `show_frnt_raw`: true | false (default: false)<br>
If _false_, front camera (uncalibrated) output window will not open.
- `show_dwn_raw`: true | false (default: false)<br>
If _false_, down camera (uncalibrated) output window will not open.
- `show_frnt_drknt`: true | false (default: true)<br>
If _false_, front camera output from darknet with labels will not be shown.

## About
- This node implements both the ROV control as well as autonomous mode.
- Uses the same YAML file used by `rov_ctrl` node for PID configuration located at `path/to/v2_control/config/pid_gains.yaml`.
- PID gains, if needed to be changed during runtime, they must be changed via `rosparam set` and the PID configuration must be refreshed via the appropriate command on the terminal running `sauvc_tasks` node.

### About files
1. `auv.h` file has all the libraries and global variable declaratiions.
2. `task_1.h` file has all the functions used for performing the task-1. All the functions here have been declared in namespace `task_1`.
3. `move.h` contains generic motion commands that are used by higher level logic described in `task_1.h` and `all_tasks.cpp`.
4. `all_tasks.cpp` uses the above to files and is tha main file. 

[Back to inter-package navigation](../../docs/v2_control.md)

[Back to Home](../../docs/Home.md)