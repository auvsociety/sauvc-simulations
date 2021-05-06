# common

All these source files are common to the roscpp nodes used in this project.

## About files
- `pid_rotate.cpp`: Implementation of PID controller for rotatory motion. Developed specifically to handle highly abrupt angle jumps when seen from a linear scale - eg: 180 to -179  degrees, 0 to 359 degrees, etc. Key parameters:
    - `kp_`: Proportional gain
	- `ki_`: Integral gain
	- `kd_`: Derivative gain
	- `snstvty`: Sensitivity | if the absolute value of the error is less than or equal to `snstvty`, then the error is forced to 0.
- `pid_translate.cpp`: Implementation of PID controller for translatory motion. Key parameters are same as that for `pid_rotate.cpp`.
- `terminal_getch.cpp`: Houses linux implementation of the `getch()` function. Used for CLI.
- `uwv.cpp`: Abstract class implementing major functions responsible for AUV control, PID controller, etc. Child class with `StoppableThread` as parent.

[Back to inter-package navigation](../../docs/v2_control.md)

[Back to Home](../../docs/Home.md)
