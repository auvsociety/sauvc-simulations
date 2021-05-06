# About auv_d
A basic underwater vehicle with 2-DoF, that can be manoeuvred using keyboard. It is neutrally buoyant and can only exhibit **surge** and **yaw** motion on the surface of the water. **You may start learning how to simulate underwater vehicles with his repository.**

## Resources referred and used:
* To simulate underwater conditions: [freefloating-gazebo][1].
* [Paper][2] released by author of *freefloating-gazebo*.
* To find out the *center of buoyancy*, the following links were referred:
	* [Link 1][3]
	* [Link 2][4]
* [T100 thrusters CAD][5] model from BlueRobotics.
* For keyboard operation, [this][6] code snippet was referred.
> Gazebo's own [hydrodynamics plugin][7] was not opted because the plugin uses the bounding box around mesh of collision model to calculate the buoyant force. The results were accurate only when simple shapes like *cuboid*, *sphere*, and *cylinder* were used.

## Commands
To view the model in *RViz*:
```
roslaunch auv_d display-rviz.launch
```
To control the bot on Gazebo:
```
roslaunch auv_d auv_d_gazebo.launch
```
```
rosrun auv_d auv_d_key_control
```
[Demo video](https://youtu.be/J9xrZcnUjJc) <br>
[Back to Home](./Home.md)


[1]:https://github.com/freefloating-gazebo/freefloating_gazebo
[2]:https://hal.inria.fr/hal-01065812v1/document
[3]:https://www.youtube.com/watch?v=Im8xpjPI3jA
[4]:https://www.youtube.com/watch?v=HRJY-0Ryw6I
[5]:https://www.google.com/search?client=ubuntu&channel=fs&q=t100+thrusters&ie=utf-8&oe=utf-8
[6]:https://stackoverflow.com/questions/7469139/what-is-the-equivalent-to-getch-getche-in-linux
[7]:http://gazebosim.org/tutorials?tut=hydrodynamics&cat=physics