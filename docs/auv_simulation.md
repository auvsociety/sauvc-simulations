# About auv_simulation
This was very first simulation of the AUV-v2 vehicle. It just shows water current, and the vehicle with buoyant forces acting on it.

## Resources required to *make* the package:
1. [freefloatingbuoyancy][1]
2. [UUV_Gazebo_World][2]

## Commands
To simply view the vehicle in RViz, use the following command

```
roslaunch auv_simulation display-rviz.launch
```
To view it in Gazebo:
```
roslaunch auv_simulation gazebo.launch
```

## Side notes:
1. The `my_world.world` is an empty world with just the clouds and no water.
2. For the `uuv_simulator`, better `git clone` the uuv_simulator repository and `catkin_make` rather than installing it
via `sudo apt install`.
3. It's totally fine if you do not go through this package.

[Back to Home](./Home.md)

[1]:https://github.com/bluerobotics/freebuoyancy_gazebo
[2]:https://github.com/uuvsimulator/uuv_simulator