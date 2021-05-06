## An Overview
`simulations` repository is a compilation of ROS packages that were, and are being developed to simulate the AUV-v2 vehicle built the AUV Society during 2019-2020 for SAUVC 2020 _(that later got postponed to 2021)_. The repository contains packages that were developed to merely test one aspect of AUV simulation and packages that completely simulate the various tasks of SAUVC. This Wiki aims to provide a one stop guide to the members of the AUV Society to better understand how to use the existing code and how to extend it forward. You can find the playlist with demonstrations wrt this repository [here](https://youtube.com/playlist?list=PLhvbtt31sTvz2M71oOWf9Pjefrbu2GmDH).

---
## Navigate through the documentation
### About packages
1. [auv_simulation](./auv_simulation.md)
2. [auv_d](./auv_d.md)
3. [v2_control](./v2_control.md)
    1. [include](../v2_control/include/README.md)
    2. [common](../v2_control/src/common/README.md)
    3. [rov_ctrl](../v2_control/src/rov_ctrl/README.md)
    4. [sauvc_tasks](../v2_control/src/sauvc_tasks/README.md)
4. [v2_dataset](./v2_dataset.md)
5. [v2_visual](./v2_visual.md)
6. [v2_utility](./v2_utility.md)
7. [Setting up darknet](./Setting-up-darknet.md)

### Identifying SLAM
1. [On VO](./On-VO.md)
2. [On SLAM algorithms](./On-SLAM-algorithms.md)
    1. [Using DSO](./Using-DSO.md)
    1. [Notes on understanding DSO](./Notes-on-understanding-DSO.md)
    2. [DSO Test 1](./DSO-Test-1.md)
    3. [DSO Test 2](./DSO-Test-2.md)
4. [Underwater SLAM scenario](./Underwater-SLAM-Scenario.md)

### Other Pages
1. [On thruster configuration](./On-thruster-configuration.md)
2. [References](./References.md)

---

> **Guidelines on page addition**
> - Add this link: `[Back to Home](./Home.md)` at the end of every new page that is added to this `/docs` folder.
> - Add link to the newly added document in the above navigation list.
> - Add a title in heading 1 format for the new page added.

## The packages
The following ROS packages are present as a part of this repository:
1. [auv_simulation](./auv_simulation.md):
First simulation of the AUV-v2 vehicle. It just shows water current, and the vehicle with buoyant forces acting on it. Not much of practical use.
2. [auv_d](./auv_d.md):
A basic underwater vehicle with 2-DoF, that can be manoeuvred using keyboard. Start learning how to simulate underwater vehicles with his repository.
3. [v2_control](./v2_control.md):
Contains files that control the auv_v2 vehicle inside the SAUVC-2020 swimming pool.
4. [v2_dataset](./v2_dataset.md):
Contains image dataset from the AUV_v2 vehicle's front and down facing cameras.
5. [v2_visual](./v2_visual.md):
Contains all the files that help in visualising the SAUVC-2020 swimming pool, and auv_v2 vehicle.
6. [v2_utility](./v2_utility.md):
Contains utility ROS nodes, plugins, and other files required by v2_control.

## Contributor's basic task
Once you make your contributions, please make sure:
- You have provided sufficient information in the README or in Wiki about how the contributions should be used by a person new to this repository.
- You add meaningful comments to the commits you make.
- Requested to follow the official roscpp/rospy [guidelines](http://wiki.ros.org/CppStyleGuide) for the code.

## A suggestion
For a better workflow and management of various repositories of the AUV Society, create a new catkin workspace (something like `auv_ws`) and clone various repositories related into it. Keep your personal catkin workspace and AUV workspace separate.

> **Note**: The packages have been developed and tested on Ubuntu 18.04 LTS and ROS Melodic. 

## Additional pages
These have been created to summarise the theory behind the HLS algorithms, how to use them, and why certain algorithms were not considered.
1. [On VO](./On-VO.md): Notes on basic terms and methods to estimate odometry from visual sensor (camera). 
2. [On SLAM algorithms](./On-SLAM-algorithms.md): Notes that focus on pros, cons, and the reasons behind choosing or not choosing a potentially useful algorithm for AUV_v2.
3. [Underwater SLAM scenario](./Underwater-SLAM-Scenario.md): Challenges to implement an underwater SLAM system.
4. [Using DSO](./Using-DSO.md): Notes, guidelines, and plans for using the DSO algorithm for AUV_v2.
5. [On thruster configuration](./On-thruster-configuration.md): Description of the vectored thruster fusion technique and other convention followed for the vehicle