# Setting up darknet
To set-up darknet with the current auv_ws, you need to install openCV first. Follow the instructions given [here](https://www.learnopencv.com/install-opencv-4-on-ubuntu-18-04/) to install it. Please note before performing the step-3 (python libraries install), run the following command and then go ahead with the 3rd step.
```
pip install dlib -vvv
```
Also you may use bash files for each step in the above link for installation.
Once openCV is installed, next step is to clone the darknet repository for gate detection. Go to the src folder of your auv_ws and clone this [repo](https://github.com/SubZer0811/darknet_ros). Make sure to use `git clone --recursive` while cloning the repository. Once the repo is cloned, follow the steps given in the readme.md file to build the package. Once you build it, you are good to go.

Some useful information: `/darknet_ros/bounding_boxes` is the name of the topic which publishes the probability & gate coordinates (w.r.t. camera frame).

[Back to Home](./Home.md)