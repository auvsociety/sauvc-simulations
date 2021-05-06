# On SLAM algorithms

## On [SVO][1]
### Brief overview
SVO stands for Semi-direct Visual Odometry. It is maintained by Robotics & Perception group of University of Zurich. The developers have actually implemented this on their runs and it works great! Check this [video](https://www.youtube.com/watch?v=2YnIMfw6bJY&feature=youtu.be) and going through the description is a must. The best thing about this package is that RPG of UZH have run the on on a [Ordroid-U3](https://www.hardkernel.com/shop/odroid-u3/) processor. If it can run on that obsolete processor, then I am sure it will run on Jetson Nano smoothly and concurrently with the control subsystem.
### Side-note
Use the forked repositories instead of original ones as minor changes have been made in the forked version so that they work properly on Melodic - [rpg_svo](https://github.com/meetm473/rpg_svo/tree/sauvc) and [rpg_vikit](https://github.com/meetm473/rpg_vikit/tree/sauvc). Note that the other packages mentioned in the Wiki of the original [repo][1] must be implemented as it has been mentioned.

The following are the results and the inferences obtained after trying to implement SVO along with AUV_v2 simulation:
- SVO is designed to work with down facing cameras. The number of feature points/corners in the visual range of the camera must be atleast 100. Since the ground plane of the pool is ultra neat, the number of feature points detected are very less. Hence, SVO cannot be used.
- Experiment on whether if front camera could give us more feature points/corners was performed. But, the number of corners and feature points, in 100% visibility with the task 1 gate, obstacle and task 2 tubs visible, is low as well (less than 100).
- SVO was tested independently. It works properly. So no issues from SVO's side.
- Most of the monocular visual odometry and SLAM algorithms that have been encountered _(until the date of drafting of this wiki page)_, rely on the detection of feature points/corners. But, the monocular SLAM and visual odometry methods that were being referred to since the beginning, were implemented for quadcopters and terrestrial bots mapping indoor environments. Indoors environments are in general "heavily textured" as in one can find a lot of feature points/corners. In underwater scenario, since the visibility is not 100% and the environment is mostly empty (in terms of feature points), we CANNOT use visual SLAM algorithms developed for terrestrial bots and quadcopters.

Additional thoughts:
- I have never heard aeroplanes or submarines relying on visual odometry (and for that matter visual SLAM). Now I think I understand the reason behind this. You DONT have a textured environment high up and deep below. This could be one of the reasons why underwater vehicles rely more on sound than vision (other obvious reason being low illumination underwater).
- Now I understand why LIDARs are also preferred for underwater exploration and not simple cameras with headlights.
- The next step is to look for special underwater SLAM systems or visual odometry algorithms that can work using low number of feature points (at max 30 or less. SVO needed atleast 100 in the first frame).
- This was listed under algorithm of practical implementation use in this [thesis][3].

Few solutions to the problem presented:
- Current SVO package uses FAST. We could use ORB instead of FAST to get more descriptors/features. I really don't know if this would work, but going through this method means tweaking aspects of the SVO package that will take a large amount of time and energy.
- More tiles in the simulation floor to make it more textured, but I really don't know if we will be able to find enough number feature points in actual pool.
- Add random dots to the image before it goes into SVO package for processing. This would increase number of feature points. Idk what will happen. Just a crazy idea.

[1]:https://github.com/uzh-rpg/rpg_svo

## On [ORB SLAM](https://github.com/raulmur/ORB_SLAM)
### Brief Overview
- ORB SLAM is a visual SLAM approach
- Derives its name from the ORB feature descriptor
- > ORB: "An efficient alternative to Scale Invariant Feature Transform (SIFT) and Speeded-Up Robust Features (SURF)”
- According to Mur-Artal et al. these features provide “good invariance to changes in viewpoint and illumination” and are cheap to compute
- Feature-based front-end and the back-end works on a keyframe-based graph-optimization procedure
Refer to [PDF][2] page numbers 13-16 for an overview of how ORB-SLAM works.
- This was considered as an algorithm for practical use in [this][3] thesis along with SIFT in the feature based methods category.

[3]:https://www.diva-portal.org/smash/get/diva2:1215266/FULLTEXT03

### Conclusions from [here][2]
The thesis that has been referred has implemented and tested ORB SLAM on Blue ROV2 vehicle in different scenarios. The ROV is tethered and computation is done on a laptop. The final results of the thesis are presented here:
- ORB SLAM cannot handle highly repetitive or symmetric areas.
- For some of the datasets the robot moved on a trajectory which did not revisit the same places. In these cases a severe problem with ORB SLAM is that when tracking fails the algorithm will forever try to relocalize.
- Key points from experimenting in the pool:
  - The pool was in closed environment (sheltered pool).
  - Since it was suspected that ORB SLAM might have trouble working in this very symmetric and feature scarce environment one experiment was done with 20 unique printed markers placed on the pool’s floor. In order to give the algorithm enough chances for loop closures the ROV was driven along the pool’s walls in both datasets whilst always staying on the surface.
  - **The pool proved to be a very difficult environment for ORB SLAM to work in**.
  - Tracking of black lines in the pool is most likely to be lost because of the poor feature distribution.
- When the experiment was performed at place, out in the open:
  - ORB SLAM struggles when faced with the lighting conditions on a sunny day.
  - Due to the ripples, ORB SLAM cannot initialize and has no chance of tracking the robot’s movement. In fact for dataset 3 it was **only able to initialize because at the beginning of the recording there is a cloud in front of the sun, changing the lighting conditions for a short time**. As soon as the cloud is gone and the ripples are seen again ORB SLAM looses tracking and can neither relocalize nor reinitialize when the algorithm is reset.
  - An interesting observation that can be made from the results on these datasets is that ORB
SLAM detected very little loop closures. This does not mean that the algorithm fails at detecting possible loop closures but rather that it is able to recognize already mapped places in an monotonous environment.
- As is evident ORB SLAM is not able to extract enough features in low texture area.

### Suggestions provided
- For overcoming monotony: A very interesting idea for overcoming this would be to mix feature-based methods with dense approaches. Even when there are little point features visible, dense approaches should still be able to to track movement based on visible gradients.

[2]:https://robotics.ee.uwa.edu.au/theses/2017-UnderwaterSLAM-Kahlefendt.pdf

## On [DSO](https://github.com/JakobEngel/dso)
### Brief Overview
DSO stands for Direct Sparse Odometry. DSO does not depend on keypoint detectors or descriptors, thus it can naturally sample pixels from across all image regions that have intensity gradient, including edges or smooth intensity variations on mostly white walls. It is developed by the Computer Vision group at Technical University of Munich (TUM).
- This was listed under algorithm of practical implementation use in this [thesis][3].
- Refer to [this](./Using-DSO.md) page to learn more about using DSO with AUV_v2.
- Further discussion is continued at [this](./Using-DSO.md) page.
- We could use the work presented in [this](https://arxiv.org/pdf/1804.05625.pdf) paper to improve DSO. Only drawback is that code is not made open source.

## On [REBVO](https://github.com/JuanTarrio/rebvo)
### Brief Overview
REBVO stands for Realtime Edge Based Inertial Visual Odometry for a Monocular Camera. It tracks edges instead of features or pixels. Its focus is to run on embedded devices; depth is estimated via EKF; it has extension for IMU integration. It is targeted for MAV operation.
> Not yet tested it in simulation

## On [LSD-SLAM](https://github.com/tum-vision/lsd_slam)
### Brief Overview
Large Scale Direct monocular SLAM. Developed by TUM before developing DSO. Instead of using keypoints, it directly operates on image intensities both for tracking and mapping. LSD-SLAM runs in real-time on a CPU, and even on a modern smartphone.
- This was REJECTED under algorithm of practical implementation use in this [thesis][3]. This was done because of the low FPS rate of the AUV’s camera. Available recorded images from the camera run at about 3 FPS rate; however, according to KTH’s robotics teacher Patric Jensfelt, LSD-SLAM cannot perform accurately with such low data FPS rate.

## On [Shi-Tomasi+PLK based VO](https://ieeexplore.ieee.org/document/7563010)
This method runs on embedded system and the authors claim that their method gave good results for computing UW VO using camera facing ocean bottom.
> Not yet tested it in simulation.

[Back to Home](./Home.md)