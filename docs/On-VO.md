# On VO
Visual Odometry (VO) methods have been classified into two types:
1. Feature-based
2. Direct type / dense methods
Recently, learning based VO solutions have also been developed. VO methods generally employ one of the following vision systems:

### Setero vision
- 3D information about environment computed in each time step using left and right image
- motion estimation by observing features in two successive frames
- the scale can also be determined _(scale: relationship between distance in the image and distance in real world)_

### Monocular vision
- 3D information extraction needs two time successive frames
- 3rd frame needed for motion estimation
- scale information is not known
- generally needs GPS or IMU for scale information

## Feature-based methods
Also known as sparse methods.
### General working mechanism
1. Identify subset of points that are locally distinctive and describe their local neighbourhood for each frame
2. Match these points between consecutive images based on their description
3. Remove outliers
4. Estimate motion between frames

### More details
- **features**: a region of image which is dissimilar to its immediate neighbourhood in terms of properties such as intensity, colour, and texture
  - important properties:
    - geometric invariance: translation, rotation, scale
    - photometric invariance: brightness, exposure
  - best features: corners, blobs
  - edges are avoided because of lack of invariance for most transformations
- **key-points**: points that are likely to be detected again in a similar image (with small variation in position, orientation, or illumination)
  - methods: Harris corner, Shi-Tomasi corner, FAST, SIFT
    - FAST: Features from Accelerated Segment Test: checks the brightness values of 16 pixels around a centre pixel and based on similarities between pixels and pixel centre, decides if the region is uniform, an edge or a corner. It can be used in real time application and it is sensitive to noise and depends on threshold.
    - SIFT: Scale Invariant Feature Transform: Detects blobs. 
- **feature descriptors**: used to describe locally distinctive points (called features) in an image
  - methods:
    - SIFT: Standard approach. Drawback: computation time
    - SURF: Speeded Up Robust Features. Blob detector and descriptor inspired by SIFT. Faster than SIFT but slightly better than SURF in terms of robustness
    - BRIEF: Binary Robust Independent Elementary Features. Uses binary strings instead of vectors of floating numbers (like SIFT) for descriptor. Binary strings are formed using Hamming distance. Drawbacks: Cannot deal with rotation and scale changes
    - ORB: Oriented FAST and Rotated BRIEF. Binary descriptor. Uses improved FAST detector, corrects disadvantages of BRIEF. ORB descriptor is BRIEF descriptor with scale and orientation information
    - BRISK: Binary Robust and Invariant Scalable Key points. Similar to ORB. Compromise between ORB's very fast sparse  pyramid technique and SIFT fine separated level but slow pyramid.
    - FREAK: Fast Retina Key point. For embedded systems like smartphones. No keypoint detector. Extracts and compares descriptors for given key points.
- **feature matching**: process of associating features in an image to features in another.
  - steps:
    1. define distance function to compare two descriptors
    2. for all features in image 1, find corresponding feature in image 2 with minimum distance
  - KLT: Kanade-Lucas-Tomasi Feature Tracker. Feature tracking algorithm. Uses intensity levels of raw image for matching instead of computing descriptors of extracted features.
- Removing outliers using RANSAC (Random Sample Consensus)
  1. Data points are sampled from matching step model
  2. New model paramters are generated from sampled points
  3. New model given a score based on number of inliers within a pre-set threshold value
  4. 1-3 steps repeated for certain number of trials and best computed model having highest number of inliers is achieved

## Dense methods
- Direct methods: _methods for motion estimation which recover the unknown parameters directly from measurable image quantities at each pixel in the image_ 
- motion estimated by minimizing photometric differences without knowing pixel correspondences
- rely on pixel intensity, extracting more information than feature based methods
- give more accurate estimation
- perform well in texture-less environments
- computationally more expensive
- changes in brightness is an issue
- summary of steps:
  1. Try an initial camera motion and find each pixel's projection in the next frame
  2. Compare the intensity of the projected pixel in the next frame with a pixel in the current frame
  3. Iteratively adjust camera's motion to lower photometric difference
- Dense methods reply on assumption: _brightness of a point projected on two consecutive images is constant or nearly constant_
  - leads to optical flow constraint or brightness constancy constraint
- displacement of each pixel between consecutive images depends on the optical flow components on both X and Y
  - leads to 2nd necessary constraint to determine displacement of a pixel: _motion model_
- Techniques exist that help in dealing with brightness changes
- List of dense methods:
  - PTAM: Parallel Tracking and Mapping. limited to small static spaces and requires sufficient textures in environment
  - DTAM: Dense Tracking and Mapping in Real-time. Sensitive to brightness changes, computationally heavy and need strong GPU for real time use
  - PLK: Pyramidal Lucas-Kanade
  - LSD-SLAM: Large Scale Direct Monocular SLAM. Authors claim the method to run on real-time CPU and can deal with changes in scale and rotation
  - Semi-Dense Visual Odometry
  - Fast Semi-Direct Monocular Visual Odometry (SVO)
  - DSO: Direct Sparse Odometry

## Learning based VO methods
Techniques:
- Feature Point Descriptors using CNNs: Use of CNN(Convolutional Neural Network) for image feature descriptor
- DeepVO
- LS-VO
- UnDeepVO
- Flowdometry

Source: [link](https://www.diva-portal.org/smash/get/diva2:1215266/FULLTEXT03)

[Back to Home](./Home.md)