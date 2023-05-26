# Image Based Visual Servoing

The aim is to implement an image-based visual servoing framework that uses the point features and servos the RRBot from one image configuration to the other in real-time.

## Problem Statement

The reference pose from the reference image is already known to the robot. Thus, the goal is to design a robust IBVS that reaches the reference pose starting from any pose in its reachability.

The object:

![object](https://github.com/kt-krutarthtrivedi/Image-Based-Visual-Servoing/blob/main/media/Object.png)

The reference image:
![reference](https://github.com/kt-krutarthtrivedi/Image-Based-Visual-Servoing/blob/main/media/Reference%20View.png)


## Results

Given the reference already known to the robot, let's start the robot from any random joint configuration as shown below:

![current](https://github.com/kt-krutarthtrivedi/Image-Based-Visual-Servoing/blob/main/media/Current%20View.png)

The robot reaches to the reference pose calculating the required joint angle using IBVS controller. The feature tracking can be visualized as shown below:

![trajectory](https://github.com/kt-krutarthtrivedi/Image-Based-Visual-Servoing/blob/main/media/Tracking%20Trajectory.png)

## Working video

https://github.com/kt-krutarthtrivedi/Image-Based-Visual-Servoing/assets/134632027/afda316f-8840-4de0-8791-41ed448f8163

