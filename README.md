# Lidar Obstacle Detection

Project for Udacity's Sensor Fusion Engineer Nanodegree Program

**Project Goals**

* Implement Obstacle detection pipeline on PCD from a lidar sensor
* use [pcl-library](http://pointclouds.org/) for general data handling and initial testing
* implement following steps:
  * PCD filtering, for reducing computational cost, without loss of detail
  * Segment the filtered cloud into two parts, road plane and obstacles, using RANSAC based 3D-plane extraction
  * Cluster the obstacle cloud, using K-D Tree for 3D space and Eucledian clustering
  * Find bounding boxes for the clusters


### Dependencies:

The configuration I used

* Ubuntu 16.04 OS
* cmake >= 3.14
* gcc/g++ >= 8.0
* PCL >= 1.2 : The code extensively utilizes the [Point Cloud Library (PCL)](http://pointclouds.org/).





### Notes on some files & folders

* README.md: this file.
* **./src/**
  * **environment.cpp** - main function , city Block function contains the Lidar obstacle detection pipeline.
  * **kdtree.h** - KD Tree structure definition for Eucledian clustering of Point cloud.
  * **processPointClouds.cpp** & **processPointClouds.h** - functions for point-cloud processing.
  * **/render/...** - contains functions for rendering point clouds and bounding boxes 
  * **/sensors/..** - contains point-cloud-data files and functions for use with synthetic data.

### Build and Run

clone this repository, enter the cloned directory/folder and build:

```
cd SFND_Lidar_Obstacle_Detection
mkdir build && cd build
cmake ..
make
```

to run, use following from within the build folder:

```
./environment
```


### Lidar-Obstacle-Detection

once `./environment` is launched pcd data is read from files at `/sensors/data/pcd/data_1/` and  point cloud is processed. Input Lidar Stream And Object Detections are rendered in viewers as shown below:

Input Lidar Stream

![input cloud](https://user-images.githubusercontent.com/56697957/99141689-2382a080-264e-11eb-8e21-bcf471af9568.gif)


Bounding Box Object Detections

![object detections](https://user-images.githubusercontent.com/56697957/99141697-31d0bc80-264e-11eb-906a-4e9a854147e1.gif)




#### Resources

* To install PCL, C++ https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/
