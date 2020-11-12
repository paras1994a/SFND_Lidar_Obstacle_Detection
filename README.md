# Lidar Obstacle Detection

Project for Udacity's Sensor Fusion Engineer Nanodegree Program

**Project Goals**

* Implement Obstacle detection on PCD from a lidar sensor
* use [pcl-library](http://pointclouds.org/) for general data handling and initial testing
* implement following steps:
  * PCD filtering, for reducing computational cost, without loss of detail
  * Segment the filtered cloud into two parts, road plane and obstacles, using RANSAC based 3D-plane extraction
  * Cluster the obstacle cloud, using K-D Tree for 3D space.
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
mkdir build && cd build
cmake ..
make
```

to run, use following from within the build folder:

```
./environment
```


### Lidar-Obstacle-Detection

once `./environment` is launched pcd data is read from files at `/sensors/data/pcd/data_1/` and  point cloud is processed and rendered . Detected objects are enclosed in bounding boxes , as shown below.



#### Resources

* To install PCL, C++ https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/
