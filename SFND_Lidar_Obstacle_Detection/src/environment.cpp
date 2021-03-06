


#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"


// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}



void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,pcl::visualization::PCLVisualizer::Ptr& streamer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)

{
  // ----------------------------------------------------
  // -----Lidar Object Detection Pipeline    -----
  // ----------------------------------------------------


  //  Rendering Input Lidar Stream

  renderPointCloud(streamer,inputCloud,"input pcd stream");

  // Downsampling and filtering input point cloud.
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud= pointProcessorI->FilterCloud(inputCloud,0.2, Eigen::Vector4f (-10,-5.5,-2, 1), Eigen::Vector4f ( 30, 7,0.2, 1));

  std::cout<<"filtered cloud has "<< filterCloud->points.size()<<" points"<<endl;
  
  // Segmenting point cloud into obstacle and Road points.

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = pointProcessorI->SegmentPlane(filterCloud,100,0.2);
 
  renderPointCloud(viewer,segment_cloud.second,"RoadCloud",Color(0,1,0));

  // Clustering obsctacle points into individual objects.

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(segment_cloud.first,0.45,15);
    
    
  // Rendering detected object Clusters and their bounding boxes ;

    int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  	for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
  	{
  	   
  		renderPointCloud(viewer,cluster,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
        Box box = pointProcessorI->BoundingBox(cluster);
		renderBox(viewer,box,clusterId);
      
        
   		++clusterId;
  	}


}
    


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer, pcl::visualization::PCLVisualizer::Ptr& streamer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    streamer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    streamer->initCameraParameters();
    
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); streamer ->setCameraPosition(-distance, -distance, distance, 1, 1, 0);break;
                    
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); streamer->setCameraPosition(0, 0, distance, 1, 0, 1);break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); streamer->setCameraPosition(0, -distance, 0, 0, 0, 1);break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);streamer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
       { viewer->addCoordinateSystem (1.0);
         streamer->addCoordinateSystem (1.0);
       }

}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Object Detections"));
    pcl::visualization::PCLVisualizer::Ptr streamer (new pcl::visualization::PCLVisualizer ("Input Lidar stream"));
    CameraAngle setAngle =  XY;
    initCamera(setAngle, viewer,streamer);



    //Setup Point Processor and Pcd file stream
   
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
   

    while (! (viewer->wasStopped() | streamer->wasStopped()))
    {   
        // Clear viewers
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        streamer->removeAllPointClouds();
        

         // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer,streamer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
        streamIterator = stream.begin();
        viewer->spinOnce();
        streamer->spinOnce();
    } 
}
