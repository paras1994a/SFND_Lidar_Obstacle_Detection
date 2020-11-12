// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
// voxel grid point reduction and region based filtering with pcl cropbox

    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>());

    // Downsampling point cloud resolution

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter(*filteredCloud);

    //get only region of interest (environment in immediate proximity of EGO vehicle , we dont care about far off obstacles)

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filteredCloud);
    region.filter(*cloudRegion);

    //eliminate car's roof points
   
    std::vector<int> roof_indices;
    pcl::CropBox<PointT> roof(true);

    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.3,1.7,0,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(roof_indices);


    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    for(int point : roof_indices)
      inliers->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << "milliseconds" << std::endl;
    
    //
    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // create two clouds,one with obstacles and other with segmented plane
  
    typename pcl::PointCloud<PointT>::Ptr obsCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr RoadCloud (new pcl::PointCloud<PointT>());
    
    // Seperating input cloud using inlier indices.

    for(int index= 0; index< cloud->points.size(); index++)

    {     
        
		PointT point = cloud->points[index];
		if(inliers.count(index))
			RoadCloud->points.push_back(point);
		else
			obsCloud->points.push_back(point);
    }
   
   
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obsCloud, RoadCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{  
   // obtain ground plane inliers using Ransac algorithm.
  
   std::unordered_set<int> inliers = ransac3D(cloud,maxIterations,distanceThreshold);

   // returning seperated point clouds for Road and Obstacle points.

   std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
   return segResult;
}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize )
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creates Kdtree and inserts obstacle points into tree structure.
   
    KdTree* tree = new KdTree;
  
    std::vector<std::vector<float>> points;

	for(int i=0 ;i< cloud->points.size();i++)
	{
		points.push_back({cloud->points[i].x,cloud->points[i].y,cloud->points[i].z});
			
	}

    for (int i=0; i<points.size(); i++) 
    	
    {    
        tree->insert(points[i],i);

    }


    //Clustering the obstacle points into individual objects 
    
    std::vector<std::vector<int>> clusters_indices = euclideanCluster(points,tree,clusterTolerance,minSize);

   // Iterate over the resulting cluster indices to create cluster clouds.
  	
      for(std::vector<int> cluster : clusters_indices)
  	{
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
  		
        for(int indice: cluster)
  			
        { PointT point = cloud->points[indice];
        
          clusterCloud->points.push_back(point);

        }

		
        clusters.push_back(clusterCloud);
		
		
    }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
   
    // returns detected obstacle clusters.
    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	//  placeholder set for resulting inlier indices. 
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	
    // running Ransac to fit ground plane in inputcloud for given  maxIterations 

    while(maxIterations--)
	{
      // inlier indices for current iteration 
	  std::unordered_set<int>inliers;

      // randomly Sample 3 points from input cloud 

	  while(inliers.size()<3)
	  { inliers.insert(rand() % cloud->points.size());
	  
	  }
      
	  float x1,y1,z1,x2,y2,z2,x3,y3,z3;

	  auto itr = inliers.begin();

	  x1 = cloud->points[*itr].x;
	  y1 = cloud->points[*itr].y;
	  z1 = cloud->points[*itr].z;

	  itr++;

	  x2 = cloud->points[*itr].x;
	  y2 = cloud->points[*itr].y;
	  z2 = cloud->points[*itr].z;

	  itr++;

	  x3 = cloud->points[*itr].x;
	  y3 = cloud->points[*itr].y;
	  z3 = cloud->points[*itr].z;

	  // Fiiting plane on sampled points

	  float i = ((y2-y1)*(z3-z1)-(z2-z1)*(y3-y1));
	  float j = ((z2-z1)*(x3-x1)-(x2-x1)*(z3-z1));
	  float k = ((x2-x1)*(y3-y1)-(y2-y1)*(x3-x1));
	

	  float a = i;
	  float b = j;
	  float c = k;
	  float d = -(i*x1+j*y1+k*z1);

      // obtain inliers for fitted ground plane.

	  for(int index=0;index < cloud->points.size();index++)
	  { 
		  if(inliers.count(index))
		  {
			  continue;
			
		  }
		
		float x4,y4,z4;

		PointT point = cloud->points[index];
		x4 = point.x;
		y4 = point.y;
		z4 = point.z;

		float dist = fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c) ;

		if(dist<= distanceTol)
		{
			inliers.insert(index);
		}

	  }

      if(inliers.size()>inliersResult.size())
	  {inliersResult= inliers;
	  }

	}

	
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
	
    std::cout<<"Total inliers="<<inliersResult.size()<<std::endl;
    
	// Return indicies of inliers from fitted plane with most inliers

    return inliersResult;



}

void proximity(std::vector<std::vector<float>>points,int index ,std::vector<int> &cluster,KdTree *tree,std::vector<bool> &processed,float distanceTol)

{ processed[index] = true;
  cluster.push_back(index);

  std::vector<int> nearby = tree->search(points[index],distanceTol);
  
  for(int id :nearby)
  { if(!processed[id])
  	{
		  proximity(points,id,cluster,tree,processed,distanceTol);

  	}

  }
  
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol,int min_size)
{


	std::vector<std::vector<int>> clusters;

    // Flags to keep track of processeed points.
	std::vector<bool> processed(points.size(),false);

    
	for (int index=0; index<points.size(); index++)
	{ 
	   if(!processed[index])
	   {  
		  // search recursively for nearest neighbours/proximity points in tree 
          // an entire object cluster is captured when search is exhausted.
		  std::vector<int> cluster;
		  proximity (points,index,cluster,tree,processed,distanceTol);
	
		  // filter noise/unreal clusters based on minsize.
		  if(cluster.size()>min_size)
		  {  
		   clusters.push_back(cluster);

          }
		 
		}

		
	}

	// return obtained cluster indices.
	return clusters;

}



