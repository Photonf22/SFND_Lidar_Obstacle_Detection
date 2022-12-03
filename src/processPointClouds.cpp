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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, 
                                                                                                                               Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object and perform voxel grid filtering then perform region of interest 
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);
    // Region of interest algorithm processor
    typename pcl::PointCloud<PointT>::Ptr cloudRegionROI (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> ROI_processor(true); // set to true because dealing with points inside the crop box
    ROI_processor.setMin(minPoint);
    ROI_processor.setMax(maxPoint);
    ROI_processor.setInputCloud(cloudFiltered);
    ROI_processor.filter(*cloudRegionROI);
    // The below algorithm is when you would like to remove the top roof of your vehicle to remove it from the filtered point cloud/ Optional
    // filter out indices wanted that represent the roof of the car to then be extracted this is similar to what we did with segmentation
    // similar steps below is the actual location of the roof in each .pcd image since the roof is static it will
    // always have the same location in every pcd
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegionROI);
    roof.filter(indices);
    // parsing through each inlier and getting their indices to pass to our vector
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);
    // creating the filtered cloud w/o a roof in which we will pass our new filtered w/o roof cloud
    typename pcl::PointCloud<PointT>::Ptr Filtered_CloudNoRoof (new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegionROI);
    extract.setIndices (inliers);
    extract.setNegative(true); // means we will be removing these roof points
    extract.filter(*Filtered_CloudNoRoof);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    return Filtered_CloudNoRoof;
}

// TODO Lidar Segmentation 
// Creating the point cloud for the road and another one for the segmented plane
// the body of this function will include a timer to see how fast this function is
// if the function is not fast then it will not be helpful to run point clouds in real time
// This function will separate the obstacle cloud and plane cloud 
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

        // Create the filtering object
    typename pcl::ExtractIndices<PointT> extract;
    // Creating segmentation object and filling in the data to tell it that we want to use RANSAC filtering
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new typename pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new typename pcl::PointCloud<PointT>()); 
    // Since now we have all the inliers/indices from the actual cloud now then we can create the planeCloud or road cloud
    // taking all the inliers and pushing it to the planeCloud
    for (int index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*obstacleCloud);
    // making a pair structure to return the plane and obstacle cloud as a function return
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud,planeCloud);
    return segResult;
}

// Segmenting the plane with PCL 
// Part 4 of Segmentation part of the lidar class
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process this will tell us how long this algorithm took to complete
    // If it’s taking a really long time to process the segmentation, 
    //then the function is not going to be useful running in real-time on a self-driving car. line 68
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ());

    typename pcl::SACSegmentation<PointT> seg;
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // calling function SeparateClouds() separates the point cloud to the non-plane points(obstacles) and plane points(road)
    // The inliers can be added to the plane cloud by looping over the inlier indices and pushing the corresponding inlier point into
    // the plane cloud's point vector
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
   
    return segResult;
}
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    // pcl::PointCloud<PointT>::Ptr cloud_filtered;
    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    // creating a kd tree for euclideancluster
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    int j = 0;
    for (const auto &cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for(const auto& inx : cluster.indices)
        {
            cloud_cluster->push_back(cloud->points[inx]);
        }
        cloud_cluster->width = clusters.size();
        cloud_cluster->height= 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


// My own Code Starts Here
// ******************************************************************************************************************************************************
// Ransac and Clustering Algorithm implementations from Class by Alejandro Sanchez for Processing point cloud of car
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::RansacImplementation(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	
	srand(time(NULL));
	float A = 0;
	float B = 0;
	float C = 0;
	float D = 0;
	int inliers = 0;
	float temp_distance = 0;
	int max_distance = 0;
	int q = 0;
	int point2 =0;
	int point1 =0;
	int point3 = 0;
	// TODO: Fill in this function
	while(q < maxIterations)
	{
		std::unordered_set<int> temp;
	 	point1 = rand() & cloud->points.size();
		while(1)
		{
			point2 = rand() & cloud->points.size();
			if(point2 != point1)
			{
				break;
			}
		}
		while(1)
		{
			point3 = rand() & cloud->points.size();
			if(point3 != point1 && point3 != point2)
			{
				break;
			}
		}
		float i = (cloud->points[point2].y - cloud->points[point1].y)*(cloud->points[point3].z - cloud->points[point1].z) -
				(cloud->points[point2].z - cloud->points[point1].z)*(cloud->points[point3].y - cloud->points[point1].y);
		
		float j = (cloud->points[point2].z - cloud->points[point1].z)*(cloud->points[point3].x - cloud->points[point1].x) -
				(cloud->points[point2].x - cloud->points[point1].x)*(cloud->points[point3].z - cloud->points[point1].z);
		
		float k = (cloud->points[point2].x - cloud->points[point1].x)*(cloud->points[point3].y - cloud->points[point1].y) -
				(cloud->points[point2].y - cloud->points[point1].y)*(cloud->points[point3].x - cloud->points[point1].x);

		A = i;
		B = j ;
		C = k;
		D= (i*cloud->points[point1].x + j*cloud->points[point1].y + k*cloud->points[point1].z);
		//iterating through every point and getting the distance of the point to the line if below threshold then its an inlier
		
		for(int p = 0;p < cloud->points.size();  p++)
		{
			temp_distance = fabs(A*cloud->points[p].x + B*cloud->points[p].y + C*cloud->points[p].z - D)/sqrt(A*A + B*B + C*C);
			if(temp_distance <= distanceTol)
			{
				temp.insert(p);
			}
		}
		if(inliersResult.size() < temp.size())
		{
			inliersResult = temp;
		}
		q++;
	}

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		auto point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = std::make_pair(cloudOutliers,cloudInliers);
    // Return indicies of inliers from fitted line with most inliers
	//auto endTime = std::chrono::steady_clock::now();
	//auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	//std::cout << "Ransac took" << elapsedTime.count() << "milliseconds" <<  std::endl;
    return segResult;
}

template <typename PointT>
void ProcessPointClouds<PointT>::proximity(KdTree **tree,std::unordered_map<int,int> &temp_map, std::vector<std::vector<float>> points,int index, std::vector<int> &cluster, float distanceTol)
{
    // 
	temp_map[index] = 1;
	cluster.push_back(index);
	std::vector<int> nearby;

	nearby = (*tree)->search(points[index], distanceTol);
	for(int &i : nearby)
	{
		//if( temp_map.find(i) == temp_map.end())
        if(temp_map[i] != 1)
		{
			proximity(tree, temp_map,points,i,cluster, distanceTol);
		}
	}
}
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;

	std::unordered_map<int, int> processed;
	for(int i = 0; i < points.size() - 1; i++)
	{
		//if(processed.find(i) == processed.end())
        // not processed
        if(processed[i] != 1)
		{
			std::vector<int> cluster;
			proximity(&tree, processed,points,i,cluster, distanceTol);
			clusters.push_back(cluster);
		}
	}
	return clusters;
}                        

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringOwn_Implementation(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterClouds;
    std::vector<std::vector<float>> vecs;
    KdTree* tree = new KdTree;

    for (int i=0; i < (cloud->points).size(); i++) 
    {
        std::vector<float> temp{(cloud->points)[i].x, (cloud->points)[i].y};
    	tree->insert(temp,i); 
        vecs.push_back(temp); 
    }
  	std::vector<std::vector<int>> clusters = euclideanCluster(vecs, tree, clusterTolerance);

  	for(std::vector<int> cluster : clusters)
  	{
        if(cluster.size() >= minSize && cluster.size() <= maxSize)
        {
  		    typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>);
  		    for(int indice: cluster)
  			    clusterCloud->points.push_back(cloud->points[indice]);
            clusterCloud->width = clusterClouds.size();
            clusterCloud->height = 1;
            clusterCloud->is_dense = true;
            clusterClouds.push_back(clusterCloud);
        }
  	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusterClouds;
} 

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringOwn_ImplementationNoMap(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterClouds;
    std::list<std::vector<float>> vecsList;
    std::vector<std::vector<float>> vecs;
    KdTree* tree = new KdTree;

    for (int i=0; i < (cloud->points).size(); i++) 
    {
        std::vector<float> temp{(cloud->points)[i].x, (cloud->points)[i].y}; 
        vecs.push_back(temp); 
        vecsList.insert(temp); 
    }
    tree->insert(temp,i);
    fill_kdtree(*tree, vecs);
    
  	std::vector<std::vector<int>> clusters = euclideanCluster(vecs, tree, clusterTolerance);

  	for(std::vector<int> cluster : clusters)
  	{
        if(cluster.size() >= minSize && cluster.size() <= maxSize)
        {
  		    typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>);
  		    for(int indice: cluster)
  			    clusterCloud->points.push_back(cloud->points[indice]);
            clusterCloud->width = clusterClouds.size();
            clusterCloud->height = 1;
            clusterCloud->is_dense = true;
            clusterClouds.push_back(clusterCloud);
        }
  	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusterClouds;
}     
// ******************************************************************************************************************************************************
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