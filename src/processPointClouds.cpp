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