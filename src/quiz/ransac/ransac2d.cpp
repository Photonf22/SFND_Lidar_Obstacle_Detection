/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "/lidar/SFND_Lidar_Obstacle_Detection/src/render/render.h"
#include <unordered_set>
#include "/lidar/SFND_Lidar_Obstacle_Detection/src/processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "/lidar/SFND_Lidar_Obstacle_Detection/src/processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

// Notes:
/* This function segments out the outliers from the inliers and this way we can detect what plane is the road
and what plane is the obstacles on the road and this way we can get the indeces of the obstacles or outliers
Then we can also separate our point clouds from the road to the obstacles
*/

// Quiz: Make a RANSAC equation for 2D data and calculate the best line to get the most amount of inliers and figure out
// what points are the outliers
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	//auto endTime = std::chrono::steady_clock::now();
	//auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	//std::cout << "Ransac took" << elapsedTime.count() << "milliseconds" <<  std::endl;
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
