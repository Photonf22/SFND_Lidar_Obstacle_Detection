/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    // So here if we turn this to false then it will take away the cars on the scene and only show our car
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars,0); 
    bool render_obst =false;
    bool render_plane =false;
    bool render_clusters =true;
    bool render_box =true;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud= lidar->scan();
    // renderRays will basically render the rays on top of the car and show the lidar unit and we can also increase the amount of laser 
    // points or lasers in the lidar. One can increase the lidars and get a better resolution or more points
    /* This function renders the lidar rays or laser rays in the viewer */
    //renderRays(viewer,lidar->position, inputCloud);
    /* Rendering the point cloud*/
    //Color colors = Color(255,102,0);
    renderPointCloud(viewer, inputCloud,"inputCloud");

    // Here starts the "segmentation" part of the Lesson!
    // used lidar and created point clouds above and visualized them!

    // TODO:: Create point cloud processor
    ProcessPointClouds<pcl::PointXYZ> ProcessorpointCloud;
    // The segmentation algorithm fits a plan to the points and uses the distance tolerance to decide which points belong
    // to that plane. A large tolerance includes more points in the plane
   
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = ProcessorpointCloud.SegmentPlane(inputCloud, 100, 0.2);
     if(render_obst)
    {
        
        renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    }
    if(render_plane)
    {
         
        renderPointCloud(viewer,segmentCloud.second,"obstCloud",Color(1,0,0));
    }
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = ProcessorpointCloud.Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,1,0)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if(render_clusters)
        {
            std::cout << "cluster size ";
            ProcessorpointCloud.numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        }
        if(render_box)
        {
            Box box = ProcessorpointCloud.BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
        }
      ++clusterId;
    }
    // renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    // renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  bool render_obst = true;
  bool render_plane = true;
  bool render_clusters = true;
  bool render_box = true;
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  // dealing with point clouds with intensity values
  // If use pointer then we initialize in the heap such as below
  // Experiment with the ? values and find what works best
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2f , Eigen::Vector4f (-15, -5.8, -2, 1), Eigen::Vector4f ( 30, 7, 4, 1));
                                                                                                    
  //renderPointCloud(viewer,filterCloud,"filterCloud");
  // to render the input cloud without filtering uncomment the below and comment out line 140
  //renderPointCloud(viewer,inputCloud,"inputCloud");

  /* Clustering and segmenting the filtered cloud */
    // The segmentation algorithm fits a plan to the points and uses the distance tolerance to decide which points belong
    // to that plane. A large tolerance includes more points in the plane
   
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    if(render_obst)
    {
        
        renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,1));
    }
    if(render_plane)
    {
         
        renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    }
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 10, 700);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,1,0)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        }
        if(render_box)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
        }
      ++clusterId;
    }
    // renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    // renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
}

int main (int argc, char** argv)
{
    int cityviewer = true;
    std::string dir = "../src/sensors/data/pcd/data_1/";
    // with the following function calls it will render the point cloud on the opengl window and we will see all the pcd points
    // We can also change the RGB colors of the points to whatever we want by changing the Color Struct
    // if we put renderScene to false in simplehighway() then it will ignore the cars on the road and not render them
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr  temp;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    /*In the code above, you are making use of a new method from point processor called, streamPcd. 
    You tell streamPcd a folder directory that contains all the sequentially ordered pcd files you want to process, 
    and it returns a chronologically ordered vector of all those file names, called stream. You can then go through 
    the stream vector in a couple of ways, one option is to use an iterator. At the end of the above code block, 
    a variable for the input point cloud is also set up.
    */
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    // creates a stream of .pcd objects loads them and processes them one by one and renders each pcd. 
    // after a loop is compleated then it resets the visual and then loads a new image
    while (!viewer->wasStopped ())
    {
         // Clear viewer from old added frames and reset it to add a new frame
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        
        cityBlock(viewer, pointProcessorI, inputCloudI);
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce ();
    } 
}