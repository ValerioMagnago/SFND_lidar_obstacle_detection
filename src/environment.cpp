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

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer,inputCloud,"inputCloud");

    // Experiment with the ? values and find what works best
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2 , Eigen::Vector4f (-30, -9, -3, 1), Eigen::Vector4f ( 30, 9, 2, 1));
    renderPointCloud(viewer,filterCloud,"filterCloud");


    // TODO:: Create point processor
    const int maxIteration = 1000;
    const float threshold = 0.3; // distance from plane
    ProcessPointClouds<pcl::PointXYZI> processPointCloud;
    auto segmentedCloud = processPointCloud.SegmentPlane(filterCloud, maxIteration, threshold);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = processPointCloud.Clustering(segmentedCloud.first, 0.5, 4, 5000);

    renderPointCloud(viewer, segmentedCloud.second, "road", Color(0.5,0.5,0.5)); 

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size "; processPointCloud.numPoints(cluster);
        auto box = processPointCloud.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        renderPointCloud(viewer,cluster,"car"+std::to_string(clusterId),colors[clusterId%colors.size()]);
        ++clusterId;
    }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    // bool renderScene = true;
    bool renderScene = false; // set to false to see only the cloud
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    const double groundSlope = 0.;
    Lidar* lidar = new Lidar(cars, groundSlope);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();    
    //renderRays(viewer, lidar->position, inputCloud);
    //clearRays(viewer);
    //renderPointCloud(viewer, inputCloud, "test", Color(1,1,1));

    // TODO:: Create point processor
    const int maxIteration = 1000;
    const float threshold = 0.2; // distance from plane
    ProcessPointClouds<pcl::PointXYZ> processPointCloud;
    auto segmentedCloud = processPointCloud.SegmentPlane(inputCloud, maxIteration, threshold);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processPointCloud.Clustering(segmentedCloud.first, 1.0, 3, 30);

    renderPointCloud(viewer, segmentedCloud.second, "objs", Color(1,1,0)); 

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size "; processPointCloud.numPoints(cluster);
        auto box = processPointCloud.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        renderPointCloud(viewer,cluster,"car"+std::to_string(clusterId),colors[clusterId%colors.size()]);
        ++clusterId;
    }
       
    renderPointCloud(viewer, segmentedCloud.second, "road", Color(0.5,0.5,0.5));

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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    //cityBlock(viewer);
    //simpleHighway(viewer);

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();  

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);   

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}

