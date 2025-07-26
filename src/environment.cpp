// A simple 3d highway enviroment using PCL for exploring self-driving car sensors
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

using segmentedCloudPair = std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>;
using segmentedCloudI = std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>;

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
    // ----------------------------------------------------3
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    bool renderBoxes = false;
    bool renderClusters = true;
    bool renderBoxesQ = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);
    // simulated point cloud from the lidar scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr InputCloud = lidar->scan();
    
    // uncomment to render the simulated lidar scan
    // renderRays(viewer, lidar->position, InputCloud);

    // uncomment to render the point cloud from the lidar scan
    // renderPointCloud(viewer, InputCloud, "cloud", Color(1,0,0));
    
    // TODO:: Create point processor of the PointXYZ type
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    // segment the point cloud into obstacle and plane clouds (to distinguish btw plane and obstacles)
    segmentedCloudPair cloudPair = pointProcessor->SegmentPlane(InputCloud, 100, 0.3); // takes roughly 12 ms
    // uncomment to render the obstacle and plane clouds
    // renderPointCloud(viewer, cloudPair.first, "obstacleCloud", Color(1,0,0));
    // renderPointCloud(viewer, cloudPair.second, "planeCloud", Color(0,1,0));

    // After segmenting, we can now cluster the obstacle cloud to identify the obstacles
    // get the clusters using the default pcl implementation
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor->Clustering(cloudPair.first, 1, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    // assign a unique color and a box to each cluster and render it
    for (auto cluster : clusters) {
        if (renderClusters) {
            std::cout << "cluster size: ";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), colors[clusterId]);
        }
        if (renderBoxes) {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        if (renderBoxesQ) {
            BoxQ boxQ = pointProcessor->BoundingBoxQ(cluster);
            renderBox(viewer, boxQ, clusterId);
        }
        clusterId++;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    bool renderBoxes = true;
    bool renderClusters = true;
    bool renderBoxesQ = false;
    bool renderSegmentation = false;

    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // NOTE: if the color is not specified in the renderPointCloud it will default to using the intensity color coding
    // Original Point cloud 
    //renderPointCloud(viewer, inputCloud, "inputCloud");
    // filter the point cloud (crop the cloud to a roi and downsample the cloud)
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 7, 1, 1));
    // Segment the filtered cloud into obstacle and plane clouds
    segmentedCloudI cloudPair = pointProcessorI->SegmentPlane(filteredCloud, 1000, 0.3);
    if (renderSegmentation) {
      // Render the obstacle and plane clouds
      renderPointCloud(viewer, cloudPair.first, "obstacleCloud", Color(1,0,0));
      renderPointCloud(viewer, cloudPair.second, "planeCloud", Color(0,1,0));
    }
    // Cluster the obstacle cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(cloudPair.first, 1.0, 3, 1000);
    int clusterId = 0;
    // Color palette for more clusters
    std::vector<Color> colors = {
        Color(1,0,0), Color(0,1,0), Color(0,0,1),
        Color(1,1,0), Color(1,0,1), Color(0,1,1),
        Color(0.5,0.5,0), Color(0.5,0,0.5), Color(0,0.5,0.5)
    };
    // assign a unique color and a box to each cluster and render it
    for (auto cluster : clusters) {
        Color clusterColor = colors[clusterId % colors.size()];
        if (renderClusters) {
            std::cout << "cluster size: ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer, cluster, "cluster" + std::to_string(clusterId), clusterColor);
        }
        if (renderBoxes) {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        if (renderBoxesQ) {
            BoxQ boxQ = pointProcessorI->BoundingBoxQ(cluster);
            renderBox(viewer, boxQ, clusterId);
        }
        clusterId++;
    }
    // Render the plane cloud
    renderPointCloud(viewer, cloudPair.second, "planeCloud", Color(0,1,0));
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
    bool simulation = false;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    if (simulation) {
        simpleHighway(viewer);
    } else {
        cityBlock(viewer);
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}