/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Time to run RANSAC
	auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	while(maxIterations--) {
		// Randomly sample two distinct points
		std::unordered_set<int> inliers;
		while (inliers.size() < 2) {
			inliers.insert(rand()%(cloud->points.size())); // since inliers is a set, distinct points are guaranteed
		}

		float x1, y1, x2, y2;
		auto it = inliers.begin();
		// go into the cloud and grab a point at a particular index
		x1 = cloud->points[*it].x;
		y1 = cloud->points[*it].y;
		it++;
		x2 = cloud->points[*it].x;
		y2 = cloud->points[*it].y;

		// formula for line: ax + by + c = 0
		float a = (y1-y2);
		float b = (x2-x1);
		float c = (x1*y2 - x2*y1);
		
		// Measure distance between every point and fitted line
		for(int index = 0; index < cloud->points.size(); index++) {
			// check if the point is already in the inliers set ( 0 means not in the set)
			if (inliers.count(index)>0){
				continue;
			}
			pcl::PointXYZ point = cloud->points[index];
			float distance = fabs(a * point.x + b * point.y + c) / sqrt(a * a + b * b);
			if (distance <= distanceTol) {
				inliers.insert(index);
			}
		}
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers; // Update inliersResult if current inliers are more
		}
	}
	
	auto endTime = std::chrono::steady_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC Line took " << duration.count() << " milliseconds" << std::endl;
	
	return inliersResult; // Return indices of inliers from fitted line with most inliers
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Time to run RANSAC
	auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	while(maxIterations--) {
		// Randomly sample two distinct points
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) {
			inliers.insert(rand()%(cloud->points.size())); // since inliers is a set, distinct points are guaranteed
		}

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto it = inliers.begin();
		// go into the cloud and grab a point at a particular index
		x1 = cloud->points[*it].x;
		y1 = cloud->points[*it].y;
		z1 = cloud->points[*it].z;
		it++;
		x2 = cloud->points[*it].x;
		y2 = cloud->points[*it].y;
		z2 = cloud->points[*it].z;
		it++;
		x3 = cloud->points[*it].x;
		y3 = cloud->points[*it].y;
		z3 = cloud->points[*it].z;

		// formula for plane: ax + by + cz + d = 0
		float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		float d = -(a*x1 + b*y1 + c*z1);
		
		// Measure distance between every point and fitted line
		for(int index = 0; index < cloud->points.size(); index++) {
			// check if the point is already in the inliers set ( 0 means not in the set)
			if (inliers.count(index)>0){
				continue;
			}
			pcl::PointXYZ point = cloud->points[index];
			float distance = fabs(a * point.x + b * point.y + c * point.z + d) / sqrt(a * a + b * b + c * c);
			if (distance <= distanceTol) {
				inliers.insert(index);
			}
		}
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers; // Update inliersResult if current inliers are more
		}
	}
	
	auto endTime = std::chrono::steady_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC Plane took " << duration.count() << " milliseconds" << std::endl;
	
	return inliersResult; // Return indices of inliers from fitted line with most inliers
}

int main (int argc, char **argv)
{
	std::string typeData = "2D";
	if (argc == 2) {
		typeData = argv[1];
	} else {
		std::cout << "Usage: " << argv[0] << " [2D/3D]" << std::endl;
		return -1;
	}
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	if (typeData == "2D") {
		cloud = CreateData();
	}
	else if (typeData == "3D") {
		cloud = CreateData3D();
	}
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers;
	if (typeData == "2D") {
		inliers = RansacLine(cloud, 100, 0.2); // 0 ms
	}
	else if (typeData == "3D") {
		inliers = RansacPlane(cloud, 100, 0.2); // RANSAC Plane takes on average 19 milliseconds
	}

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
