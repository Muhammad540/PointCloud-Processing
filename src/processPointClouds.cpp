// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <memory>


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, 
     float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Reference: https://pointclouds.org/documentation/tutorials/voxel_grid.html and https://pointclouds.org/documentation/classpcl_1_1_crop_box_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
              << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
    
    // Voxel Grid Downsampling
    typename pcl::PointCloud<PointT>::Ptr filteredCloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filteredCloud);
    
    // Crop to region of interest
    typename pcl::PointCloud<PointT>::Ptr croppedCloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::CropBox<PointT> cropBoxFilter;
    cropBoxFilter.setInputCloud(filteredCloud);
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);
    cropBoxFilter.filter(*croppedCloud);

    // Remove roof points
    // USing True to enable index extraction
    pcl::CropBox<PointT> roofFilter(true);  
    roofFilter.setInputCloud(croppedCloud);
    roofFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roofFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    
    // Get roof indices (points to remove)
    pcl::IndicesPtr roofIndices(new pcl::Indices);
    // This gets the indices, not the points
    roofFilter.filter(*roofIndices);  
    
    // Remove roof points
    // Basic idea is: get the indices first and then use ExtractIndices to create the cloud with indices removed
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(croppedCloud);
    extract.setIndices(roofIndices);
    // Keep everything EXCEPT roof points
    extract.setNegative(true); 
    extract.filter(*croppedCloud);

    std::cerr << "PointCloud after filtering: " << croppedCloud->width * croppedCloud->height 
              << " data points (" << pcl::getFieldsList(*croppedCloud) << ")." << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return croppedCloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud = std::make_shared<pcl::PointCloud<PointT>>();
    typename pcl::PointCloud<PointT>::Ptr planeCloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::ExtractIndices<PointT> extract;
    // Get the plane cloud
    for (int index : inliers->indices){ // indices is a vector of indices of the points that belong to the plane
        planeCloud->points.push_back(cloud->points[index]);
    }
    // Extract the obstacle cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	auto inliers = std::make_shared<pcl::PointIndices>();
    // TODO:: Fill in this function to find inliers for the cloud.
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    // Create segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
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
      return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(cloud, cloud);
    }
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

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KDTree object for the search method of extraction
    typename pcl::search::KdTree<PointT>::Ptr tree = std::make_unique<pcl::search::KdTree<PointT>>();
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (const auto& cluster : cluster_indices)
    {
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster = std::make_shared<pcl::PointCloud<PointT>>();
      for (const auto& idx : cluster.indices) {
        cloud_cluster->push_back((*cloud)[idx]);
      }
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
      clusters.push_back(cloud_cluster);
    }

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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters using pca
    // PCA is used to find the minimum oriented bounding box of the cluster cloud
    // The PCA is done on the covariance matrix of the cluster cloud
    // Compute the principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covarianceMatrix;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covarianceMatrix);
    // Get the eigenvectors and eigenvalues of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covarianceMatrix, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigenSolver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigenSolver.eigenvalues();
    // This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    // the signs are different and the box doesn't get correctly oriented in some cases.
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    /* // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cluster);
    pca.project(*cluster, *cloudPCAprojection);
    std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
    std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
    // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
    */
    /*
    These eigen vectors are used to transform the point cloud to the origin point (0,0,0) such that
    the eigenvectors correspond to the axes of the space. The minimum, maximum point and the
    middle of the diagonal between these two points are calculated for the transformed cloud
    (also referred to as the projected cloud when using PCL's PCA interface)
    */
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    /*
    Finally, the quaternion is calculated using the eigenvectors 
    (which determines how the final box gets rotated), 
    and the transform to put the box in correct location is calculated. 
    The minimum and maximum points are used to determine the box width, height, and depth.
    */
    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    // Create box
    BoxQ box;
    box.bboxTransform = bboxTransform;
    box.bboxQuaternion = bboxQuaternion;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;
    
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