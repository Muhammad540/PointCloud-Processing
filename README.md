# Point Cloud Processing using PCL

This repo is for exploratory purposes only.
- [x] Creating a simple 3d highway enviroment and rendering 'cars' on it
- [x] Creating a simulation for a lidar sensor and casting rays to detect objects
- [x] Use PCL to filter and segment the point cloud into obstacles and the ground plane
- [x] Creating a kd tree for clustering points
- [x] Writing a custom implementation of RANSAC line/plane fitting algorithm
- [x] Creating a bounding box for a cluster of points
- [x] Creating a bounding box for a cluster of points using PCA (Minimum Oriented Bounding Box)
- [x] Load real PCL data from a file
- [x] Filter
    - [x] Downsample using Voxel Grid
    - [x] Extract region of interest using CropBox functionality
    - [x] Remove roof points using CropBox to get the indices of the roof points
    - [x] Given the indices of the roof points, remove them from the point cloud