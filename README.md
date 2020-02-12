# Lidar Obstacle Detection
This project implements the pipeline for extracting trackable box from raw LIDAR sensor measurements. 
It implements filtering, segmentation, clustering, boundbox routines. Filtering was performed using the PCL functions for downsampling and trimming the cloud and custom coded function to organize the cloud in a KD-tree, to segment the plane (RANSAC) and for euclidian clustering.
The pipeline details are reported below.

## PipeLine 
The main step of the implemented pipeline are:
- Load: import a PCD file in a pcl::PointCloud object.
- Filtering: reduce the complexity of the pointcloud by filtering (VoxelGrid, cropping)
- Segmentation: segment road point from object using RANSAC
- Clustering: cluster the non road point
- Bounding box: compute the minimum parallelepiped that that includes each cluster. 
- Rendering: render the objects with the relative bounding box and road to the screen. 

Following are the details for a few of the above steps. 

### Filtering
The number of points collected by the LIDAR sensor is massive. In order to keep the process fast and reduce false targets points are filtered in 3 differents processes taking about 11 ms:
 - Downsampling: pointcloud resolution is downsampled in a voxel grid of fixed dimension (~20 cm). PCL function VoxelGrid was used for this operation. Generally the input has more than 100000 points and number of points of the output is less than 30000 (i.e. more than 70% of point are filtered out).
 - Crop: Remove all the points that are far from LIDAR since they are less important for planning and control and they have a coarse resolution. PCL function CropBox was used for this operation and the bound were empirically determined. Generally after this step the number of point is less than 10000 (i.e., less than 10% of the total).
 - RoofCrop: since the LIDAR is mounted on the car roof some points of the roof are detected from the LIDAR. Therefore it was necessary to filter them out. This was done using again the PCL CropBox function.
 
### Segmentation
After filtering segmentation divides the pointcloud into road and objects (non-road) points. For this operation a custom developed RANSAC algorithm was used. The algorithm iterate over the following 3 main step
  - choose 3 random points from the cloud and fit a plane on them.
  - iterate through all the points in the cloud and compute the distance from the plane for each point and count the number of points which have a distance lower than a defined threshold.
  - return the plane which has more votes as road and define as road the points close to this plane. More time we iterate through these step higher is the probability to detect correctly the ground plane. For this project 100 iteration were found enough to guarantee good performance keeping fast computation (~ 3 ms). 
 
### Clustering
Usually the road is the link joining the different object. Therefore by segmenting out the road from the LIDAR measure usually we can distinguish the different object extracting the cluster in the pointcloud. To identify the cluser a recursive Euclidean Clustering mechanism is implemented. To speed up the process that needs to loop several time through the points, the pointcloud was organized as a 3 dimensional KD-Tree, improving the initial linear complexity of the search process in logarithmic. 

### Bounding Box 
In order to fit each cluster in a bounding box with minimum volume a PCA (principal component analysis) based algorithm is implemented.
 

## Results

### CASE A
The image below is what the results of running the pipeline on the data contained in data_1 folder. On the left part the raw data are presented and the color represents the intensity coding. During the motion several cars parked along the sides of the road and a truck approaching to pass the ego car on the left side can be seen. On the right the fitted bounding boxes (red) around the clustered cars and the passing truck (green dots) are showed together with the segmented road in gray. These information can be used as input information to the path planner, keeping the vehicle on the road trying to avoid any collisions with those obstacles.
<img src="media/ObstacleDetection2FPS.gif" width="800" height="350" />


### CASE B
Similar to case A but with a more complicated scene. A bicyclist is riding in front of the car and several obstacles are spread in the scene.
<img src="media/BikeDetectionFPS.gif" width="800" height="350" />


## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
$> cd SFND_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```
