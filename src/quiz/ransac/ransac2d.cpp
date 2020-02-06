/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include <pcl/filters/random_sample.h>

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

std::array<double, 3> crossProduct(std::array<double, 3> v1, std::array<double, 3> v2){
	std::array<double, 3> out{
		{
			v1[1]*v2[2] - v1[2]*v2[1],
			v1[2]*v2[0] - v1[0]*v2[2],
			v1[0]*v2[1] - v1[1]*v2[0]						
		}
	};
	
	return out;	
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliersCurrent;	
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	for(auto i=0; i< maxIterations; i++){		
		// Randomly sample subset and fit line
		pcl::RandomSample<pcl::PointXYZ> sample(false);
		sample.setSeed(i);
		sample.setInputCloud (cloud);
		sample.setSample(3);
		std::vector<int> indices;
		sample.filter (indices);

		const auto& p1 = cloud->points[indices[0]];
		const auto& p2 = cloud->points[indices[1]];
		const auto& p3 = cloud->points[indices[2]];

		const std::array<double, 3> v1{{p2.x - p1.x, p2.y - p1.y, p2.z - p1.z}};
		const std::array<double, 3> v2{{p3.x - p1.x, p3.y - p1.y, p3.z - p1.z}};

		const std::array<double, 3> perp = crossProduct(v1, v2);

		//std::cout << indices.size() << " " << indices[0] << " " << indices[1] << std::endl;
		const double A = perp[0];
		const double B = perp[1];		
		const double C = perp[2];
		const double D = -(A*p1.x + B*p1.y + C*p1.z);
		
		// d = abs|A*x+B*y+C|/sqrt(A*A+B*B) < distanceTol 
		// abs|A*x+B*y+C| < distanceTol*sqrt(A*A+B*B)
		const double threshold = distanceTol * sqrt(A*A + B*B + C*C);

		// Measure distance between every point and fitted line		
		for(size_t k=0; k<cloud->points.size(); k++){
			const auto& pt = cloud->points[k];
			const double dist = std::abs(A*pt.x + B*pt.y + C*pt.z + D);
			// If distance is smaller than threshold count it as inlier
			if(dist < threshold){
				inliersCurrent.insert(k);
			} 
		}

		if(inliersCurrent.size() >= inliersResult.size()){		
			inliersResult.swap(inliersCurrent);
		}
		inliersCurrent.clear();
	}

	// Return indicies of inliers from fitted line with most inliers		
	return inliersResult;

}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliersCurrent;	
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	for(auto i=0; i< maxIterations; i++){		
		// Randomly sample subset and fit line
		pcl::RandomSample<pcl::PointXYZ> sample(false);
		sample.setSeed(i);
		sample.setInputCloud (cloud);
		sample.setSample(2);
		std::vector<int> indices;
		sample.filter (indices);
		//std::cout << indices.size() << " " << indices[0] << " " << indices[1] << std::endl;
		const double A = cloud->points[indices[0]].y- cloud->points[indices[1]].y;		
		const double B = cloud->points[indices[1]].x - cloud->points[indices[0]].x;
		const double xy2 = cloud->points[indices[0]].x * cloud->points[indices[1]].y;		
		const double xy1 = cloud->points[indices[1]].x * cloud->points[indices[0]].y;
		const double C = xy2 - xy1;
		
		// d = abs|A*x+B*y+C|/sqrt(A*A+B*B) < distanceTol 
		// abs|A*x+B*y+C| < distanceTol*sqrt(A*A+B*B)
		const double threshold = distanceTol * sqrt(A*A + B*B);

		// Measure distance between every point and fitted line		
		for(size_t k=0; k<cloud->points.size(); k++){
			const auto& pt = cloud->points[k];
			const double dist = std::abs(A*pt.x + B*pt.y + C);
			// If distance is smaller than threshold count it as inlier
			if(dist < threshold){
				inliersCurrent.insert(k);
			} 
		}

		if(inliersCurrent.size() >= inliersResult.size()){		
			inliersResult.swap(inliersCurrent);
		}
		inliersCurrent.clear();
	}

	// Return indicies of inliers from fitted line with most inliers		
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();  // for 2D case
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();   // for 3D case
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 50, 1);   // for 2D case
	std::unordered_set<int> inliers = RansacPlane(cloud, 500, 0.2);   // for 3D case

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
