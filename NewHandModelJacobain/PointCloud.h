#pragma once

#include<pcl\point_types.h>
#include<pcl\filters\voxel_grid.h>
#include<pcl\filters\statistical_outlier_removal.h>
//必须放在最前面，不知道为什么放在opencv后面就会无法使用statistical_outlier_removal这个方法（幺蛾子事件）

#include<opencv2/opencv.hpp>    
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<vector>
#include"Camera.h"
#include"HandFinder.h"

using namespace std;


class PointCloud
{

public:
	Camera * camera;

	pcl::PointCloud<pcl::PointXYZ> pointcloud_from_depth;
	pcl::PointCloud<pcl::PointXYZ> pointcloud_filtered;
	pcl::PointCloud<pcl::PointXYZ> pointcloud_downsample;

	float PointCloud_center_x, PointCloud_center_y, PointCloud_center_z = 0.0f;

	PointCloud() {}
	~PointCloud() {}

	void DepthMatToPointCloud(cv::Mat& depth, HandFinder* hanfinder);
	void DepthMatToPointCloud(cv::Mat& depth, int *indicator, int NUM_indicator);
	
	void Filter_visible_cloud();

	void downSample();

};