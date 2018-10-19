#pragma once

#include<pcl\point_types.h>
#include<pcl\filters\voxel_grid.h>
#include<pcl\filters\statistical_outlier_removal.h>
//���������ǰ�棬��֪��Ϊʲô����opencv����ͻ��޷�ʹ��statistical_outlier_removal����������۶����¼���

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