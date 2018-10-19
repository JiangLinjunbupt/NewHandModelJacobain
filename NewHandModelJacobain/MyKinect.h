#pragma once
#include <Kinect.h>
#include"Camera.h"
#include"DataFrame.h"
#include"HandFinder.h"
#include"PointCloud.h"

#include<string>
#include <opencv2\opencv.hpp>
using namespace cv;
using namespace std;


// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


class myKinect
{
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
private:
	const Camera        * camera;

	IKinectSensor       * mySensor;
	IColorFrameReader   * mycolorReader;
	IDepthFrameReader   * mydepthReader;
	IBodyFrameReader	* myBodyReader;
	IBodyFrameSource	* myBodySource;
	ICoordinateMapper   * myMapper;

	ColorSpacePoint     * m_pcolorcoordinate;
	CameraSpacePoint    * m_pcameracoordinate;


	cv::Mat depth_image[2];
	cv::Mat color_image[2];
	cv::Mat sensor_silhouette_buffer;

	int *idx_image_buffer_BACK_BUFFER;
	int *idx_image_buffer_FRONT_BUFFER;

	pcl::PointCloud<pcl::PointXYZ> pointcloud_downSample[2];
	pcl::PointCloud<pcl::PointXYZ> pointcloud_from_depth[2];

	Eigen::Vector3f pointcloud_center[2];


	cv::Mat m_middepth8u;

	int indicator[cDepthWidth*cDepthHeight];
	int NUM_indicator;

public:
	HandFinder * handfinder;
	PointCloud * pointcloud;
	DistanceTransform distance_transform;
public:
	myKinect(Camera *_camera);
	~myKinect();

	HRESULT  InitializeDefaultSensor();//用于初始化kinect
	void fetch_data(DataFrame &frame, HandFinder & handfinder, PointCloud &other_pointcloud);
	bool run();
	void run2();//without color band
};
