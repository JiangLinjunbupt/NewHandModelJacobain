#include"PointCloud.h"


void PointCloud::DepthMatToPointCloud(cv::Mat& depth, HandFinder* hanfinder)
{
	this->pointcloud_from_depth.points.clear();

	cv::Mat depth_flip;
	cv::flip(depth, depth_flip, 0);
	for (int i = 0; i < hanfinder->num_sensor_points; i++)
	{
		pcl::PointXYZ p;

		int col = hanfinder->sensor_indicator[i] % 512;        //x
		int row = hanfinder->sensor_indicator[i] / 512;        //y

		Integer z = depth_flip.at<unsigned short>(row, col);

		Eigen::Vector3f p_pixel = camera->depth_to_world(row, col, z);

		p.x = p_pixel.x();
		p.y = p_pixel.y();
		p.z = p_pixel.z();

		this->PointCloud_center_x += p_pixel.x();
		this->PointCloud_center_y += p_pixel.y();
		this->PointCloud_center_z += p_pixel.z();

		this->pointcloud_from_depth.points.push_back(p);
	}

	if (this->pointcloud_from_depth.points.size() != 0)
	{
		this->PointCloud_center_x = this->PointCloud_center_x / (float)(this->pointcloud_from_depth.points.size());
		this->PointCloud_center_y = this->PointCloud_center_y / (float)(this->pointcloud_from_depth.points.size());
		this->PointCloud_center_z = this->PointCloud_center_z / (float)(this->pointcloud_from_depth.points.size());
	}
}

void PointCloud::DepthMatToPointCloud(cv::Mat& depth, int *indicator, int NUM_indicator,int *palm_indicator,int NUM_palm_indicator)
{
	this->pointcloud_from_depth.points.clear();
	cv::Mat depth_flip;
	cv::flip(depth, depth_flip, 0);

	for (int i = 0; i < NUM_indicator; i++)
	{
		int index = indicator[i];
		int col = index % 512;        //x
		int row = index / 512;        //y

		pcl::PointXYZ p;

		Integer z = depth_flip.at<unsigned short>(row, col);

		Eigen::Vector3f p_pixel = camera->depth_to_world(row, col, z);

		p.x = p_pixel.x();
		p.y = p_pixel.y();
		p.z = p_pixel.z();

		this->pointcloud_from_depth.points.push_back(p);

	}

	for (int i = 0; i < NUM_palm_indicator; i++)
	{
		int index = palm_indicator[i];
		int col = index % 512;        //x
		int row = index / 512;        //y

		pcl::PointXYZ p;

		Integer z = depth_flip.at<unsigned short>(row, col);

		Eigen::Vector3f p_pixel = camera->depth_to_world(row, col, z);

		p.x = p_pixel.x();
		p.y = p_pixel.y();
		p.z = p_pixel.z();

		this->PointCloud_center_x += p_pixel.x();
		this->PointCloud_center_y += p_pixel.y();
		this->PointCloud_center_z += p_pixel.z();
	}

	if (this->pointcloud_from_depth.size() != 0)
	{
		this->PointCloud_center_x = this->PointCloud_center_x / (float)(NUM_palm_indicator);
		this->PointCloud_center_y = this->PointCloud_center_y / (float)(NUM_palm_indicator);
		this->PointCloud_center_z = this->PointCloud_center_z / (float)(NUM_palm_indicator);
	}

	if (this->PointCloud_center_x > 10000) this->PointCloud_center_x = 10000;
	if (this->PointCloud_center_x < -10000) this->PointCloud_center_x = -10000;

	if (this->PointCloud_center_y > 10000) this->PointCloud_center_y = 10000;
	if (this->PointCloud_center_y < -10000) this->PointCloud_center_y = -10000;

	if (this->PointCloud_center_z > 10000) this->PointCloud_center_z = 10000;
	if (this->PointCloud_center_z < -10000) this->PointCloud_center_z = -10000;


}

void PointCloud::Filter_visible_cloud()
{
	if (pointcloud_from_depth.points.size() > 0)
	{
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(pointcloud_from_depth.makeShared());
		sor.setMeanK(100);
		sor.setStddevMulThresh(1.0);
		sor.filter(pointcloud_filtered);
	}
}


void PointCloud::downSample()
{
	if (pointcloud_filtered.points.size())
	{
		pcl::VoxelGrid<pcl::PointXYZ> sor;  //����դ���²�������
		sor.setInputCloud(pointcloud_filtered.makeShared());             //ԭʼ����
		sor.setLeafSize(8.0f, 8.0f, 8.0f);    // ���ò������ش�С
		sor.filter(pointcloud_downsample);        //����

		//std::cout << "after filter ,the point cloud size is : " << after_cloud.points.size() << std::endl;
	}
}