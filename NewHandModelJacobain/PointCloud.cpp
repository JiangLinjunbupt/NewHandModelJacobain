#include"PointCloud.h"


void PointCloud::DepthMatToPointCloud(cv::Mat& depth, HandFinder* hanfinder)
{
	this->pointcloud_from_depth.points.clear();

	for (int i = 0; i < hanfinder->num_sensor_points; i++)
	{
		pcl::PointXYZ p;

		int col = hanfinder->sensor_indicator[i] % 512;        //x
		int row = hanfinder->sensor_indicator[i] / 512;        //y

		Integer z = depth.at<unsigned short>(row, col);

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


void PointCloud::Filter_visible_cloud()
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(pointcloud_from_depth.makeShared());
	sor.setMeanK(100);
	sor.setStddevMulThresh(1.0);
	sor.filter(pointcloud_filtered);
}


void PointCloud::downSample()
{
	pcl::VoxelGrid<pcl::PointXYZ> sor;  //体素栅格下采样对象
	sor.setInputCloud(pointcloud_filtered.makeShared());             //原始点云
	sor.setLeafSize(10.0f, 10.0f, 10.0f);    // 设置采样体素大小
	sor.filter(pointcloud_downsample);        //保存

	//std::cout << "after filter ,the point cloud size is : " << after_cloud.points.size() << std::endl;
}