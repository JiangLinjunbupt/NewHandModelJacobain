#include "Camera.h"


Camera::Camera() {
	//kinect 2.0 ����ȿռ�ĸ�*���� 424 * 512���ڹ�������˵��
	_width = 512;
	_height = 424;

	_focal_length_x = 381.8452f;
	_focal_length_y = 382.1713f;

	centerx = 264.0945f;
	centery = 217.1487f;
}

Eigen::Vector3f Camera::depth_to_world(float i, float j, float depth) {

	Eigen::Vector3f world;

	world(0) = (j - centerx) * (-depth) / _focal_length_x;
	world(1) = (i - centery) * (-depth) / _focal_length_y;
	world(2) = -depth;

	return world;
}

Eigen::Vector3f Camera::world_to_depth_image(const Eigen::Vector3f& world) {
	float x = world[0] / world[2];
	float y = world[1] / world[2];
	x = x*_focal_length_x + centerx;
	y = y*_focal_length_y + centery;
	return Eigen::Vector3f(x, y, -world[2]);
}

Matrix_2x3 Camera::projection_jacobian(const Eigen::Vector3f &p) {
	Matrix_2x3 M;
	M << _focal_length_x / p.z(), 0, -p.x() * _focal_length_x / (p.z()*p.z()),
		0, _focal_length_y / p.z(), -p.y() * _focal_length_y / (p.z()*p.z());
	return M;
}
