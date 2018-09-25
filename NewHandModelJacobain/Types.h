#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <limits>

using namespace std;
using namespace Eigen;
struct Pose
{
	float x;
	float y;
	float z;
	Pose(float xx, float yy, float zz) :x(xx), y(yy), z(zz) {}
	Pose() {}
};

typedef unsigned int uint;
typedef int Integer;
typedef Eigen::Matrix4f Matrix4;
typedef Eigen::Matrix3f Matrix3;
typedef Eigen::VectorXf VectorN;
typedef Eigen::Vector2i Vector2i;

/// More complex matrixes
typedef Eigen::Matrix<float, 2, 3> Matrix_2x3;
typedef Eigen::Matrix<float, 1, Eigen::Dynamic> Matrix_1xN;
typedef Eigen::Matrix<float, 2, Eigen::Dynamic> Matrix_2xN;
typedef Eigen::Matrix<float, 3, Eigen::Dynamic> Matrix_3xN;
typedef Eigen::Matrix<float, Eigen::Dynamic, 2> Matrix_Nx2;
typedef Eigen::Matrix<float, Eigen::Dynamic, 3> Matrix_Nx3;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Matrix_MxN;


/// Nan for the default type
inline float nan() { return std::numeric_limits<float>::quiet_NaN(); }
inline float inf() { return (std::numeric_limits<float>::max)(); }

