#pragma once
#include"Types.h"
#include<fstream>
#include<iostream>
#include<vector>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#define PI 3.1415926


enum dof_type
{
	x_axis_rotate, y_axis_rotate, z_axis_rotate,
	x_axis_trans, y_axis_trans, z_axis_trans
};

class Joint_handmodel
{
public:
	std::string joint_name;
	int joint_index;

	bool HasChild;

	Eigen::Vector4f GlobalInitPosition;       //position before change
	Eigen::Vector4f ChildGlobalInitPosition;

	bool joint_dof[3];
	Eigen::Vector4f dof_axis[3];//列向量；

	int params_length;
	int* params_index;
	int* params_type;

	int parent_joint_index;

	// local coordiate // 4*4
	Eigen::MatrixXf local;     //模型矩阵（世界坐标等于模型矩阵乘以模型坐标）								
							   // local rotation // 4*4
	Eigen::MatrixXf rotation;
	// transformation to parent: 4*4
	Eigen::MatrixXf trans;
	// global coordinate // 4*4
	Eigen::MatrixXf global;


	Eigen::Vector4f CorrespondingPosition;   // position after change
	Eigen::Vector4f CorrespondingAxis[3];    // Axis after change 

	Joint_handmodel() {
		HasChild = true;
		dof_axis[0] << 1, 0, 0, 1;
		dof_axis[1] << 0, 1, 0, 1;
		dof_axis[2] << 0, 0, 1, 1;
		local = Eigen::MatrixXf::Zero(4, 4);
		rotation = Eigen::MatrixXf::Identity(4, 4);
		trans = Eigen::MatrixXf::Zero(4, 4);
		global = Eigen::MatrixXf::Zero(4, 4);
	}
};


class HandModel
{
public:
	Joint_handmodel* Joints;

	Matrix_Nx3 FaceIndex;
	Matrix_Nx3 Vectices;

	Matrix_Nx3 Vertices_normal;    //和Vectices一起在加载的时候初始化为0矩阵
	vector<Vector3f> Visible_vertices;
	vector<int> Visible_vertices_index;

	Matrix_Nx3 Joint_matrix;  //这里是将joint*数组中的关节点位置整合到一个矩阵中
	Matrix_Nx3 Target_vertices;
	Matrix_Nx3 Load_visible_vertices;
	Matrix_Nx3 Target_joints;
	Matrix_MxN Weights;

	float hand_scale;
	int NumofJoints;
	int NumofVertices;
	int NumofFaces;
	int Load_visible_vertices_NUM;
	int NumberofParams;

	float *Params;
	int* ParamsUpperBound;
	int* ParamsLowerBound;

	Eigen::Vector4f GlobalPosition;
	Eigen::MatrixXf vertices_update_;

	Eigen::MatrixXf jacobian;
	Eigen::MatrixXf Joints_jacobian;

	Eigen::MatrixXf jacobian_correspond;

	HandModel();
	~HandModel() { delete ParamsLowerBound; delete ParamsUpperBound; delete Params; }
	void Updata(float *Params);
	void Updata_Jacobian();
	void Updata_joints_Jacobian();

	void Updata_Jacobian_correspond(std::vector<int> & cor);

	void MoveToJointTarget();
	void MoveToVerticeTarget();

	void MoveToDownSamoleCorrespondingVertices(pcl::PointCloud<pcl::PointXYZ>& p, std::vector<int>& cor);

	bool Solved;
private:
	void load_faces(char* file);
	void load_vertices(char* file);
	void load_weight(char* file);

	void compute_local_coordinate();
	void compute_parent_child_transform();
	void compute_global_matrix();
	void compute_rotation_matrix(float* params);
	void set_one_rotation(Pose p, int index);
	void Updata_Joints_Axis();
	void Updata_Vertics();

	void normalize(float axis[3]) {
		float sum = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
		axis[0] /= (sum + 1e-20f);
		axis[1] /= (sum + 1e-20f);
		axis[2] /= (sum + 1e-20f);
	}

	void cross_product(float axis_a[3], float axis_b[3], float axis_c[3]) {
		axis_c[0] = axis_a[1] * axis_b[2] - axis_a[2] * axis_b[1];
		axis_c[1] = axis_a[2] * axis_b[0] - axis_a[0] * axis_b[2];
		axis_c[2] = axis_a[0] * axis_b[1] - axis_a[1] * axis_b[0];
	}

	void save_target_vertices();
	void load_target_vertices();
	void save_target_joints();
	void load_target_joints();

	void save_visible_vertices();
	void load_visible_vertices();

	void load_glove_data();
	void Compute_normal_And_visibel_vertices();

	MatrixXf Compute_joint_Limited(Eigen::VectorXf & e_limit);
};