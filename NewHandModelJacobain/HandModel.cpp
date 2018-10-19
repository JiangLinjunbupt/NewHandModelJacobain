#include"HandModel.h"
#include<random>


void HandModel::load_faces(char* filename)
{
	std::ifstream f;
	f.open(filename, std::ios::in);
	f >> NumofFaces;
	FaceIndex = Eigen::MatrixXi::Zero(NumofFaces, 3);
	Face_norm.resize(NumofFaces);
	for (int i = 0; i < NumofFaces; ++i) {
		f >> FaceIndex(i, 0) >> FaceIndex(i, 1) >> FaceIndex(i, 2);
	}
	f.close();
	printf("Load Faces succeed!!!\n");
	std::cout << "num of Face is: " << NumofFaces << std::endl;
}

void HandModel::load_vertices(char* filename)
{
	std::ifstream f;
	f.open(filename, std::ios::in);
	f >> NumofVertices;
	Vectices = Eigen::MatrixXf::Zero(NumofVertices, 3);
	Vertices_normal = Eigen::MatrixXf::Zero(NumofVertices, 3);
	for (int i = 0; i < NumofVertices; ++i) {
		f >> Vectices(i, 0) >> Vectices(i, 1) >> Vectices(i, 2);
	}
	f.close();
	printf("Load vertices succeed!!!\n");
	std::cout << "num of vertices is: " << NumofVertices << std::endl;
}

void HandModel::load_weight(char* filename)
{
	std::ifstream f;
	f.open(filename, std::ios::in);
	Weights = Eigen::MatrixXf::Zero(NumofVertices, NumofJoints);
	for (int i = 0; i < NumofVertices; ++i) {
		for (int j = 0; j < NumofJoints; ++j)
		{
			f >> Weights(i, j);
		}
	}
	f.close();
	printf("Load weights succeed!!!\n");
}

void HandModel::SetParamsBound()
{
	//wrist
	ParamsUpperBound[0] = 1.0e+10f;	ParamsLowerBound[0] = -1.0e+10f;
	ParamsUpperBound[1] = 1.0e+10f;	ParamsLowerBound[1] = -1.0e+10f;
	ParamsUpperBound[2] = 1.0e+10f;	ParamsLowerBound[2] = -1.0e+10f;
	ParamsUpperBound[3] = 1.0e+10f;	ParamsLowerBound[3] = -1.0e+10f;
	ParamsUpperBound[4] = 1.0e+10f;	ParamsLowerBound[4] = -1.0e+10f;
	ParamsUpperBound[5] = 1.0e+10f;	ParamsLowerBound[5] = -1.0e+10f;

	//thumb
	ParamsUpperBound[6] = 50.0f;   ParamsLowerBound[6] = -15.0f;
	ParamsUpperBound[7] = 0.0f;	   ParamsLowerBound[7] = -60.0f;
	ParamsUpperBound[8] = 0.0f;	   ParamsLowerBound[8] = -50.0f;
	ParamsUpperBound[9] = 0.0f;	   ParamsLowerBound[9] = -98.0f;

	//index
	ParamsUpperBound[10] = 98.0f;  ParamsLowerBound[10] = -10.0f;
	ParamsUpperBound[11] = 30.0f;  ParamsLowerBound[11] = -20.0f;
	ParamsUpperBound[12] = 98.0f;  ParamsLowerBound[12] = 0.0f;
	ParamsUpperBound[13] = 98.0f;  ParamsLowerBound[13] = 0.0f;

	//middle
	ParamsUpperBound[14] = 98.0f;  ParamsLowerBound[14] = -10.0f;
	ParamsUpperBound[15] = 20.0;   ParamsLowerBound[15] = -10.0f;
	ParamsUpperBound[16] = 98.0f;  ParamsLowerBound[16] = 0.0f;
	ParamsUpperBound[17] = 98.0f;  ParamsLowerBound[17] = 0.0f;

	//ring
	ParamsUpperBound[18] = 98.0f;  ParamsLowerBound[18] = -10.0f;
	ParamsUpperBound[19] = 20.0f;  ParamsLowerBound[19] = -15.0f;
	ParamsUpperBound[20] = 98.0f;  ParamsLowerBound[20] = 0.0f;
	ParamsUpperBound[21] = 98.0f;  ParamsLowerBound[21] = 0.0f;

	//pinkey
	ParamsUpperBound[22] = 98.0f;  ParamsLowerBound[22] = -10.0f;
	ParamsUpperBound[23] = 20.0f;  ParamsLowerBound[23] = -20.0f;
	ParamsUpperBound[24] = 98.0f; ParamsLowerBound[24] = 0.0f;
	ParamsUpperBound[25] = 98.0f;  ParamsLowerBound[25] = 0.0f;

}

HandModel::HandModel(Camera *camera_):camera(camera_)
{
	NumofJoints = 22;
	Joint_matrix = Eigen::MatrixXf::Zero(NumofJoints, 3);

	Joints = new Joint_handmodel[22];
	//wrist
	{
		Joints[0].joint_name = "Wrist";
		Joints[0].joint_index = 0;
		Joints[0].GlobalInitPosition << 0, 0, 0, 1;
		Joints[0].ChildGlobalInitPosition << 4.74695f, 71.5019f, -12.1453f, 1;
		Joints[0].joint_dof[0] = true; Joints[0].joint_dof[1] = true; Joints[0].joint_dof[2] = true;
		Joints[0].parent_joint_index = -1;

		Joints[0].params_length = 6;
		Joints[0].params_index = new int[6]; Joints[0].params_type = new int[6];
		Joints[0].params_index[0] = 0; Joints[0].params_type[0] = dof_type(x_axis_trans);
		Joints[0].params_index[1] = 1; Joints[0].params_type[1] = dof_type(y_axis_trans);
		Joints[0].params_index[2] = 2; Joints[0].params_type[2] = dof_type(z_axis_trans);
		Joints[0].params_index[3] = 3; Joints[0].params_type[3] = dof_type(x_axis_rotate);
		Joints[0].params_index[4] = 4; Joints[0].params_type[4] = dof_type(y_axis_rotate);
		Joints[0].params_index[5] = 5; Joints[0].params_type[5] = dof_type(z_axis_rotate);
	}

	//handbone
	{
		Joints[1].joint_name = "Handbone";
		Joints[1].joint_index = 1;
		Joints[1].GlobalInitPosition << 4.74695f, 71.5019f, -12.1453f, 1;
		Joints[1].ChildGlobalInitPosition << 9.00001f, 101.267f, -3.93472f, 1;
		Joints[1].joint_dof[0] = false; Joints[1].joint_dof[1] = false; Joints[1].joint_dof[2] = false;
		Joints[1].parent_joint_index = 0;

		Joints[1].params_length = 0;
		Joints[1].params_index = NULL; Joints[1].params_type = NULL;
	}
	//index

	{
		{
			Joints[2].joint_name = "IndexLower";
			Joints[2].joint_index = 2;
			Joints[2].GlobalInitPosition << 29.2408f, 96.351f, -3.11671f, 1;
			Joints[2].ChildGlobalInitPosition << 35.6817f, 128.786f, -3.3307f, 1;
			Joints[2].joint_dof[0] = false; Joints[2].joint_dof[1] = true; Joints[2].joint_dof[2] = true;
			Joints[2].params_length = 2;
			Joints[2].params_index = new int[2]; Joints[2].params_type = new int[2];
			Joints[2].params_index[0] = 10; Joints[2].params_type[0] = dof_type(y_axis_rotate);
			Joints[2].params_index[1] = 11; Joints[2].params_type[1] = dof_type(z_axis_rotate);
			Joints[2].parent_joint_index = 1;
		}


		{
			Joints[3].joint_name = "IndexMiddle";
			Joints[3].joint_index = 3;
			Joints[3].GlobalInitPosition << 35.6817f, 128.786f, -3.3307f, 1;
			Joints[3].ChildGlobalInitPosition << 39.1804f, 159.882f, -3.93921f, 1;
			Joints[3].joint_dof[0] = false; Joints[3].joint_dof[1] = true; Joints[3].joint_dof[2] = false;
			Joints[3].params_length = 1;
			Joints[3].params_index = new int[1]; Joints[3].params_type = new int[1];
			Joints[3].params_index[0] = 12; Joints[3].params_type[0] = dof_type(y_axis_rotate);
			Joints[3].parent_joint_index = 2;
		}

		{
			Joints[4].joint_name = "IndexTop";
			Joints[4].joint_index = 4;
			Joints[4].GlobalInitPosition << 39.1804f, 159.882f, -3.93921f, 1;
			Joints[4].ChildGlobalInitPosition << 40.8995f, 178.091f, -5.84128f, 1;
			Joints[4].joint_dof[0] = false; Joints[4].joint_dof[1] = true; Joints[4].joint_dof[2] = false;
			Joints[4].params_length = 1;
			Joints[4].params_index = new int[1]; Joints[4].params_type = new int[1];
			Joints[4].params_index[0] = 13; Joints[4].params_type[0] = dof_type(y_axis_rotate);
			Joints[4].parent_joint_index = 3;
		}

		{
			Joints[5].joint_name = "IndexSite";
			Joints[5].joint_index = 5;
			Joints[5].GlobalInitPosition << 40.8995f, 178.091f, -5.84128f, 1;
			Joints[5].HasChild = false;
			Joints[5].joint_dof[0] = false; Joints[5].joint_dof[1] = false; Joints[5].joint_dof[2] = false;
			Joints[5].params_length = 0;
			Joints[5].params_index = NULL; Joints[5].params_type = NULL;
			Joints[5].parent_joint_index = 4;
		}
	}

	//middle
	{
		{
			Joints[6].joint_name = "MiddleLower";
			Joints[6].joint_index = 6;
			Joints[6].GlobalInitPosition << 9.00001f, 101.267f, -3.93472f, 1;
			Joints[6].ChildGlobalInitPosition << 8.19879f, 138.712f, -5.20814f, 1;
			Joints[6].joint_dof[0] = false; Joints[6].joint_dof[1] = true; Joints[6].joint_dof[2] = false;
			Joints[6].params_length = 2;
			Joints[6].params_index = new int[2]; Joints[6].params_type = new int[2];
			Joints[6].params_index[0] = 14;  Joints[6].params_type[0] = dof_type(y_axis_rotate);
			Joints[6].params_index[1] = 15; Joints[6].params_type[1] = dof_type(z_axis_rotate);
			Joints[6].parent_joint_index = 1;
		}
		{
			Joints[7].joint_name = "MiddleMiddle";
			Joints[7].joint_index = 7;
			Joints[7].GlobalInitPosition << 8.19879f, 138.712f, -5.20814f, 1;
			Joints[7].ChildGlobalInitPosition << 7.12465f, 168.352f, -4.7175f, 1;
			Joints[7].joint_dof[0] = false; Joints[7].joint_dof[1] = true; Joints[7].joint_dof[2] = false;
			Joints[7].params_length = 1;
			Joints[7].params_index = new int[1]; Joints[7].params_type = new int[1];
			Joints[7].params_index[0] = 16; Joints[7].params_type[0] = dof_type(y_axis_rotate);
			Joints[7].parent_joint_index = 6;
		}

		{
			Joints[8].joint_name = "MiddleTop";
			Joints[8].joint_index = 8;
			Joints[8].GlobalInitPosition << 7.12465f, 168.352f, -4.7175f, 1;
			Joints[8].ChildGlobalInitPosition << 6.95019f, 185.733f, -7.207f, 1;
			Joints[8].joint_dof[0] = false; Joints[8].joint_dof[1] = true; Joints[8].joint_dof[2] = false;
			Joints[8].params_length = 1;
			Joints[8].params_index = new int[1]; Joints[8].params_type = new int[1];
			Joints[8].params_index[0] = 17; Joints[8].params_type[0] = dof_type(y_axis_rotate);
			Joints[8].parent_joint_index = 7;
		}

		{
			Joints[9].joint_name = "MiddleSite";
			Joints[9].joint_index = 9;
			Joints[9].GlobalInitPosition << 6.95019f, 185.733f, -7.207f, 1;
			Joints[9].HasChild = false;
			Joints[9].joint_dof[0] = false; Joints[9].joint_dof[1] = false; Joints[9].joint_dof[2] = false;
			Joints[9].params_length = 0;
			Joints[9].params_index = NULL; Joints[9].params_type = NULL;
			Joints[9].parent_joint_index = 8;
		}
	}
	//Pinkey
	{
		{
			Joints[10].joint_name = "PinkeyLower";
			Joints[10].joint_index = 10;
			Joints[10].GlobalInitPosition << -28.85f, 86.4131f, -0.163318f, 1;
			Joints[10].ChildGlobalInitPosition << -40.1235f, 115.175f, 0.380951f, 1;
			Joints[10].joint_dof[0] = false; Joints[10].joint_dof[1] = true; Joints[10].joint_dof[2] = true;
			Joints[10].params_length = 2;
			Joints[10].params_index = new int[2]; Joints[10].params_type = new int[2];
			Joints[10].params_index[0] = 22; Joints[10].params_type[0] = dof_type(y_axis_rotate);
			Joints[10].params_index[1] = 23; Joints[10].params_type[1] = dof_type(z_axis_rotate);
			Joints[10].parent_joint_index = 1;
		}
		{
			Joints[11].joint_name = "PinkeyMiddle";
			Joints[11].joint_index = 11;
			Joints[11].GlobalInitPosition << -40.1235f, 115.175f, 0.380951f, 1;
			Joints[11].ChildGlobalInitPosition << -49.0937f, 136.986f, -0.0051192f, 1;
			Joints[11].joint_dof[0] = false; Joints[11].joint_dof[1] = true; Joints[11].joint_dof[2] = false;
			Joints[11].params_length = 1;
			Joints[11].params_index = new int[1]; Joints[11].params_type = new int[1];
			Joints[11].params_index[0] = 24; Joints[11].params_type[0] = dof_type(y_axis_rotate);
			Joints[11].parent_joint_index = 10;
		}

		{
			Joints[12].joint_name = "PinkeyTop";
			Joints[12].joint_index = 12;
			Joints[12].GlobalInitPosition << -49.0937f, 136.986f, -0.0051192f, 1;
			Joints[12].ChildGlobalInitPosition << -55.3593f, 151.231f, -0.197348f, 1;
			Joints[12].joint_dof[0] = false; Joints[12].joint_dof[1] = true; Joints[12].joint_dof[2] = false;
			Joints[12].params_length = 1;
			Joints[12].params_index = new int[1]; Joints[12].params_type = new int[1];
			Joints[12].params_index[0] = 25; Joints[12].params_type[0] = dof_type(y_axis_rotate);
			Joints[12].parent_joint_index = 11;
		}

		{
			Joints[13].joint_name = "PinkeySite";
			Joints[13].joint_index = 13;
			Joints[13].GlobalInitPosition << -55.3593f, 151.231f, -0.197348f, 1;
			Joints[13].HasChild = false;
			Joints[13].joint_dof[0] = false; Joints[13].joint_dof[1] = false; Joints[13].joint_dof[2] = false;
			Joints[13].params_length = 0;
			Joints[13].params_index = NULL; Joints[13].params_type = NULL;
			Joints[13].parent_joint_index = 12;
		}
	}

	//ring
	{
		{
			Joints[14].joint_name = "RingLower";
			Joints[14].joint_index = 14;
			Joints[14].GlobalInitPosition << -11.2335f, 96.4782f, -3.66419f, 1;
			Joints[14].ChildGlobalInitPosition << -16.3215f, 130.53f, -7.34456f, 1;
			Joints[14].joint_dof[0] = false; Joints[14].joint_dof[1] = true; Joints[14].joint_dof[2] = true;
			Joints[14].params_length = 2;
			Joints[14].params_index = new int[2]; Joints[14].params_type = new int[2];
			Joints[14].params_index[0] = 18; Joints[14].params_type[0] = dof_type(y_axis_rotate);
			Joints[14].params_index[1] = 19; Joints[14].params_type[1] = dof_type(z_axis_rotate);
			Joints[14].parent_joint_index = 1;
		}

		{
			Joints[15].joint_name = "RingMiddle";
			Joints[15].joint_index = 15;
			Joints[15].GlobalInitPosition << -16.3215f, 130.53f, -7.34456f, 1;
			Joints[15].ChildGlobalInitPosition << -21.4588f, 160.548f, -8.54878f, 1;
			Joints[15].joint_dof[0] = false; Joints[15].joint_dof[1] = true; Joints[15].joint_dof[2] = false;
			Joints[15].params_length = 1;
			Joints[15].params_index = new int[1]; Joints[15].params_type = new int[0];
			Joints[15].params_index[0] = 20;  Joints[15].params_type[0] = dof_type(y_axis_rotate);
			Joints[15].parent_joint_index = 14;
		}

		{
			Joints[16].joint_name = "RingTop";
			Joints[16].joint_index = 16;
			Joints[16].GlobalInitPosition << -21.4588f, 160.548f, -8.54878f, 1;
			Joints[16].ChildGlobalInitPosition << -23.8667f, 178.005f, -9.97166f, 1;
			Joints[16].joint_dof[0] = false; Joints[16].joint_dof[1] = true; Joints[16].joint_dof[2] = false;
			Joints[16].params_length = 1;
			Joints[16].params_index = new int[1]; Joints[16].params_type = new int[1];
			Joints[16].params_index[0] = 21; Joints[16].params_type[0] = dof_type(y_axis_rotate);
			Joints[16].parent_joint_index = 15;
		}

		{
			Joints[17].joint_name = "RingSite";
			Joints[17].joint_index = 17;
			Joints[17].GlobalInitPosition << -23.8667f, 178.005f, -9.97166f, 1;
			Joints[17].HasChild = false;
			Joints[17].joint_dof[0] = false; Joints[17].joint_dof[1] = false; Joints[17].joint_dof[2] = false;
			Joints[17].params_length = 0;
			Joints[17].params_index = NULL; Joints[17].params_type = NULL;
			Joints[17].parent_joint_index = 16;
		}
	}

	//thumb

	{
		{
			Joints[18].joint_name = "ThumbLower";
			Joints[18].joint_index = 18;
			Joints[18].GlobalInitPosition << 42.7179f, 28.8392f, -1.60427f, 1;
			Joints[18].ChildGlobalInitPosition << 59.2979f, 56.6487f, 1.57589f, 1;
			Joints[18].joint_dof[0] = true; Joints[18].joint_dof[1] = true; Joints[18].joint_dof[2] = true;
			Joints[18].params_length = 2;
			Joints[18].params_index = new int[2];  Joints[18].params_type = new int[2];
			Joints[18].params_index[0] = 6; Joints[18].params_type[0] = dof_type(y_axis_rotate);
			Joints[18].params_index[1] = 7; Joints[18].params_type[1] = dof_type(z_axis_rotate);
			Joints[18].parent_joint_index = 1;
		}
		{
			Joints[19].joint_name = "ThumbMiddle";
			Joints[19].joint_index = 19;
			Joints[19].GlobalInitPosition << 59.2979f, 56.6487f, 1.57589f, 1;
			Joints[19].ChildGlobalInitPosition << 81.6844f, 85.0783f, 3.81609f, 1;
			Joints[19].joint_dof[0] = false; Joints[19].joint_dof[1] = true; Joints[19].joint_dof[2] = false;
			Joints[19].params_length = 1;
			Joints[19].params_index = new int[1]; Joints[19].params_type = new int[1];
			Joints[19].params_index[0] = 8; Joints[19].params_type[0] = dof_type(z_axis_rotate);
			Joints[19].parent_joint_index = 18;
		}

		{
			Joints[20].joint_name = "ThumbTop";
			Joints[20].joint_index = 20;
			Joints[20].GlobalInitPosition << 81.6844f, 85.0783f, 3.81609f, 1;
			Joints[20].ChildGlobalInitPosition << 95.188f, 99.7447f, 8.79695f, 1;
			Joints[20].joint_dof[0] = false; Joints[20].joint_dof[1] = true; Joints[20].joint_dof[2] = false;
			Joints[20].params_length = 1;
			Joints[20].params_index = new int[1]; Joints[20].params_type = new int[1];
			Joints[20].params_index[0] = 9; Joints[20].params_type[0] = dof_type(z_axis_rotate);
			Joints[20].parent_joint_index = 19;
		}

		{
			Joints[21].joint_name = "ThumbSite";
			Joints[21].joint_index = 21;
			Joints[21].GlobalInitPosition << 95.188f, 99.7447f, 8.79695f, 1;
			Joints[21].HasChild = false;
			Joints[21].joint_dof[0] = false; Joints[21].joint_dof[1] = false; Joints[21].joint_dof[2] = false;
			Joints[21].params_length = 0;
			Joints[21].params_index = NULL; Joints[21].params_type = NULL;
			Joints[21].parent_joint_index = 20;
		}
	}


	GlobalPosition << 0, 0, 0, 0;


	load_vertices(".\\model\\new_new_vertes.txt");
	load_faces(".\\model\\newFaces.txt");
	load_weight(".\\model\\newWeights.txt");

	outputImage = cv::Mat::zeros(424, 512, CV_8UC1);

	NumberofParams = 26;
	Params = new float[26]();
	init_Params = new float[26]();
	previous_Params = new float[26]();
	ParamsUpperBound = new int[26]();
	ParamsLowerBound = new int[26]();
	SetParamsBound();

	//Scale对应关系
	//       0      ------>    整体变长
	//       1      ------>    整体变宽
	//       2      ------>    整体变厚
	Hand_scale << 0.9f, 0.9f, 0.9f;

	//Jacobain related 
	Joints_jacobian = Eigen::MatrixXf::Zero(NumofJoints * 3, NumberofParams);
	Solved = false;

	//collision related
	set_collosion();
	create_adjacency_matrix();

	compute_local_coordinate();
	compute_parent_child_transform();
	Updata(Params);
}


void HandModel::set_one_rotation(const Pose& pose, int index)
{
	Eigen::MatrixXf x = Eigen::MatrixXf::Identity(4, 4);
	Eigen::MatrixXf y = Eigen::MatrixXf::Identity(4, 4);
	Eigen::MatrixXf z = Eigen::MatrixXf::Identity(4, 4);

	float cx = cos(pose.x / 180 * PI);
	float sx = sin(pose.x / 180 * PI);

	float cy = cos(pose.y / 180 * PI);
	float sy = sin(pose.y / 180 * PI);

	float cz = cos(pose.z / 180 * PI);
	float sz = sin(pose.z / 180 * PI);

	x(1, 1) = cx; x(2, 2) = cx;
	x(1, 2) = -sx; x(2, 1) = sx;

	y(0, 0) = cy; y(0, 2) = sy;
	y(2, 0) = -sy; y(2, 2) = cy;

	z(0, 0) = cz; z(1, 1) = cz;
	z(0, 1) = -sz; z(1, 0) = sz;

	if (index == 0)
	{
		Joints[index].rotation = y*x*z;
	}
	else
	{
		if (index == 18)
		{
			Joints[index].rotation = x*z*y;
		}
		else
		{
			Joints[index].rotation = x*y*z;
		}
	}

}

void HandModel::compute_local_coordinate() {

	float axisx[3] = { 0.0f,0.0f,0.0f };
	float axisy[3] = { 0.0f,0.0f,0.0f };
	float axisz[3] = { 0.0f,0.0f,1.0f };

	for (int i = 0; i < NumofJoints; ++i)
	{
		if (Joints[i].HasChild)
		{

			axisz[0] = 0.0f; axisz[1] = 0.0f; axisz[2] = 1.0f;

			float position[3] = { Joints[i].GlobalInitPosition(0) ,Joints[i].GlobalInitPosition(1) ,Joints[i].GlobalInitPosition(2) };

			axisx[0] = Joints[i].ChildGlobalInitPosition(0) - Joints[i].GlobalInitPosition(0);
			axisx[1] = Joints[i].ChildGlobalInitPosition(1) - Joints[i].GlobalInitPosition(1);
			axisx[2] = Joints[i].ChildGlobalInitPosition(2) - Joints[i].GlobalInitPosition(2);

			normalize(axisx);
			cross_product(axisx, axisz, axisy);
			normalize(axisy);
			cross_product(axisx, axisy, axisz);
			normalize(axisz);

			Joints[i].local = Eigen::MatrixXf::Zero(4, 4);

			Joints[i].local(0, 0) = axisx[0]; Joints[i].local(1, 0) = axisx[1]; Joints[i].local(2, 0) = axisx[2];
			Joints[i].local(0, 1) = axisy[0]; Joints[i].local(1, 1) = axisy[1]; Joints[i].local(2, 1) = axisy[2];
			Joints[i].local(0, 2) = axisz[0]; Joints[i].local(1, 2) = axisz[1]; Joints[i].local(2, 2) = axisz[2];
			Joints[i].local(0, 3) = position[0]; Joints[i].local(1, 3) = position[1]; Joints[i].local(2, 3) = position[2];
			Joints[i].local(3, 3) = 1.0;
		}
		else
		{
			float position[3] = { Joints[i].GlobalInitPosition(0) ,Joints[i].GlobalInitPosition(1) ,Joints[i].GlobalInitPosition(2) };
			int parent_index = Joints[i].parent_joint_index;
			Joints[i].local = Joints[parent_index].local;
			Joints[i].local(0, 3) = position[0]; Joints[i].local(1, 3) = position[1]; Joints[i].local(2, 3) = position[2];
		}
	}
}

void HandModel::compute_parent_child_transform()
{
	for (int i = 0; i < NumofJoints; ++i)
	{
		int parent_joint_index = Joints[i].parent_joint_index;
		if (parent_joint_index != -1)
		{
			Joints[i].trans = Joints[parent_joint_index].local.inverse()*Joints[i].local;
		}
		else
		{
			Joints[i].trans = Joints[i].local;
		}
	}
}

void HandModel::compute_rotation_matrix(float* params)
{

	//Params对应关系
	//       0       ------>    wrist_T_x    //全局平移
	//       1       ------>    wrist_T_y    //全局平移
	//       2       ------>    wrist_T_z    //全局平移
	//       3       ------>    wrist_R_x
	//       4       ------>    wrist_R_y
	//       5       ------>    wrist_R_z
	//       6       ------>    Thumb_Low_R_y
	//       7       ------>    Thumb_Low_R_z
	//       8       ------>    Thumb_mid_R_y    //这里注意了，是z不是y了
	//       9       ------>    Thumb_top_R_y    //这里注意了，是z不是y了
	//       10      ------>    Index_Low_R_y
	//       11      ------>    Index_Low_R_z
	//       12      ------>    Index_mid_R_y
	//       13      ------>    Index_top_R_y
	//       14      ------>    Middle_Low_R_y
	//       15      ------>    Middle_Low_R_z
	//       16      ------>    Middle_mid_R_y
	//       17      ------>    Middle_top_R_y
	//       18      ------>    Ring_Low_R_y
	//       19      ------>    Ring_Low_R_z
	//       20      ------>    Ring_mid_R_y
	//       21      ------>    Ring_top_R_y
	//       22      ------>    Pinkey_Low_R_y
	//       23      ------>    Pinkey_Low_R_z
	//       24      ------>    Pinkey_mid_R_y
	//       25      ------>    Pinkey_top_R_y

	GlobalPosition(0) = params[0];
	GlobalPosition(1) = params[1];
	GlobalPosition(2) = params[2];
	GlobalPosition(3) = 1;

	Pose p_wrist(params[3], params[4], params[5]);
	set_one_rotation(p_wrist, 0);

	//thumb
	Pose p_thumb_lower(0, params[6], params[7]);
	Pose p_thumb_middle(0, 0, params[8]);    //这里注意了，是z不是y了
	Pose p_thumb_top(0, 0, params[9]);      //这里注意了，是z不是y了
	set_one_rotation(p_thumb_lower, 18);
	set_one_rotation(p_thumb_middle, 19);
	set_one_rotation(p_thumb_top, 20);

	//index
	Pose p_pinkey_lower(0, params[10], params[11]);
	Pose p_pinkey_middle(0, params[12], 0);
	Pose p_pinkey_top(0, params[13], 0);
	set_one_rotation(p_pinkey_lower, 2);
	set_one_rotation(p_pinkey_middle, 3);
	set_one_rotation(p_pinkey_top, 4);

	//middle
	Pose p_ring_lower(0, params[14], params[15]);
	Pose p_ring_middle(0, params[16], 0);
	Pose p_ring_top(0, params[17], 0);
	set_one_rotation(p_ring_lower, 6);
	set_one_rotation(p_ring_middle, 7);
	set_one_rotation(p_ring_top, 8);

	//ring
	Pose p_middle_lower(0, params[18], params[19]);
	Pose p_middle_middle(0, params[20], 0);
	Pose p_middle_top(0, params[21], 0);
	set_one_rotation(p_middle_lower, 14);
	set_one_rotation(p_middle_middle, 15);
	set_one_rotation(p_middle_top, 16);

	//pinkey
	Pose p_index_lower(0, params[22], params[23]);
	Pose p_index_middle(0, params[24], 0);
	Pose p_index_top(0, params[25], 0);
	set_one_rotation(p_index_lower, 10);
	set_one_rotation(p_index_middle, 11);
	set_one_rotation(p_index_top, 12);
}

void HandModel::compute_global_matrix()
{
	//这里的Scaling是在Wrist局部坐标系下进行缩放，然后再将缩放后的坐标转换到世界坐标系下，然后在世界坐标系中进行Params[0~2]的平移
	Eigen::Matrix<float, 4, 4> Scaling = Eigen::MatrixXf::Zero(4, 4);
	Scaling(0, 0) = Hand_scale(0);
	Scaling(1, 1) = Hand_scale(1);
	Scaling(2, 2) = Hand_scale(2);
	Scaling(3, 3) = 1;

	Joints[0].global = Joints[0].local*Scaling*Joints[0].rotation;

	for (int i = 0; i < NumofJoints; ++i)
	{
		int parent_joint_index = Joints[i].parent_joint_index;
		if (parent_joint_index != -1)
		{
			Joints[i].global = Joints[parent_joint_index].global*Joints[i].trans*Joints[i].rotation;
		}
	}
}


void HandModel::Updata_Joints()
{
	Eigen::Vector4f handbase = Joints[1].global*Joints[1].local.inverse()*Joints[1].GlobalInitPosition;
	//updata joints
	for (int i = 0; i < NumofJoints; ++i)
	{
		Joints[i].CorrespondingPosition << Joints[i].global*Joints[i].local.inverse()*Joints[i].GlobalInitPosition + GlobalPosition - handbase;
	}

	Joint_matrix.setZero();
	for (int i = 0; i < NumofJoints; ++i)
	{
		Joint_matrix(i, 0) = Joints[i].CorrespondingPosition(0);
		Joint_matrix(i, 1) = Joints[i].CorrespondingPosition(1);
		Joint_matrix(i, 2) = Joints[i].CorrespondingPosition(2);
	}
}

void HandModel::Updata_axis()
{
	Eigen::Vector4f handbase = Joints[1].global*Joints[1].local.inverse()*Joints[1].GlobalInitPosition;

	//updata axis  这里的axis指的是绕的哪个轴旋转的axis的更新，对于不饶旋转的轴，可以不更新，比如: joint[1、5、9、13、17]这五根手指的low 
	//
	for (int i = 0; i < NumofJoints; ++i)
	{
		if (i != 0 && i != 2 && i != 6 && i != 10 && i != 14 && i != 18)
		{
			Joints[i].CorrespondingAxis[0] << Joints[i].global*Joints[i].dof_axis[0] + GlobalPosition - handbase;
			Joints[i].CorrespondingAxis[1] << Joints[i].global*Joints[i].dof_axis[1] + GlobalPosition - handbase;
			Joints[i].CorrespondingAxis[2] << Joints[i].global*Joints[i].dof_axis[2] + GlobalPosition - handbase;
		}
		else
		{
			if (i == 0)
			{
				//y*x*z;         ==> y轴不变，x轴左乘y轴旋转，z轴完全旋转
				Pose pose(Params[3], Params[4], Params[5]);
				Eigen::MatrixXf x = Eigen::MatrixXf::Identity(4, 4);
				Eigen::MatrixXf y = Eigen::MatrixXf::Identity(4, 4);
				Eigen::MatrixXf z = Eigen::MatrixXf::Identity(4, 4);

				float cx = cos(pose.x / 180 * PI);
				float sx = sin(pose.x / 180 * PI);

				float cy = cos(pose.y / 180 * PI);
				float sy = sin(pose.y / 180 * PI);

				float cz = cos(pose.z / 180 * PI);
				float sz = sin(pose.z / 180 * PI);

				x(1, 1) = cx; x(2, 2) = cx;
				x(1, 2) = -sx; x(2, 1) = sx;

				y(0, 0) = cy; y(0, 2) = sy;
				y(2, 0) = -sy; y(2, 2) = cy;

				z(0, 0) = cz; z(1, 1) = cz;
				z(0, 1) = -sz; z(1, 0) = sz;


				Joints[i].CorrespondingAxis[0] << Joints[i].local*y*Joints[i].dof_axis[0] + GlobalPosition - handbase;
				Joints[i].CorrespondingAxis[1] << Joints[i].local*Joints[i].dof_axis[1] + GlobalPosition - handbase;
				Joints[i].CorrespondingAxis[2] << Joints[i].local*y*x*Joints[i].dof_axis[2] + GlobalPosition - handbase;
			}
			else
			{
				if (i == 18)
				{
					//x*z*y;   ==>x轴用不到，因此不更新，z本应该左乘x的旋转，但是x的旋转为单位阵，则z不变，y轴则遵循完全旋转
					int parent_joint_index = Joints[i].parent_joint_index;

					Joints[i].CorrespondingAxis[1] << Joints[i].global*Joints[i].dof_axis[1] + GlobalPosition - handbase;
					Joints[i].CorrespondingAxis[2] << Joints[parent_joint_index].global*Joints[i].trans*Joints[i].dof_axis[2] + GlobalPosition - handbase;
				}
				else
				{
					//x*y*z;    ==>x轴不变，但是x轴用不到，因此不更新，y轴左乘x轴旋转，但是x轴不旋转，因此绕x旋转是单位矩阵，因此y轴也不变，z轴完全旋转
					int parent_joint_index = Joints[i].parent_joint_index;

					Joints[i].CorrespondingAxis[1] << Joints[parent_joint_index].global*Joints[i].trans*Joints[i].dof_axis[1] + GlobalPosition - handbase;
					Joints[i].CorrespondingAxis[2] << Joints[i].global*Joints[i].dof_axis[2] + GlobalPosition - handbase;
				}
			}
		}

	}
}

void HandModel::Updata_Vertics()
{
	Eigen::Vector4f handbase = Joints[1].global*Joints[1].local.inverse()*Joints[1].GlobalInitPosition ;

	Eigen::MatrixXf t = Eigen::MatrixXf::Zero(4, NumofVertices);
	Eigen::MatrixXf x = Eigen::MatrixXf::Ones(4, NumofVertices);
	x.block(0, 0, 3, NumofVertices) = Vectices.block(0, 0, NumofVertices, 3).transpose();

    #pragma omp parallel
	{
        #pragma omp for
		for (int i = 0; i < NumofJoints; ++i) {
			Eigen::MatrixXf y = Weights.block(0, i, NumofVertices, 1);// 在所有顶点 对于 该关节点的weight
			Eigen::MatrixXf y0 = y.replicate(1, 4);    //分别是行重复1遍，列重复4遍，结果为（num_vertices_，4）这么大小的矩阵
			Eigen::MatrixXf z = Joints[i].global * Joints[i].local.inverse() * x;
            #pragma omp critical(a)
			{
				t = t + z.cwiseProduct(y0.transpose());
			}
		}
	}

	vertices_update_ = t.transpose();

#pragma omp parallel for 
	for (int i = 0; i < vertices_update_.rows(); ++i) {
		vertices_update_(i, 0) += GlobalPosition(0) - handbase(0);
		vertices_update_(i, 1) += GlobalPosition(1) - handbase(1);
		vertices_update_(i, 2) += GlobalPosition(2) - handbase(2);
	}
}


void HandModel::Updata(float* params)
{

	compute_rotation_matrix(params);
	compute_global_matrix();

	updata_collosion();
	Judge_Collision();

	Updata_Joints();
	Updata_axis();
	Updata_Vertics();        

	Compute_normal_And_visibel_vertices();    
}


void HandModel::Compute_normal_And_visibel_vertices()
{
	Visible_vertices.clear();
	Visible_vertices_index.clear();
	Visible_vertices_2D.clear();
	Face_norm.clear();

	Vertices_normal.setZero();


#pragma omp parallel
	{
#pragma omp for
		for (int i = 0; i < NumofFaces; ++i)
		{
			Vector3f A, B, C, BA, BC;
			//这里我假设，如果假设错了，那么叉乘时候，就BC*BA变成BA*BC
			//            A
			//          /  \
			//         B — C
			A << vertices_update_(FaceIndex(i, 0), 0), vertices_update_(FaceIndex(i, 0), 1), vertices_update_(FaceIndex(i, 0), 2);
			B << vertices_update_(FaceIndex(i, 1), 0), vertices_update_(FaceIndex(i, 1), 1), vertices_update_(FaceIndex(i, 1), 2);
			C << vertices_update_(FaceIndex(i, 2), 0), vertices_update_(FaceIndex(i, 2), 1), vertices_update_(FaceIndex(i, 2), 2);

			BC << C - B;
			BA << A - B;

			Vector3f nom(BC.cross(BA));

			nom.normalize();
			Face_norm[i] = nom;


			Vertices_normal(FaceIndex(i, 0), 0) += nom(0);
			Vertices_normal(FaceIndex(i, 1), 0) += nom(0);
			Vertices_normal(FaceIndex(i, 2), 0) += nom(0);

			Vertices_normal(FaceIndex(i, 0), 1) += nom(1);
			Vertices_normal(FaceIndex(i, 1), 1) += nom(1);
			Vertices_normal(FaceIndex(i, 2), 1) += nom(1);

			Vertices_normal(FaceIndex(i, 0), 2) += nom(2);
			Vertices_normal(FaceIndex(i, 1), 2) += nom(2);
			Vertices_normal(FaceIndex(i, 2), 2) += nom(2);

		}
	}

	for (int i = 0; i < Vertices_normal.rows(); ++i)
	{
		Vertices_normal.row(i).normalize();

		if (-(Vertices_normal(i, 2)) <= 0)
		{
			Eigen::Vector3f visible_v(vertices_update_(i, 0), vertices_update_(i, 1), vertices_update_(i, 2));
			Visible_vertices.push_back(visible_v);
			Visible_vertices_index.push_back(i);


			Eigen::Vector2i visible_2D;
			visible_2D << (int)(camera->world_to_depth_image(visible_v)(0)), (int)(camera->world_to_depth_image(visible_v)(1));

			Visible_vertices_2D.push_back(visible_2D);
		}
	}

	//cout << "visible vertices find done!" << endl;
}


//=================Jacobain    related    function==============================
void HandModel::Updata_joints_Jacobian()
{
	Joints_jacobian.setZero();

	for (int i = 0; i < NumofJoints; ++i)
	{
		Joints_jacobian.block(i * 3, 0, 3, NumberofParams) = Compute_one_Joint_Jacobian(i);
	}
}

Eigen::MatrixXf HandModel::Compute_one_Joint_Jacobian(int index)
{
	float omiga = 3.141592f / 180.0f;
	Eigen::MatrixXf Jacobain_ = Eigen::MatrixXf::Zero(3, NumberofParams);

	int current_indx = index;
	Eigen::Vector3f current_joint_position(Joints[index].CorrespondingPosition(0), Joints[index].CorrespondingPosition(1), Joints[index].CorrespondingPosition(2));

	//计算时候会用到的变量
	Eigen::Vector3f axis_base_position;
	Eigen::Vector3f x_axis_position;
	Eigen::Vector3f y_axis_position;
	Eigen::Vector3f z_axis_position;

	Eigen::Vector3f w_x, w_y, w_z;
	Eigen::Vector3f S;
	Eigen::Vector3f result;

	while (current_indx >= 0)
	{
		axis_base_position << Joints[current_indx].CorrespondingPosition(0), Joints[current_indx].CorrespondingPosition(1), Joints[current_indx].CorrespondingPosition(2);

		x_axis_position << Joints[current_indx].CorrespondingAxis[0](0), Joints[current_indx].CorrespondingAxis[0](1), Joints[current_indx].CorrespondingAxis[0](2);
		y_axis_position << Joints[current_indx].CorrespondingAxis[1](0), Joints[current_indx].CorrespondingAxis[1](1), Joints[current_indx].CorrespondingAxis[1](2);
		z_axis_position << Joints[current_indx].CorrespondingAxis[2](0), Joints[current_indx].CorrespondingAxis[2](1), Joints[current_indx].CorrespondingAxis[2](2);


		w_x << (x_axis_position - axis_base_position);
		w_y << (y_axis_position - axis_base_position);
		w_z << (z_axis_position - axis_base_position);

		w_x.normalize();
		w_y.normalize();
		w_z.normalize();

		S << (current_joint_position - axis_base_position);

		int params_len = Joints[current_indx].params_length;
		for (int idx = 0; idx < params_len; idx++)
		{
			int params_idx = Joints[current_indx].params_index[idx];

			switch (Joints[current_indx].params_type[idx])
			{
			case dof_type(x_axis_rotate): {
				result << omiga*w_x.cross(S);
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(y_axis_rotate): {
				result << omiga*w_y.cross(S);
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(z_axis_rotate): {
				result << omiga*w_z.cross(S);
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(x_axis_trans): {
				result << 1, 0, 0;
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(y_axis_trans): {
				result << 0, 1, 0;
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(z_axis_trans): {
				result << 0, 0, 1;
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			}

		}

		current_indx = Joints[current_indx].parent_joint_index;
	}

	return Jacobain_;
}

void HandModel::Updata_Jacobian_correspond(std::vector<int> & cor)
{
	int NUMofCorrespond = cor.size();

	jacobian_correspond = Eigen::MatrixXf::Zero(NUMofCorrespond * 3, NumberofParams);
	//cout << jacobian.rows() << "," << jacobian.cols();

	for (int cor_id = 0; cor_id < NUMofCorrespond; ++cor_id)
	{
		int vert_idx = Visible_vertices_index[cor[cor_id]];
		jacobian_correspond.block(cor_id * 3, 0, 3, NumberofParams) = Compute_one_Vertex_Jacobain(vert_idx);
	}
}

Eigen::MatrixXf HandModel::Compute_one_Vertex_Jacobain(int index)
{
	float omiga = 3.141592f / 180.0f;
	Eigen::MatrixXf Jacobain_ = Eigen::MatrixXf::Zero(3, NumberofParams);

	int vert_idx = index;

	Eigen::Vector3f vertice_position(vertices_update_(vert_idx, 0), vertices_update_(vert_idx, 1), vertices_update_(vert_idx, 2));

	//计算时候会用到的变量
	Eigen::Vector3f axis_base_position;
	Eigen::Vector3f x_axis_position;
	Eigen::Vector3f y_axis_position;
	Eigen::Vector3f z_axis_position;

	Eigen::Vector3f w_x, w_y, w_z;
	Eigen::Vector3f S;
	Eigen::Vector3f result;

	for (int joint_idx = 0; joint_idx < NumofJoints; joint_idx++)
	{
		float weight = Weights(vert_idx, joint_idx);
		if (weight > 0.0f)
		{
			//do some thing follow the kanimatic chains
			int current_joint_idx = joint_idx;

			while (current_joint_idx >= 0)
			{
				axis_base_position<<Joints[current_joint_idx].CorrespondingPosition(0), Joints[current_joint_idx].CorrespondingPosition(1), Joints[current_joint_idx].CorrespondingPosition(2);

				x_axis_position<<Joints[current_joint_idx].CorrespondingAxis[0](0), Joints[current_joint_idx].CorrespondingAxis[0](1), Joints[current_joint_idx].CorrespondingAxis[0](2);
				y_axis_position<<Joints[current_joint_idx].CorrespondingAxis[1](0), Joints[current_joint_idx].CorrespondingAxis[1](1), Joints[current_joint_idx].CorrespondingAxis[1](2);
				z_axis_position<<Joints[current_joint_idx].CorrespondingAxis[2](0), Joints[current_joint_idx].CorrespondingAxis[2](1), Joints[current_joint_idx].CorrespondingAxis[2](2);


				w_x << (x_axis_position - axis_base_position);
				w_y << (y_axis_position - axis_base_position);
				w_z << (z_axis_position - axis_base_position);

				w_x.normalize();
				w_y.normalize();
				w_z.normalize();

				S << (vertice_position - axis_base_position);

				int params_len = Joints[current_joint_idx].params_length;
				for (int idx = 0; idx < params_len; idx++)
				{
					int params_idx = Joints[current_joint_idx].params_index[idx];

					switch (Joints[current_joint_idx].params_type[idx])
					{
					case dof_type(x_axis_rotate): {
						result << omiga*w_x.cross(S);
						Jacobain_(0, params_idx) += weight*result(0);
						Jacobain_(1, params_idx) += weight*result(1);
						Jacobain_(2, params_idx) += weight*result(2);
						break;
					}
					case dof_type(y_axis_rotate): {
						result << omiga*w_y.cross(S);
						Jacobain_(0, params_idx) += weight*result(0);
						Jacobain_(1, params_idx) += weight*result(1);
						Jacobain_(2, params_idx) += weight*result(2);
						break;
					}
					case dof_type(z_axis_rotate): {
						result << omiga*w_z.cross(S);
						Jacobain_(0, params_idx) += weight*result(0);
						Jacobain_(1, params_idx) += weight*result(1);
						Jacobain_(2, params_idx) += weight*result(2);
						break;
					}
					case dof_type(x_axis_trans): {
						result << 1, 0, 0;
						Jacobain_(0, params_idx) += weight*result(0);
						Jacobain_(1, params_idx) += weight*result(1);
						Jacobain_(2, params_idx) += weight*result(2);
						break;
					}
					case dof_type(y_axis_trans): {
						result << 0, 1, 0;
						Jacobain_(0, params_idx) += weight*result(0);
						Jacobain_(1, params_idx) += weight*result(1);
						Jacobain_(2, params_idx) += weight*result(2);
						break;
					}
					case dof_type(z_axis_trans): {
						result << 0, 0, 1;
						Jacobain_(0, params_idx) += weight*result(0);
						Jacobain_(1, params_idx) += weight*result(1);
						Jacobain_(2, params_idx) += weight*result(2);
						break;
					}
					}

				}
				current_joint_idx = Joints[current_joint_idx].parent_joint_index;
			}
		}
	}

	return Jacobain_;
}


void HandModel::show_Params()
{
	//Params对应关系
	//       0       ------>    wrist_T_x    //全局平移
	//       1       ------>    wrist_T_y    //全局平移
	//       2       ------>    wrist_T_z    //全局平移
	//       3       ------>    wrist_R_x
	//       4       ------>    wrist_R_y
	//       5       ------>    wrist_R_z
	//       6       ------>    Thumb_Low_R_y
	//       7       ------>    Thumb_Low_R_z
	//       8       ------>    Thumb_mid_R_y    //这里注意了，是z不是y了
	//       9       ------>    Thumb_top_R_y    //这里注意了，是z不是y了
	//       10      ------>    Index_Low_R_y
	//       11      ------>    Index_Low_R_z
	//       12      ------>    Index_mid_R_y
	//       13      ------>    Index_top_R_y
	//       14      ------>    Middle_Low_R_y
	//       15      ------>    Middle_Low_R_z
	//       16      ------>    Middle_mid_R_y
	//       17      ------>    Middle_top_R_y
	//       18      ------>    Ring_Low_R_y
	//       19      ------>    Ring_Low_R_z
	//       20      ------>    Ring_mid_R_y
	//       21      ------>    Ring_top_R_y
	//       22      ------>    Pinkey_Low_R_y
	//       23      ------>    Pinkey_Low_R_z
	//       24      ------>    Pinkey_mid_R_y
	//       25      ------>    Pinkey_top_R_y
	cout << "Params is  : " << endl << "--------------------" << endl;
	cout << "===>>>  Global_P :  " << (int)Params[0] << "   " << (int)Params[1] << "   " << (int)Params[2] << endl;
	cout << "===>>>  Global_R :  " << (int)Params[3] << "   " << (int)Params[4] << "   " << (int)Params[5] << endl;
	cout << endl;
	cout << "===>>>  Thumb        Index        Middle        Ring        Pinkey        " << endl;  
	cout << "R_low   " << (int)Params[6] << "         " << (int)Params[10] << "         " << (int)Params[14] << "         " << (int)Params[18] << "         " << (int)Params[22] << "       " << endl;
	cout << "B_low   " << (int)Params[7] << "         " << (int)Params[11] << "         " << (int)Params[15] << "         " << (int)Params[19] << "         " << (int)Params[23] << "       " << endl;
	cout << "R_mid   " << (int)Params[8] << "         " << (int)Params[12] << "         " << (int)Params[16] << "         " << (int)Params[20] << "         " << (int)Params[24] << "       " << endl;
	cout << "R_top   " << (int)Params[9] << "         " << (int)Params[13] << "         " << (int)Params[17] << "         " << (int)Params[21] << "         " << (int)Params[25] << "       " << endl;
	cout << endl<< "--------------------" << endl;

}

void HandModel::show_initParams()
{
	//Params对应关系
	//       0       ------>    wrist_T_x    //全局平移
	//       1       ------>    wrist_T_y    //全局平移
	//       2       ------>    wrist_T_z    //全局平移
	//       3       ------>    wrist_R_x
	//       4       ------>    wrist_R_y
	//       5       ------>    wrist_R_z
	//       6       ------>    Thumb_Low_R_y
	//       7       ------>    Thumb_Low_R_z
	//       8       ------>    Thumb_mid_R_y    //这里注意了，是z不是y了
	//       9       ------>    Thumb_top_R_y    //这里注意了，是z不是y了
	//       10      ------>    Index_Low_R_y
	//       11      ------>    Index_Low_R_z
	//       12      ------>    Index_mid_R_y
	//       13      ------>    Index_top_R_y
	//       14      ------>    Middle_Low_R_y
	//       15      ------>    Middle_Low_R_z
	//       16      ------>    Middle_mid_R_y
	//       17      ------>    Middle_top_R_y
	//       18      ------>    Ring_Low_R_y
	//       19      ------>    Ring_Low_R_z
	//       20      ------>    Ring_mid_R_y
	//       21      ------>    Ring_top_R_y
	//       22      ------>    Pinkey_Low_R_y
	//       23      ------>    Pinkey_Low_R_z
	//       24      ------>    Pinkey_mid_R_y
	//       25      ------>    Pinkey_top_R_y
	cout << "init_Params is  : " << endl << "--------------------" << endl;
	cout << "===>>>  Global_P :  " << (int)init_Params[0] << "   " << (int)init_Params[1] << "   " << (int)init_Params[2] << endl;
	cout << "===>>>  Global_R :  " << (int)init_Params[3] << "   " << (int)init_Params[4] << "   " << (int)init_Params[5] << endl;
	cout << endl;
	cout << "===>>>  Thumb        Index        Middle        Ring        Pinkey        " << endl;
	cout << "R_low   " << (int)init_Params[6] << "         " << (int)init_Params[10] << "         " << (int)init_Params[14] << "         " << (int)init_Params[18] << "         " << (int)init_Params[22] << "       " << endl;
	cout << "B_low   " << (int)init_Params[7] << "         " << (int)init_Params[11] << "         " << (int)init_Params[15] << "         " << (int)init_Params[19] << "         " << (int)init_Params[23] << "       " << endl;
	cout << "R_mid   " << (int)init_Params[8] << "         " << (int)init_Params[12] << "         " << (int)init_Params[16] << "         " << (int)init_Params[20] << "         " << (int)init_Params[24] << "       " << endl;
	cout << "R_top   " << (int)init_Params[9] << "         " << (int)init_Params[13] << "         " << (int)init_Params[17] << "         " << (int)init_Params[21] << "         " << (int)init_Params[25] << "       " << endl;
	cout << endl << "--------------------" << endl;

}

void HandModel::MoveToDownSampleCorrespondingVertices(int itr,pcl::PointCloud<pcl::PointXYZ>& p, std::vector<int>& cor,int *idx_img,bool has_glove)
{
	int NumofCorrespond = p.points.size();
	Eigen::VectorXf e = Eigen::VectorXf::Zero(3 * NumofCorrespond, 1);

	for (int i = 0; i < NumofCorrespond; i++)
	{
		e(i * 3 + 0) = p.points[i].x - Visible_vertices[cor[i]].x();
		e(i * 3 + 1) = p.points[i].y - Visible_vertices[cor[i]].y();
		e(i * 3 + 2) = p.points[i].z - Visible_vertices[cor[i]].z();
	}

	Updata_Jacobian_correspond(cor);

	//cloud Point Limited
	int omiga_3D = 1;
	Eigen::MatrixXf J = jacobian_correspond;
	Eigen::MatrixXf Jt = jacobian_correspond.transpose();

	//silhouette Limited
	int omiga_2D = 1;
	Eigen::VectorXf e_sil;
	Eigen::MatrixXf J_sil = Compute_Silhouette_Limited(e_sil, idx_img);


	if (itr > 15)
	{
		Solved = true;
		float e_3D = 0;
		float e_2D = 0;

		for (int i = 0; i < e.rows() / 3; ++i)
		{
			e_3D += sqrt(e(i * 3 + 0, 0)*e(i * 3 + 0, 0) + e(i * 3 + 1, 0)*e(i * 3 + 1, 0) + e(i * 3 + 2, 0)*e(i * 3 + 2, 0));
		}
		for (int i = 0; i < e_sil.rows() / 2; ++i)
		{
			e_2D += sqrt(e_sil(i * 2 + 0, 0)*e_sil(i * 2 + 0, 0) + e_sil(i * 2 + 1, 0)*e_sil(i * 2 + 1, 0));
		}

		float e_final = e_3D + e_2D;
		cout << " final e is : " << e_final << endl
			<< "---------------》 e_3D  is : " << e.norm() << endl
			<< "---------------》 e_2D  is : " << e_sil.norm() << endl;
		show_initParams();
		show_Params();

		if (e_final < 3500)
		{
			this->track_failure = false;

			//save Params as previous Params
			for (int i = 0; i < NumberofParams; ++i) previous_Params[i] = Params[i];

			//save Joint Position as temporal information
			if (temporal_position.size() == 2)
			{
				temporal_position.pop();
				temporal_position.push(Joint_matrix);
			}
			else
			{
				temporal_position.push(Joint_matrix);
			}
		}
		else
		{
			this->track_failure = true;

			for (int i = 0; i < NumberofParams; ++i) Params[i] = init_Params[i];
			//if track fail
			//reset previous_params
			for (int i = 0; i < NumberofParams; ++i) previous_Params[i] = 0;

			//reset temporal information
			int temporal_size = temporal_position.size();
			for (int i = 0; i < temporal_size; ++i) temporal_position.pop();
		}

		return;
	}
	else
	{
		Solved = false;
	}


	//joint Limited
	int omiga_joint_limited = 50;
	Eigen::VectorXf e_limit;
	Eigen::MatrixXf J_limit = Compute_joint_Limited(e_limit,has_glove);

	//collision Limied
	int omiga_collision = 50;
	Eigen::VectorXf e_col;
	Eigen::MatrixXf J_col = Compute_Collision_Limit(e_col);

	//Temporal Limited
	int omiga_Temporal_1 = 0.1;
	int omiga_Temporal_2 = 0.1;
	Eigen::VectorXf e_temporal_1;
	Eigen::MatrixXf J_temporal_1 = Compute_Temporal_Limited(e_temporal_1,true);

	Eigen::VectorXf e_temporal_2;
	Eigen::MatrixXf J_temporal_2 = Compute_Temporal_Limited(e_temporal_2,false);


	//Damping
	Eigen::MatrixXf D = Compute_Damping_Limited();

	

	Eigen::VectorXf dAngles = Eigen::VectorXf::Zero(NumberofParams, 1);


	MatrixXf JtJ = omiga_3D*Jt*J
		+ omiga_2D*J_sil.transpose()*J_sil
		+ omiga_joint_limited*J_limit.transpose()*J_limit
		+ omiga_collision*J_col.transpose()*J_col
		+ omiga_Temporal_1*J_temporal_1.transpose()*J_temporal_1
		+ omiga_Temporal_2*J_temporal_2.transpose()*J_temporal_2
		+ D;

	VectorXf JTe = omiga_3D*Jt*e
		+ omiga_2D*J_sil.transpose()*e_sil
		+ omiga_joint_limited*J_limit.transpose()*e_limit
		+ omiga_collision*J_col.transpose()*e_col
		+ omiga_Temporal_1*J_temporal_1.transpose()*e_temporal_1
		+ omiga_Temporal_2*J_temporal_2.transpose()*e_temporal_2;


	if (itr < 2)
	{
		for (int i = 6; i < NumberofParams; ++i) JtJ.col(i).setZero();

		for (int i = 6; i < NumberofParams; ++i)
		{
			JtJ.row(i).setZero();
			JTe.row(i).setZero();
		}
	}


	dAngles = JtJ.colPivHouseholderQr().solve(JTe);


	for (int i = 0; i < NumberofParams; i++)
		Params[i] += dAngles(i);


	Updata(Params);

}


//other Limited
MatrixXf HandModel::Compute_joint_Limited(Eigen::VectorXf & e_limit, bool has_glove)
{
	MatrixXf J_limit = MatrixXf::Zero(26, 26);

	e_limit = Eigen::VectorXf::Zero(26, 1);


	if (has_glove)
	{
		for (int i = 8; i < NumberofParams; i++)
		{

			if (i != 13 && i != 17 && i != 21 && i != 25)
			{
				if (i == 11 || i == 15 || i == 19 || i == 23)
				{
					float q_max = ParamsUpperBound[i];
					float q_min = ParamsLowerBound[i];
					if (Params[i] > q_max) {
						//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
						e_limit(i) = (q_max - Params[i]) - std::numeric_limits<float>::epsilon();
						J_limit(i, i) = 1;
					}
					else if (Params[i] < q_min) {
						//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
						e_limit(i) = (q_min - Params[i]) + std::numeric_limits<float>::epsilon();
						J_limit(i, i) = 1;
					}
					else {
						J_limit(i, i) = 0;
						e_limit(i) = 0;
					}
				}
				else
				{
					float q_max = (init_Params[i] + 20) > ParamsUpperBound[i] ? ParamsUpperBound[i] : (init_Params[i] + 20);
					float q_min = (init_Params[i] - 20) < ParamsLowerBound[i] ? ParamsLowerBound[i] : (init_Params[i] - 20);
					if (Params[i] > q_max) {
						//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
						e_limit(i) = (q_max - Params[i]) - std::numeric_limits<float>::epsilon();
						J_limit(i, i) = 1;
					}
					else if (Params[i] < q_min) {
						//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
						e_limit(i) = (q_min - Params[i]) + std::numeric_limits<float>::epsilon();
						J_limit(i, i) = 1;
					}
					else {
						J_limit(i, i) = 0;
						e_limit(i) = 0;
					}
				}
			}
			else
			{
				float q_max = (init_Params[i] + 20) > ParamsUpperBound[i] ? ParamsUpperBound[i] : (init_Params[i] + 20);
				float q_min = Params[i - 1]*0.4f;
				if (Params[i] > q_max) {
					//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
					e_limit(i) = (q_max - Params[i]) - std::numeric_limits<float>::epsilon();
					J_limit(i, i) = 1;
				}
				else if (Params[i] < q_min) {
					//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
					e_limit(i) = (q_min - Params[i]) + std::numeric_limits<float>::epsilon();
					J_limit(i, i) = 1;
				}
				else {
					J_limit(i, i) = 0;
					e_limit(i) = 0;
				}
			}
		}
	}
	else
	{
		for (int i = 6; i < NumberofParams; i++)
		{
			if (i != 13 && i != 17 && i != 21 && i != 25)
			{
				float q_max = ParamsUpperBound[i];
				float q_min = ParamsLowerBound[i];
				if (Params[i] > q_max) {
					//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
					e_limit(i) = (q_max - Params[i]) - std::numeric_limits<float>::epsilon();
					J_limit(i, i) = 1;
				}
				else if (Params[i] < q_min) {
					//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
					e_limit(i) = (q_min - Params[i]) + std::numeric_limits<float>::epsilon();
					J_limit(i, i) = 1;
				}
				else {
					J_limit(i, i) = 0;
					e_limit(i) = 0;
				}
			}
			else
			{
				float q_max = (Params[i - 1] + 20.0f) > ParamsUpperBound[i] ? ParamsUpperBound[i] : (Params[i - 1] + 20.0f);
				float q_min = Params[i - 1] * 0.5f;
				if (Params[i] > q_max) {
					//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
					e_limit(i) = (q_max - Params[i]) - std::numeric_limits<float>::epsilon();
					J_limit(i, i) = 1;
				}
				else if (Params[i] < q_min) {
					//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
					e_limit(i) = (q_min - Params[i]) + std::numeric_limits<float>::epsilon();
					J_limit(i, i) = 1;
				}
				else {
					J_limit(i, i) = 0;
					e_limit(i) = 0;
				}
			}
		}
	}


	return J_limit;
}

MatrixXf HandModel::Compute_Silhouette_Limited(Eigen::VectorXf & e_sil,int *idx_img)
{
	vector<pair<Eigen::Matrix<float, 2, 26>, Eigen::Vector2f>> outside_silhouette;

	for (int i = 0; i < Visible_vertices_2D.size(); i++)
	{
		Eigen::Vector3f pixel_3D_position(Visible_vertices[i]);
		int vert_idx = Visible_vertices_index[i];
		Eigen::Vector2i pixel_2D_position(Visible_vertices_2D[i]);

		if ((pixel_2D_position(0) >= 0) &&
			(pixel_2D_position(0) <= 512 - 1) &&
			(pixel_2D_position(1) >= 0) &&
			(pixel_2D_position(1) <= 424 - 1))
		{
			Eigen::Vector2i pixel_2D_closest;
			pixel_2D_closest << idx_img[pixel_2D_position(1) * 512 + pixel_2D_position(0)] % 512, idx_img[pixel_2D_position(1) * 512 + pixel_2D_position(0)] / 512;

			float closest_distance = (pixel_2D_closest(0) - pixel_2D_position(0))* (pixel_2D_closest(0) - pixel_2D_position(0)) + (pixel_2D_closest(1) - pixel_2D_position(1))*(pixel_2D_closest(1) - pixel_2D_position(1));

			if (closest_distance > 1)
			{
				//计算J和e
				pair<Eigen::Matrix2Xf, Eigen::Vector2f> J_and_e;

				//先算e
				Eigen::Vector2f e;
				e(0) = (float)pixel_2D_closest(0) - (float)pixel_2D_position(0);
				e(1) = (float)pixel_2D_closest(1) - (float)pixel_2D_position(1);

				J_and_e.second = e;

				//再算J
				Matrix_2x3 J_perspective = camera->projection_jacobian(pixel_3D_position);
				Eigen::Matrix<float, 2, 26> J_2D = MatrixXf::Zero(2, 26);    //J_2D = J_perspective * J_3D

				J_2D = J_perspective*Compute_one_Vertex_Jacobain(vert_idx);

				J_and_e.first = J_2D;

				outside_silhouette.push_back(J_and_e);
			}
		}
	}

	int size = outside_silhouette.size();

	if (size > 0)
	{
		MatrixXf J_sil = MatrixXf::Zero(2 * size, 26);

		e_sil = Eigen::VectorXf::Zero(2 * size, 1);

		for (int i = 0; i < size; i++)
		{
			J_sil.row(i * 2 + 0) = outside_silhouette[i].first.row(0);
			J_sil.row(i * 2 + 1) = outside_silhouette[i].first.row(1);

			e_sil.row(i * 2 + 0) = outside_silhouette[i].second.row(0);
			e_sil.row(i * 2 + 1) = outside_silhouette[i].second.row(1);
		}
		return J_sil;
	}
	else
	{
		//cout << "all rendered pixel are in the silhouette " << endl;

		MatrixXf J_sil = MatrixXf::Zero(1, 26);

		e_sil = Eigen::VectorXf::Zero(1, 1);

		return J_sil;
	}
}

MatrixXf HandModel::Compute_Temporal_Limited(Eigen::VectorXf & e_limit,bool first_order)
{
	Eigen::Matrix<float, 3*22, 26> J_Tem = MatrixXf::Zero(3*22, 26); 
	e_limit = Eigen::VectorXf::Zero(3*22, 1);
	if (temporal_position.size() == 2)
	{
		for (int i = 0; i < NumofJoints; i++)
		{
			if (first_order)
			{
				e_limit(i * 3 + 0) = temporal_position.back()(i, 0) - Joints[i].CorrespondingPosition(0);
				e_limit(i * 3 + 1) = temporal_position.back()(i, 1) - Joints[i].CorrespondingPosition(1);
				e_limit(i * 3 + 2) = temporal_position.back()(i, 2) - Joints[i].CorrespondingPosition(2);
			}
			else
			{
				e_limit(i * 3 + 0) = 2 * temporal_position.back()(i, 0) - temporal_position.front()(i, 0) - Joints[i].CorrespondingPosition(0);
				e_limit(i * 3 + 1) = 2 * temporal_position.back()(i, 1) - temporal_position.front()(i, 1) - Joints[i].CorrespondingPosition(1);
				e_limit(i * 3 + 2) = 2 * temporal_position.back()(i, 2) - temporal_position.front()(i, 2) - Joints[i].CorrespondingPosition(2);
			}


			this->Updata_joints_Jacobian();
		}

		J_Tem = this->Joints_jacobian;
	}

	return J_Tem;
}

MatrixXf HandModel::Compute_Damping_Limited()
{
	Eigen::MatrixXf D = Eigen::MatrixXf::Identity(NumberofParams, NumberofParams);
	int rotate_omiga = 20;
	int abound_omiga = 50;
	//D damping
	{
		D(0, 0) = 1;
		D(1, 1) = 1;
		D(2, 2) = 1;
		D(3, 3) = rotate_omiga;
		D(4, 4) = rotate_omiga;
		D(5, 5) = rotate_omiga;

		D(6, 6) = abound_omiga;
		D(7, 7) = rotate_omiga;
		D(8, 8) = rotate_omiga;
		D(9, 9) = rotate_omiga;

		D(10, 10) = rotate_omiga;
		D(11, 11) = abound_omiga;
		D(12, 12) = rotate_omiga;
		D(13, 13) = rotate_omiga;

		D(14, 14) = rotate_omiga;
		D(15, 15) = abound_omiga;
		D(16, 16) = rotate_omiga;
		D(17, 17) = rotate_omiga;

		D(18, 18) = rotate_omiga;
		D(19, 19) = abound_omiga;
		D(20, 20) = rotate_omiga;
		D(21, 21) = rotate_omiga;

		D(22, 22) = rotate_omiga;
		D(23, 23) = abound_omiga;
		D(24, 24) = rotate_omiga;
		D(25, 25) = rotate_omiga;
	}

	return D;

}

//collision related
void HandModel::set_collosion()
{
	Collision_sphere.clear();
	//index
	{
		//index_low
		{
			Collision index_Low_0;
			index_Low_0.id = 0;
			index_Low_0.adjscent_id[0] = -1; index_Low_0.adjscent_id[1] = 1;
			index_Low_0.init_Position << 0, -2, -0.8f, 1;
			index_Low_0.init_radius = 9.0f;
			index_Low_0.joint_index = 2;
			Collision_sphere.push_back(index_Low_0);

			Collision index_low_1;
			index_low_1.id = 1;
			index_low_1.adjscent_id[0] = 0; index_low_1.adjscent_id[1] = 2;
			index_low_1.init_Position << 11, -2, -0.8f, 1;
			index_low_1.init_radius = 8.5f;
			index_low_1.joint_index = 2;
			Collision_sphere.push_back(index_low_1);

			Collision index_low_2;
			index_low_2.id = 2;
			index_low_2.adjscent_id[0] = 1; index_low_2.adjscent_id[1] = 3;
			index_low_2.init_Position << 20, -1, -0.8f, 1;
			index_low_2.init_radius = 8.2f;
			index_low_2.joint_index = 2;
			Collision_sphere.push_back(index_low_2);
		}

		//index_middle
		{
			Collision index_middle_0;
			index_middle_0.id = 3;
			index_middle_0.adjscent_id[0] = 2; index_middle_0.adjscent_id[1] = 4;
			index_middle_0.init_Position << 0, -1.0f, -0.8f, 1;
			index_middle_0.init_radius = 8.3f;
			index_middle_0.joint_index = 3;
			Collision_sphere.push_back(index_middle_0);

			Collision index_middle_1;
			index_middle_1.id = 4;
			index_middle_1.adjscent_id[0] = 3; index_middle_1.adjscent_id[1] = 5;
			index_middle_1.init_Position << 11, -1.0f, -0.8f, 1;
			index_middle_1.init_radius = 7.3f;
			index_middle_1.joint_index = 3;
			Collision_sphere.push_back(index_middle_1);

			Collision index_middle_2;
			index_middle_2.id = 5;
			index_middle_2.adjscent_id[0] = 4; index_middle_2.adjscent_id[1] = 6;
			index_middle_2.init_Position << 20, -1.0f, -0.8f, 1;
			index_middle_2.init_radius = 7.0f;
			index_middle_2.joint_index = 3;
			Collision_sphere.push_back(index_middle_2);
		}

		//index_top
		{
			Collision index_top_0;
			index_top_0.id = 6;
			index_top_0.adjscent_id[0] = 5; index_top_0.adjscent_id[1] = 7;
			index_top_0.init_Position << 0, -1.0f, -0.8f, 1;
			index_top_0.init_radius = 6.8f;
			index_top_0.joint_index = 4;
			Collision_sphere.push_back(index_top_0);

			Collision index_top_1;
			index_top_1.id = 7;
			index_top_1.adjscent_id[0] = 6; index_top_1.adjscent_id[1] = -1;
			index_top_1.init_Position << 10, -1.0f, -1.0f, 1;
			index_top_1.init_radius = 4.5f;
			index_top_1.joint_index = 4;
			Collision_sphere.push_back(index_top_1);
		}
	}

	//Middle
	{
		//Middle_low
		{
			Collision Middle_low_0;
			Middle_low_0.id = 8;
			Middle_low_0.adjscent_id[0] = 9; Middle_low_0.adjscent_id[1] = -1;
			Middle_low_0.init_Position << 0, -2, -0.3f, 1;
			Middle_low_0.init_radius = 9.0f;
			Middle_low_0.joint_index = 6;
			Collision_sphere.push_back(Middle_low_0);

			Collision Middle_low_1;
			Middle_low_1.id = 9;
			Middle_low_1.adjscent_id[0] = 8; Middle_low_1.adjscent_id[1] = 10;
			Middle_low_1.init_Position << 12, -2, -0.3f, 1;
			Middle_low_1.init_radius = 8.5f;
			Middle_low_1.joint_index = 6;
			Collision_sphere.push_back(Middle_low_1);

			Collision Middle_low_2;
			Middle_low_2.id = 10;
			Middle_low_2.adjscent_id[0] = 9; Middle_low_2.adjscent_id[1] = 11;
			Middle_low_2.init_Position << 24, -1, -0.3f, 1;
			Middle_low_2.init_radius = 8.5f;
			Middle_low_2.joint_index = 6;
			Collision_sphere.push_back(Middle_low_2);
		}

		//Middle_middle
		{
			Collision Middle_middle_0;
			Middle_middle_0.id = 11;
			Middle_middle_0.adjscent_id[0] = 10; Middle_middle_0.adjscent_id[1] = 12;
			Middle_middle_0.init_Position << 0, -1.0f, -0.3f, 1;
			Middle_middle_0.init_radius = 8.0f;
			Middle_middle_0.joint_index = 7;
			Collision_sphere.push_back(Middle_middle_0);

			Collision Middle_middle_1;
			Middle_middle_1.id = 12;
			Middle_middle_1.adjscent_id[0] = 11; Middle_middle_1.adjscent_id[1] = 13;
			Middle_middle_1.init_Position << 10, -1.0f, 0.5f, 1;
			Middle_middle_1.init_radius = 7.3f;
			Middle_middle_1.joint_index = 7;
			Collision_sphere.push_back(Middle_middle_1);

			Collision Middle_middle_2;
			Middle_middle_2.id = 13;
			Middle_middle_2.adjscent_id[0] = 12; Middle_middle_2.adjscent_id[1] = 14;
			Middle_middle_2.init_Position << 20, -1.0f, 1.0f, 1;
			Middle_middle_2.init_radius = 6.8f;
			Middle_middle_2.joint_index = 7;
			Collision_sphere.push_back(Middle_middle_2);
		}

		//Middle_top
		{
			Collision Middle_top_0;
			Middle_top_0.id = 14;
			Middle_top_0.adjscent_id[0] = 13; Middle_top_0.adjscent_id[1] = 15;
			Middle_top_0.init_Position << 0, -1.0f, 1.0f, 1;
			Middle_top_0.init_radius = 6.3f;
			Middle_top_0.joint_index = 8;
			Collision_sphere.push_back(Middle_top_0);

			Collision Middle_top_1;
			Middle_top_1.id = 15;
			Middle_top_1.adjscent_id[0] = 14; Middle_top_1.adjscent_id[1] = -1;
			Middle_top_1.init_Position << 10, -1.0f, 1.0f, 1;
			Middle_top_1.init_radius = 5.2f;
			Middle_top_1.joint_index = 8;
			Collision_sphere.push_back(Middle_top_1);
		}
	}

	//Ring
	{
		//Ring_low
		{
			Collision Ring_low_0;
			Ring_low_0.id = 16;
			Ring_low_0.adjscent_id[0] = -1; Ring_low_0.adjscent_id[1] = 17;
			Ring_low_0.init_Position << 0, -1.3f, 0.0f, 1;
			Ring_low_0.init_radius = 8.3f;
			Ring_low_0.joint_index = 14;
			Collision_sphere.push_back(Ring_low_0);

			Collision Ring_low_1;
			Ring_low_1.id = 17;
			Ring_low_1.adjscent_id[0] = 16; Ring_low_1.adjscent_id[1] = 18;
			Ring_low_1.init_Position << 11, -1.3f, 0.0f, 1;
			Ring_low_1.init_radius = 7.6f;
			Ring_low_1.joint_index = 14;
			Collision_sphere.push_back(Ring_low_1);

			Collision Ring_low_2;
			Ring_low_2.id = 18;
			Ring_low_2.adjscent_id[0] = 17; Ring_low_2.adjscent_id[1] = 19;
			Ring_low_2.init_Position << 22, -1.4f, -0.7f, 1;
			Ring_low_2.init_radius = 7.3f;
			Ring_low_2.joint_index = 14;
			Collision_sphere.push_back(Ring_low_2);
		}

		//Ring_middle
		{
			Collision Ring_middle_0;
			Ring_middle_0.id = 19;
			Ring_middle_0.adjscent_id[0] = 18; Ring_middle_0.adjscent_id[1] = 20;
			Ring_middle_0.init_Position << 0, -1.3f, -1.1f, 1;
			Ring_middle_0.init_radius = 6.9f;
			Ring_middle_0.joint_index = 15;
			Collision_sphere.push_back(Ring_middle_0);

			Collision Ring_middle_1;
			Ring_middle_1.id = 20;
			Ring_middle_1.adjscent_id[0] = 19; Ring_middle_1.adjscent_id[1] = 21;
			Ring_middle_1.init_Position << 11, -1.3f, -1.4f, 1;
			Ring_middle_1.init_radius = 6.4f;
			Ring_middle_1.joint_index = 15;
			Collision_sphere.push_back(Ring_middle_1);

			Collision Ring_middle_2;
			Ring_middle_2.id = 21;
			Ring_middle_2.adjscent_id[0] = 20; Ring_middle_2.adjscent_id[1] = 22;
			Ring_middle_2.init_Position << 22, -1.3f, -1.6f, 1;
			Ring_middle_2.init_radius = 5.9f;
			Ring_middle_2.joint_index = 15;
			Collision_sphere.push_back(Ring_middle_2);
		}

		//Ring_top
		{
			Collision Ring_top_0;
			Ring_top_0.id = 22;
			Ring_top_0.adjscent_id[0] = 21; Ring_top_0.adjscent_id[1] = 23;
			Ring_top_0.init_Position << 0, -1.3f, -1.6f, 1;
			Ring_top_0.init_radius = 5.6f;
			Ring_top_0.joint_index = 16;
			Collision_sphere.push_back(Ring_top_0);

			Collision Ring_top_1;
			Ring_top_1.id = 23;
			Ring_top_1.adjscent_id[0] = 22; Ring_top_1.adjscent_id[1] = -1;
			Ring_top_1.init_Position << 8, -1.3f, -2.0f, 1.0f;
			Ring_top_1.init_radius = 4.7f;
			Ring_top_1.joint_index = 16;
			Collision_sphere.push_back(Ring_top_1);
		}
	}

	//Pinkey
	{
		//Pinkey_low
		{
			Collision Pinkey_Low_0;
			Pinkey_Low_0.id = 24;
			Pinkey_Low_0.adjscent_id[0] = -1; Pinkey_Low_0.adjscent_id[1] = 25;
			Pinkey_Low_0.init_Position << 0, -1, -0.8f, 1;
			Pinkey_Low_0.init_radius = 7.5f;
			Pinkey_Low_0.joint_index = 10;
			Collision_sphere.push_back(Pinkey_Low_0);

			Collision Pinkey_low_1;
			Pinkey_low_1.id = 25;
			Pinkey_low_1.adjscent_id[0] = 24; Pinkey_low_1.adjscent_id[1] = 26;
			Pinkey_low_1.init_Position << 10, -1, -1.5f, 1;
			Pinkey_low_1.init_radius = 6.9f;
			Pinkey_low_1.joint_index = 10;
			Collision_sphere.push_back(Pinkey_low_1);

			Collision Pinkey_low_2;
			Pinkey_low_2.id = 26;
			Pinkey_low_2.adjscent_id[0] = 25; Pinkey_low_2.adjscent_id[1] = 27;
			Pinkey_low_2.init_Position << 19, -1, -1.8f, 1;
			Pinkey_low_2.init_radius = 6.4f;
			Pinkey_low_2.joint_index = 10;
			Collision_sphere.push_back(Pinkey_low_2);
		}

		//Pinkey_middle
		{
			Collision Pinkey_middle_0;
			Pinkey_middle_0.id = 27;
			Pinkey_middle_0.adjscent_id[0] = 26; Pinkey_middle_0.adjscent_id[1] = 28;
			Pinkey_middle_0.init_Position << 0, -1, -1.8f, 1;
			Pinkey_middle_0.init_radius = 6.4f;
			Pinkey_middle_0.joint_index = 11;
			Collision_sphere.push_back(Pinkey_middle_0);

			Collision Pinkey_middle_1;
			Pinkey_middle_1.id = 28;
			Pinkey_middle_1.adjscent_id[0] = 27; Pinkey_middle_1.adjscent_id[1] = 29;
			Pinkey_middle_1.init_Position << 10, -1, -1.8f, 1;
			Pinkey_middle_1.init_radius = 6.0f;
			Pinkey_middle_1.joint_index = 11;
			Collision_sphere.push_back(Pinkey_middle_1);

			Collision Pinkey_middle_2;
			Pinkey_middle_2.id = 29;
			Pinkey_middle_2.adjscent_id[0] = 28; Pinkey_middle_2.adjscent_id[1] = 30;
			Pinkey_middle_2.init_Position << 16, -1, -1.8f, 1;
			Pinkey_middle_2.init_radius = 5.5f;
			Pinkey_middle_2.joint_index = 11;
			Collision_sphere.push_back(Pinkey_middle_2);
		}

		//Pinkey_top
		{
			Collision Pinkey_top_0;
			Pinkey_top_0.id = 30;
			Pinkey_top_0.adjscent_id[0] = 29; Pinkey_top_0.adjscent_id[1] = 31;
			Pinkey_top_0.init_Position << 0, -1, -1.8f, 1;
			Pinkey_top_0.init_radius = 5.3f;
			Pinkey_top_0.joint_index = 12;
			Collision_sphere.push_back(Pinkey_top_0);

			Collision Pinkey_top_1;
			Pinkey_top_1.id = 31;
			Pinkey_top_1.adjscent_id[0] = 30; Pinkey_top_1.adjscent_id[1] = -1;
			Pinkey_top_1.init_Position << 9, -1, -1.8f, 1;
			Pinkey_top_1.init_radius = 4.2f;
			Pinkey_top_1.joint_index = 12;
			Collision_sphere.push_back(Pinkey_top_1);
		}
	}

	//Thumb
	{
		//Thumb_low
		{
			Collision Thumb_Low_0;
			Thumb_Low_0.id = 32;
			Thumb_Low_0.adjscent_id[0] = -1; Thumb_Low_0.adjscent_id[1] = 33;
			Thumb_Low_0.init_Position << 0, -14.0f, 3.0f, 1;
			Thumb_Low_0.init_radius = 16.0f;
			Thumb_Low_0.joint_index = 18;
			Collision_sphere.push_back(Thumb_Low_0);

			Collision Thumb_low_1;
			Thumb_low_1.id = 33;
			Thumb_low_1.adjscent_id[0] = 32; Thumb_low_1.adjscent_id[1] = 34;
			Thumb_low_1.init_Position << 17, -8.0f, 3.0f, 1;
			Thumb_low_1.init_radius = 13.0f;
			Thumb_low_1.joint_index = 18;
			Collision_sphere.push_back(Thumb_low_1);
		}

		//Thumb_middle
		{
			Collision Thumb_middle_0;
			Thumb_middle_0.id = 34;
			Thumb_middle_0.adjscent_id[0] = 33; Thumb_middle_0.adjscent_id[1] = 35;
			Thumb_middle_0.init_Position << 0, -1.0f, 3.0f, 1;
			Thumb_middle_0.init_radius = 11.0f;
			Thumb_middle_0.joint_index = 19;
			Collision_sphere.push_back(Thumb_middle_0);

			Collision Thumb_middle_1;
			Thumb_middle_1.id = 35;
			Thumb_middle_1.adjscent_id[0] = 34; Thumb_middle_1.adjscent_id[1] = 36;
			Thumb_middle_1.init_Position << 14, 0.0f, 1.0f, 1;
			Thumb_middle_1.init_radius = 8.5f;
			Thumb_middle_1.joint_index = 19;
			Collision_sphere.push_back(Thumb_middle_1);

			Collision Thumb_middle_2;
			Thumb_middle_2.id = 36;
			Thumb_middle_2.adjscent_id[0] = 35; Thumb_middle_2.adjscent_id[1] = 37;
			Thumb_middle_2.init_Position << 24, 0.0f, 1.0f, 1;
			Thumb_middle_2.init_radius = 8.3f;
			Thumb_middle_2.joint_index = 19;
			Collision_sphere.push_back(Thumb_middle_2);
		}

		//Thumb_top
		{
			Collision Thumb_top_0;
			Thumb_top_0.id = 37;
			Thumb_top_0.adjscent_id[0] = 36; Thumb_top_0.adjscent_id[1] = 38;
			Thumb_top_0.init_Position << 0, -1, 1.0f, 1;
			Thumb_top_0.init_radius = 8.0f;
			Thumb_top_0.joint_index = 20;
			Collision_sphere.push_back(Thumb_top_0);

			Collision Thumb_top_1;
			Thumb_top_1.id = 38;
			Thumb_top_1.adjscent_id[0] = 37; Thumb_top_1.adjscent_id[1] = 39;
			Thumb_top_1.init_Position << 9, -1, 1.0f, 1;
			Thumb_top_1.init_radius = 6.0f;
			Thumb_top_1.joint_index = 20;
			Collision_sphere.push_back(Thumb_top_1);

			Collision Thumb_top_2;
			Thumb_top_2.id = 39;
			Thumb_top_2.adjscent_id[0] = 38; Thumb_top_2.adjscent_id[1] = -1;
			Thumb_top_2.init_Position << 16, 2, 1.0f, 1;
			Thumb_top_2.init_radius = 5.0f;
			Thumb_top_2.joint_index = 20;
			Collision_sphere.push_back(Thumb_top_2);
		}
	}


}

void HandModel::updata_collosion()
{
	Eigen::Vector4f handbase = Joints[1].global*Joints[1].local.inverse()*Joints[1].GlobalInitPosition;
	for (int i = 0; i < Collision_sphere.size(); ++i)
	{
		Collision_sphere[i].updata_Position << Joints[Collision_sphere[i].joint_index].global*Collision_sphere[i].init_Position + GlobalPosition - handbase;
		Collision_sphere[i].updata_radius = Hand_scale(0)*Collision_sphere[i].init_radius;
	}
}

Eigen::MatrixXf HandModel::Compute_one_CollisionPoint_Jacobian(Collision& a, Eigen::Vector3f& point)
{
	float omiga = 3.141592f / 180.0f;
	Eigen::MatrixXf Jacobain_ = Eigen::MatrixXf::Zero(3, NumberofParams);

	int current_indx = a.joint_index;
	Eigen::Vector3f current_CollisionPoint_position(point(0), point(1), point(2));

	//计算时候会用到的变量
	Eigen::Vector3f axis_base_position;
	Eigen::Vector3f x_axis_position;
	Eigen::Vector3f y_axis_position;
	Eigen::Vector3f z_axis_position;

	Eigen::Vector3f w_x, w_y, w_z;
	Eigen::Vector3f S;
	Eigen::Vector3f result;

	while (current_indx >= 0)
	{
		axis_base_position << Joints[current_indx].CorrespondingPosition(0), Joints[current_indx].CorrespondingPosition(1), Joints[current_indx].CorrespondingPosition(2);

		x_axis_position << Joints[current_indx].CorrespondingAxis[0](0), Joints[current_indx].CorrespondingAxis[0](1), Joints[current_indx].CorrespondingAxis[0](2);
		y_axis_position << Joints[current_indx].CorrespondingAxis[1](0), Joints[current_indx].CorrespondingAxis[1](1), Joints[current_indx].CorrespondingAxis[1](2);
		z_axis_position << Joints[current_indx].CorrespondingAxis[2](0), Joints[current_indx].CorrespondingAxis[2](1), Joints[current_indx].CorrespondingAxis[2](2);


		w_x << (x_axis_position - axis_base_position);
		w_y << (y_axis_position - axis_base_position);
		w_z << (z_axis_position - axis_base_position);

		w_x.normalize();
		w_y.normalize();
		w_z.normalize();

		S << (current_CollisionPoint_position - axis_base_position);

		int params_len = Joints[current_indx].params_length;
		for (int idx = 0; idx < params_len; idx++)
		{
			int params_idx = Joints[current_indx].params_index[idx];

			switch (Joints[current_indx].params_type[idx])
			{
			case dof_type(x_axis_rotate): {
				result << omiga*w_x.cross(S);
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(y_axis_rotate): {
				result << omiga*w_y.cross(S);
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(z_axis_rotate): {
				result << omiga*w_z.cross(S);
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(x_axis_trans): {
				result << 1, 0, 0;
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(y_axis_trans): {
				result << 0, 1, 0;
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(z_axis_trans): {
				result << 0, 0, 1;
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			}

		}

		current_indx = Joints[current_indx].parent_joint_index;
	}

	return Jacobain_;
}

void HandModel::create_adjacency_matrix()
{
	int NuMofCollision = Collision_sphere.size();
	this->adjacency_matrix = Eigen::MatrixXi::Zero(NuMofCollision, NuMofCollision);
	this->Collision_Judge_Matrix = Eigen::MatrixXi::Zero(NuMofCollision, NuMofCollision);

	for (int i = 0; i < NuMofCollision; ++i)
	{
		adjacency_matrix(i, i) = 1;
		if (Collision_sphere[i].adjscent_id[0] >= 0)  adjacency_matrix(i, Collision_sphere[i].adjscent_id[0]) = 1;
		if (Collision_sphere[i].adjscent_id[1] >= 0)  adjacency_matrix(i, Collision_sphere[i].adjscent_id[1]) = 1;
	}

}

std::pair<Eigen::Vector3f, Eigen::Vector3f> HandModel::Collision_to_Collision_distance(Collision& a, Collision&b)
{
	std::pair<Eigen::Vector3f, Eigen::Vector3f> shortest_path;

	Eigen::Vector3f a_cernter(a.updata_Position.head(3));
	Eigen::Vector3f b_cernter(b.updata_Position.head(3));

	Eigen::Vector3f direction_a_to_b;

	direction_a_to_b << b_cernter - a_cernter;
	direction_a_to_b.normalize();

	Eigen::Vector3f a_point, b_point;

	a_point << a_cernter + a.updata_radius*direction_a_to_b;
	b_point << b_cernter - b.updata_radius*direction_a_to_b;

	shortest_path.first = a_point;
	shortest_path.second = b_point;

	return shortest_path;
}

int HandModel::Judge_Collision()
{
	int NuMofCollision = Collision_sphere.size();
	this->Collision_Judge_Matrix.setZero();

	int CollisionNUM = 0;
	for (int i = 0; i < NuMofCollision; ++i) {
		for (int j = 0; j < NuMofCollision; j++) {
			if (adjacency_matrix(i, j) == 0)
			{
				float distance = ((Collision_sphere[j].updata_Position - Collision_sphere[i].updata_Position).head(3)).norm();

				if (distance < (Collision_sphere[j].updata_radius + Collision_sphere[i].updata_radius))
				{
					this->Collision_Judge_Matrix(i, j) = 1;
					++CollisionNUM;
				}
			}
		}
	}

	return CollisionNUM;
}

std::vector<std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>> HandModel::create_distance_matrix()
{
	int NuMofCollision = Collision_sphere.size();
	std::pair<Eigen::Vector3f, Eigen::Vector3f> big_distance(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(RAND_MAX, RAND_MAX, RAND_MAX));
	std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> current_row(NuMofCollision, big_distance);
	std::vector<std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>> distance_matrix(NuMofCollision, current_row);

	for (int i = 0; i < NuMofCollision; ++i) {
		for (int j = 0; j < NuMofCollision; j++) {
			if (adjacency_matrix(i, j) == 0)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> shortest_path =
					Collision_to_Collision_distance(Collision_sphere[i], Collision_sphere[j]);
				distance_matrix[i][j] = shortest_path;
			}
		}
	}

	return distance_matrix;
}

Eigen::MatrixXf HandModel::Compute_Collision_Limit(Eigen::VectorXf& e_limit)
{
	int NuMofCollision = Collision_sphere.size();
	int CollisionNUM = Judge_Collision();
	Eigen::MatrixXf J_col;

	float fraction = 0.1f;

	if (CollisionNUM > 0)
	{
		J_col = Eigen::MatrixXf::Zero(3 * CollisionNUM, NumberofParams);
		e_limit = Eigen::VectorXf::Zero(3 * CollisionNUM, 1);

		std::vector<std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>> distance_matrix = create_distance_matrix();

		int count = 0;

		for (int i = 0; i < NuMofCollision; ++i)
		{
			for (int j = 0; j < NuMofCollision; ++j)
			{
				if (Collision_Judge_Matrix(i, j) == 1)
				{
					Eigen::Vector3f target_point = distance_matrix[i][j].second;
					Eigen::Vector3f now_point = distance_matrix[i][j].first;

					e_limit(count * 3 + 0) = fraction*(target_point(0) - now_point(0));
					e_limit(count * 3 + 1) = fraction*(target_point(1) - now_point(1));
					e_limit(count * 3 + 2) = fraction*(target_point(2) - now_point(2));

					J_col.block(count * 3, 0, 3, NumberofParams) = Compute_one_CollisionPoint_Jacobian(Collision_sphere[i], now_point);
					count++;
				}
			}
		}
	}
	else
	{
		J_col = Eigen::MatrixXf::Zero(3, NumberofParams);
		e_limit = Eigen::VectorXf::Zero(3, 1);
	}

	return J_col;

}

cv::Mat HandModel::Generate_handimg()
{
	outputImage.setTo(0);
	uchar * pointer = outputImage.data;
	//cout << "size" << Visible_vertices_2D.size() << endl;
	for (int i = 0; i < Visible_vertices_2D.size(); i++)
	{
		if((Visible_vertices_2D[i](0)>=0) && 
			(Visible_vertices_2D[i](0) <=512-1) && 
			(Visible_vertices_2D[i](1) >= 0)&& 
			(Visible_vertices_2D[i](1) <=424-1) )
			pointer[Visible_vertices_2D[i](1)*512 + Visible_vertices_2D[i](0)] = 255;
	}

	return outputImage;
}