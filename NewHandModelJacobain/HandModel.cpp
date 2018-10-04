#include"HandModel.h"
#include<random>

std::random_device rd1;
std::default_random_engine random1 = std::default_random_engine(rd1());
std::uniform_real_distribution<float> distribute1(-10.0f, 10.0f);

auto randfun1 = std::bind(distribute1, random1);
void HandModel::load_glove_data()
{
	std::ifstream f;

	f.open("gloveParams.txt", std::ios::in);
	for (int i = 0; i < 27; i++) {
		f >> Params[i];
	}
	f.close();
	printf("Load visible vertices succeed!!!\n");
}
HandModel::HandModel(Camera *camera_):camera(camera_)
{
	NumofJoints = 22;
	Joints = new Joint_handmodel[22];
	//wrist

	Joints[0].joint_name = "Wrist";
	Joints[0].joint_index = 0;
	Joints[0].GlobalInitPosition << 0, 0, 0, 1;
	Joints[0].ChildGlobalInitPosition << 4.74695f, 71.5019f, -12.1453f, 1;
	Joints[0].joint_dof[0] = true; Joints[0].joint_dof[1] = true; Joints[0].joint_dof[2] = true;
	Joints[0].params_length = 6;
	Joints[0].params_index = new int[6]; Joints[0].params_type = new int[6];
	Joints[0].params_index[0] = 15; Joints[0].params_type[0] = dof_type(x_axis_rotate);
	Joints[0].params_index[1] = 16; Joints[0].params_type[1] = dof_type(y_axis_rotate);
	Joints[0].params_index[2] = 17; Joints[0].params_type[2] = dof_type(z_axis_rotate);
	Joints[0].params_index[3] = 24; Joints[0].params_type[3] = dof_type(x_axis_trans);
	Joints[0].params_index[4] = 25; Joints[0].params_type[4] = dof_type(y_axis_trans);
	Joints[0].params_index[5] = 26; Joints[0].params_type[5] = dof_type(z_axis_trans);
	Joints[0].parent_joint_index = -1;



	//handbone

	Joints[1].joint_name = "Handbone";
	Joints[1].joint_index = 1;
	Joints[1].GlobalInitPosition << 4.74695f, 71.5019f, -12.1453f, 1;
	Joints[1].ChildGlobalInitPosition << 9.00001f, 101.267f, -3.93472f, 1;
	Joints[1].joint_dof[0] = false; Joints[1].joint_dof[1] = false; Joints[1].joint_dof[2] = false;
	Joints[1].params_length = 0;
	Joints[1].params_index = NULL; Joints[1].params_type = NULL;
	Joints[1].parent_joint_index = 0;

	//index

	Joints[2].joint_name = "IndexLower";
	Joints[2].joint_index = 2;
	Joints[2].GlobalInitPosition << 29.2408f, 96.351f, -3.11671f, 1;
	Joints[2].ChildGlobalInitPosition << 35.6817f, 128.786f, -3.3307f, 1;
	Joints[2].joint_dof[0] = false; Joints[2].joint_dof[1] = true; Joints[2].joint_dof[2] = true;
	Joints[2].params_length = 2;
	Joints[2].params_index = new int[2]; Joints[2].params_type = new int[2];
	Joints[2].params_index[0] = 9; Joints[2].params_type[0] = dof_type(y_axis_rotate);
	Joints[2].params_index[1] = 10; Joints[2].params_type[1] = dof_type(z_axis_rotate);
	Joints[2].parent_joint_index = 1;



	Joints[3].joint_name = "IndexMiddle";
	Joints[3].joint_index = 3;
	Joints[3].GlobalInitPosition << 35.6817f, 128.786f, -3.3307f, 1;
	Joints[3].ChildGlobalInitPosition << 39.1804f, 159.882f, -3.93921f, 1;
	Joints[3].joint_dof[0] = false; Joints[3].joint_dof[1] = true; Joints[3].joint_dof[2] = false;
	Joints[3].params_length = 1;
	Joints[3].params_index = new int[1]; Joints[3].params_type = new int[1];
	Joints[3].params_index[0] = 11; Joints[3].params_type[0] = dof_type(y_axis_rotate);
	Joints[3].parent_joint_index = 2;


	Joints[4].joint_name = "IndexTop";
	Joints[4].joint_index = 4;
	Joints[4].GlobalInitPosition << 39.1804f, 159.882f, -3.93921f, 1;
	Joints[4].ChildGlobalInitPosition << 40.8995f, 178.091f, -5.84128f, 1;
	Joints[4].joint_dof[0] = false; Joints[4].joint_dof[1] = true; Joints[4].joint_dof[2] = false;
	Joints[4].params_length = 1;
	Joints[4].params_index = new int[1]; Joints[4].params_type = new int[1];
	Joints[4].params_index[0] = 23; Joints[4].params_type[0] = dof_type(y_axis_rotate);
	Joints[4].parent_joint_index = 3;


	Joints[5].joint_name = "IndexSite";
	Joints[5].joint_index = 5;
	Joints[5].GlobalInitPosition << 40.8995f, 178.091f, -5.84128f, 1;
	Joints[5].HasChild = false;
	Joints[5].joint_dof[0] = false; Joints[5].joint_dof[1] = false; Joints[5].joint_dof[2] = false;
	Joints[5].params_length = 0;
	Joints[5].params_index = NULL; Joints[5].params_type = NULL;
	Joints[5].parent_joint_index = 4;



	//middle

	Joints[6].joint_name = "MiddleLower";
	Joints[6].joint_index = 6;
	Joints[6].GlobalInitPosition << 9.00001f, 101.267f, -3.93472f, 1;
	Joints[6].ChildGlobalInitPosition << 8.19879f, 138.712f, -5.20814f, 1;
	Joints[6].joint_dof[0] = false; Joints[6].joint_dof[1] = true; Joints[6].joint_dof[2] = false;
	Joints[6].params_length = 1;
	Joints[6].params_index = new int[1]; Joints[6].params_type = new int[1];
	Joints[6].params_index[0] = 6;  Joints[6].params_type[0] = dof_type(y_axis_rotate);
	Joints[6].parent_joint_index = 1;


	Joints[7].joint_name = "MiddleMiddle";
	Joints[7].joint_index = 7;
	Joints[7].GlobalInitPosition << 8.19879f, 138.712f, -5.20814f, 1;
	Joints[7].ChildGlobalInitPosition << 7.12465f, 168.352f, -4.7175f, 1;
	Joints[7].joint_dof[0] = false; Joints[7].joint_dof[1] = true; Joints[7].joint_dof[2] = false;
	Joints[7].params_length = 1;
	Joints[7].params_index = new int[1]; Joints[7].params_type = new int[1];
	Joints[7].params_index[0] = 8; Joints[7].params_type[0] = dof_type(y_axis_rotate);
	Joints[7].parent_joint_index = 6;


	Joints[8].joint_name = "MiddleTop";
	Joints[8].joint_index = 8;
	Joints[8].GlobalInitPosition << 7.12465f, 168.352f, -4.7175f, 1;
	Joints[8].ChildGlobalInitPosition << 6.95019f, 185.733f, -7.207f, 1;
	Joints[8].joint_dof[0] = false; Joints[8].joint_dof[1] = true; Joints[8].joint_dof[2] = false;
	Joints[8].params_length = 1;
	Joints[8].params_index = new int[1]; Joints[8].params_type = new int[1];
	Joints[8].params_index[0] = 22; Joints[8].params_type[0] = dof_type(y_axis_rotate);
	Joints[8].parent_joint_index = 7;


	Joints[9].joint_name = "MiddleSite";
	Joints[9].joint_index = 9;
	Joints[9].GlobalInitPosition << 6.95019f, 185.733f, -7.207f, 1;
	Joints[9].HasChild = false;
	Joints[9].joint_dof[0] = false; Joints[9].joint_dof[1] = false; Joints[9].joint_dof[2] = false;
	Joints[9].params_length = 0;
	Joints[9].params_index = NULL; Joints[9].params_type = NULL;
	Joints[9].parent_joint_index = 8;


	//Pinkey


	Joints[10].joint_name = "PinkeyLower";
	Joints[10].joint_index = 10;
	Joints[10].GlobalInitPosition << -28.85f, 86.4131f, -0.163318f, 1;
	Joints[10].ChildGlobalInitPosition << -40.1235f, 115.175f, 0.380951f, 1;
	Joints[10].joint_dof[0] = false; Joints[10].joint_dof[1] = true; Joints[10].joint_dof[2] = true;
	Joints[10].params_length = 2;
	Joints[10].params_index = new int[2]; Joints[10].params_type = new int[2];
	Joints[10].params_index[0] = 0; Joints[10].params_type[0] = dof_type(y_axis_rotate);
	Joints[10].params_index[1] = 1; Joints[10].params_type[1] = dof_type(z_axis_rotate);
	Joints[10].parent_joint_index = 1;


	Joints[11].joint_name = "PinkeyMiddle";
	Joints[11].joint_index = 11;
	Joints[11].GlobalInitPosition << -40.1235f, 115.175f, 0.380951f, 1;
	Joints[11].ChildGlobalInitPosition << -49.0937f, 136.986f, -0.0051192f, 1;
	Joints[11].joint_dof[0] = false; Joints[11].joint_dof[1] = true; Joints[11].joint_dof[2] = false;
	Joints[11].params_length = 1;
	Joints[11].params_index = new int[1]; Joints[11].params_type = new int[1];
	Joints[11].params_index[0] = 2; Joints[11].params_type[0] = dof_type(y_axis_rotate);
	Joints[11].parent_joint_index = 10;


	Joints[12].joint_name = "PinkeyTop";
	Joints[12].joint_index = 12;
	Joints[12].GlobalInitPosition << -49.0937f, 136.986f, -0.0051192f, 1;
	Joints[12].ChildGlobalInitPosition << -55.3593f, 151.231f, -0.197348f, 1;
	Joints[12].joint_dof[0] = false; Joints[12].joint_dof[1] = true; Joints[12].joint_dof[2] = false;
	Joints[12].params_length = 1;
	Joints[12].params_index = new int[1]; Joints[12].params_type = new int[1];
	Joints[12].params_index[0] = 20; Joints[12].params_type[0] = dof_type(y_axis_rotate);
	Joints[12].parent_joint_index = 11;


	Joints[13].joint_name = "PinkeySite";
	Joints[13].joint_index = 13;
	Joints[13].GlobalInitPosition << -55.3593f, 151.231f, -0.197348f, 1;
	Joints[13].HasChild = false;
	Joints[13].joint_dof[0] = false; Joints[13].joint_dof[1] = false; Joints[13].joint_dof[2] = false;
	Joints[13].params_length = 0;
	Joints[13].params_index = NULL; Joints[13].params_type = NULL;
	Joints[13].parent_joint_index = 12;



	//ring
	Joints[14].joint_name = "RingLower";
	Joints[14].joint_index = 14;
	Joints[14].GlobalInitPosition << -11.2335f, 96.4782f, -3.66419f, 1;
	Joints[14].ChildGlobalInitPosition << -16.3215f, 130.53f, -7.34456f, 1;
	Joints[14].joint_dof[0] = false; Joints[14].joint_dof[1] = true; Joints[14].joint_dof[2] = true;
	Joints[14].params_length = 2;
	Joints[14].params_index = new int[2]; Joints[14].params_type = new int[2];
	Joints[14].params_index[0] = 3; Joints[14].params_type[0] = dof_type(y_axis_rotate);
	Joints[14].params_index[1] = 4; Joints[14].params_type[1] = dof_type(z_axis_rotate);
	Joints[14].parent_joint_index = 1;


	Joints[15].joint_name = "RingMiddle";
	Joints[15].joint_index = 15;
	Joints[15].GlobalInitPosition << -16.3215f, 130.53f, -7.34456f, 1;
	Joints[15].ChildGlobalInitPosition << -21.4588f, 160.548f, -8.54878f, 1;
	Joints[15].joint_dof[0] = false; Joints[15].joint_dof[1] = true; Joints[15].joint_dof[2] = false;
	Joints[15].params_length = 1;
	Joints[15].params_index = new int[1]; Joints[15].params_type = new int[0];
	Joints[15].params_index[0] = 5;  Joints[15].params_type[0] = dof_type(y_axis_rotate);
	Joints[15].parent_joint_index = 14;


	Joints[16].joint_name = "RingTop";
	Joints[16].joint_index = 16;
	Joints[16].GlobalInitPosition << -21.4588f, 160.548f, -8.54878f, 1;
	Joints[16].ChildGlobalInitPosition << -23.8667f, 178.005f, -9.97166f, 1;
	Joints[16].joint_dof[0] = false; Joints[16].joint_dof[1] = true; Joints[16].joint_dof[2] = false;
	Joints[16].params_length = 1;
	Joints[16].params_index = new int[1]; Joints[16].params_type = new int[1];
	Joints[16].params_index[0] = 21; Joints[16].params_type[0] = dof_type(y_axis_rotate);
	Joints[16].parent_joint_index = 15;


	Joints[17].joint_name = "RingSite";
	Joints[17].joint_index = 17;
	Joints[17].GlobalInitPosition << -23.8667f, 178.005f, -9.97166f, 1;
	Joints[17].HasChild = false;
	Joints[17].joint_dof[0] = false; Joints[17].joint_dof[1] = false; Joints[17].joint_dof[2] = false;
	Joints[17].params_length = 0;
	Joints[17].params_index = NULL; Joints[17].params_type = NULL;
	Joints[17].parent_joint_index = 16;



	//thumb

	Joints[18].joint_name = "ThumbLower";
	Joints[18].joint_index = 18;
	Joints[18].GlobalInitPosition << 42.7179f, 28.8392f, -1.60427f, 1;
	Joints[18].ChildGlobalInitPosition << 62.5114f, 53.7368f, 5.22996f, 1;
	Joints[18].joint_dof[0] = true; Joints[18].joint_dof[1] = true; Joints[18].joint_dof[2] = true;
	Joints[18].params_length = 3;
	Joints[18].params_index = new int[3];  Joints[18].params_type = new int[3];
	Joints[18].params_index[0] = 18; Joints[18].params_type[0] = dof_type(x_axis_rotate);
	Joints[18].params_index[1] = 12; Joints[18].params_type[1] = dof_type(y_axis_rotate);
	Joints[18].params_index[2] = 13; Joints[18].params_type[2] = dof_type(z_axis_rotate);
	Joints[18].parent_joint_index = 1;


	Joints[19].joint_name = "ThumbMiddle";
	Joints[19].joint_index = 19;
	Joints[19].GlobalInitPosition << 62.5114f, 53.7368f, 5.22996f, 1;
	Joints[19].ChildGlobalInitPosition << 86.9918f, 80.2351f, 8.83076f, 1;
	Joints[19].joint_dof[0] = false; Joints[19].joint_dof[1] = true; Joints[19].joint_dof[2] = false;
	Joints[19].params_length = 1;
	Joints[19].params_index = new int[1]; Joints[19].params_type = new int[1];
	Joints[19].params_index[0] = 19; Joints[19].params_type[0] = dof_type(y_axis_rotate);
	Joints[19].parent_joint_index = 18;


	Joints[20].joint_name = "ThumbTop";
	Joints[20].joint_index = 20;
	Joints[20].GlobalInitPosition << 86.9918f, 80.2351f, 8.83076f, 1;
	Joints[20].ChildGlobalInitPosition << 103.135f, 92.3703f, 12.6242f, 1;
	Joints[20].joint_dof[0] = false; Joints[20].joint_dof[1] = true; Joints[20].joint_dof[2] = false;
	Joints[20].params_length = 1;
	Joints[20].params_index = new int[1]; Joints[20].params_type = new int[1];
	Joints[20].params_index[0] = 14; Joints[20].params_type[0] = dof_type(y_axis_rotate);
	Joints[20].parent_joint_index = 19;


	Joints[21].joint_name = "ThumbSite";
	Joints[21].joint_index = 21;
	Joints[21].GlobalInitPosition << 103.135f, 92.3703f, 12.6242f, 1;
	Joints[21].HasChild = false;
	Joints[21].joint_dof[0] = false; Joints[21].joint_dof[1] = false; Joints[21].joint_dof[2] = false;
	Joints[21].params_length = 0;
	Joints[21].params_index = NULL; Joints[21].params_type = NULL;
	Joints[21].parent_joint_index = 20;


	GlobalPosition << 0, 0, 0, 0;
	load_vertices(".\\model\\newVertices.txt");
	load_faces(".\\model\\newFaces.txt");
	load_weight(".\\model\\newWeights.txt");

	outputImage = cv::Mat::zeros(424, 512, CV_8UC1);

	NumberofParams = 27;
	Params = new float[27]();
	ParamsUpperBound = new int[27]();
	ParamsLowerBound = new int[27]();

	//Params[9] = 90;
	//Params[0] = 90;
	//Params[2] = 60;
	//Params[15] = 30;
	//Params[16] = -30;
	//Params[17] = -50;
	//Params[24] = 20;
	//Params[25] = 10;
	//Params[26] = -50;

	////pinkey
	//Params[0] = 70;
	//Params[2] = 50;

	////index
	//Params[9] = 60;

	////middle
	//Params[6] = 20;

	////global rotation
	//Params[15] = 30 + randfun1();
	//Params[16] = -30 + randfun1();
	//Params[17] = -50 + randfun1();

	////global trans
	//Params[24] = 20 + randfun1();
	//Params[25] = 10 + randfun1();
	//Params[26] = -50 + randfun1();

	load_glove_data();
	Params[16] = -Params[16];
	Params[17] = -Params[17];
	Params[18] = -Params[18];
	jacobian = Eigen::MatrixXf::Zero(NumofVertices * 3, NumberofParams);
	Joints_jacobian = Eigen::MatrixXf::Zero(NumofJoints * 3, NumberofParams);

	compute_local_coordinate();
	compute_parent_child_transform();
	Updata(Params);

	hand_scale = 0.9;
	//save_visible_vertices();
	//save_target_vertices();
	load_target_vertices();

	//save_target_joints();
	//load_target_joints();

	load_visible_vertices();

	Solved = false;
}

void HandModel::set_one_rotation(Pose pose, int index)
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

	x(1, 1) = cx; x(2, 2) = cx;          //这里的旋转矩阵之所以会出现正负号的问题，在于坐标系是左手坐标系还是右手坐标系
	x(1, 2) = -sx; x(2, 1) = sx;

	y(0, 0) = cy; y(0, 2) = sy;
	y(2, 0) = -sy; y(2, 2) = cy;

	z(0, 0) = cz; z(1, 1) = cz;
	z(0, 1) = -sz; z(1, 0) = sz;

	Joints[index].rotation = x*y*z;   //旋转顺序 z-y-x 这里是绕固定的世界zyx轴旋转，也等价于 先绕自身x轴旋转，再绕自身y旋转，最后绕z旋转。二者等价
}

void HandModel::load_faces(char* filename)
{
	std::ifstream f;
	f.open(filename, std::ios::in);
	//int number;
	f >> NumofFaces;
	FaceIndex = Eigen::MatrixXf::Zero(NumofFaces, 3);
	for (int i = 0; i < NumofFaces; i++) {
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
	//int number;
	f >> NumofVertices;
	Vectices = Eigen::MatrixXf::Zero(NumofVertices, 3);

	Vertices_normal = Eigen::MatrixXf::Zero(NumofVertices, 3);

	for (int i = 0; i < NumofVertices; i++) {
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
	for (int i = 0; i < NumofVertices; i++) {
		for (int j = 0; j < NumofJoints; j++)
		{
			f >> Weights(i, j);
		}
	}
	f.close();
	printf("Load weights succeed!!!\n");
}

void HandModel::compute_local_coordinate() {

	for (int i = 0; i < NumofJoints; i++)
	{
		float axisx[3] = { 0.0f,0.0f,0.0f };
		float axisy[3] = { 0.0f,0.0f,0.0f };
		float axisz[3] = { 0.0f,0.0f,1.0f };

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
}

void HandModel::compute_parent_child_transform()
{
	for (int i = 0; i < NumofJoints; i++)
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
	GlobalPosition(0) = params[24];
	GlobalPosition(1) = params[25];
	GlobalPosition(2) = params[26];
	GlobalPosition(3) = 1;

	Pose p_globle(params[15], params[16], params[17]);
	set_one_rotation(p_globle, 0);

	//thumb
	Pose p_thumb_lower(params[18], params[12], params[13]);
	Pose p_thumb_middle(0, params[19], 0);
	Pose p_thumb_top(0, params[14], 0);
	set_one_rotation(p_thumb_lower, 18);
	set_one_rotation(p_thumb_middle, 19);
	set_one_rotation(p_thumb_top, 20);

	//pinkey
	Pose p_pinkey_lower(0, params[0], params[1]);
	Pose p_pinkey_middle(0, params[2], 0);
	Pose p_pinkey_top(0, params[20], 0);
	set_one_rotation(p_pinkey_lower, 10);
	set_one_rotation(p_pinkey_middle, 11);
	set_one_rotation(p_pinkey_top, 12);

	//ring
	Pose p_ring_lower(0, params[3], params[4]);
	Pose p_ring_middle(0, params[5], 0);
	Pose p_ring_top(0, params[21], 0);
	set_one_rotation(p_ring_lower, 14);
	set_one_rotation(p_ring_middle, 15);
	set_one_rotation(p_ring_top, 16);

	//middle
	Pose p_middle_lower(0, params[6], params[7]);
	Pose p_middle_middle(0, params[8], 0);
	Pose p_middle_top(0, params[22], 0);
	set_one_rotation(p_middle_lower, 6);
	set_one_rotation(p_middle_middle, 7);
	set_one_rotation(p_middle_top, 8);

	//index
	Pose p_index_lower(0, params[9], params[10]);
	Pose p_index_middle(0, params[11], 0);
	Pose p_index_top(0, params[23], 0);
	set_one_rotation(p_index_lower, 2);
	set_one_rotation(p_index_middle, 3);
	set_one_rotation(p_index_top, 4);
}

void HandModel::compute_global_matrix()
{
	Joints[0].global = Joints[0].rotation*Joints[0].local;

	for (int i = 0; i < NumofJoints; i++)
	{
		int parent_joint_index = Joints[i].parent_joint_index;
		if (parent_joint_index != -1)
		{
			Joints[i].global = Joints[parent_joint_index].global*Joints[i].trans*Joints[i].rotation;
		}
	}
}

void HandModel::Updata_Joints_Axis()
{

	Eigen::Vector4f handbone_position;
	handbone_position << Joints[1].global*hand_scale*Joints[1].local.inverse()*Joints[1].GlobalInitPosition;

	for (int i = 0; i < NumofJoints; i++)
	{
		Joints[i].CorrespondingPosition << Joints[i].global*hand_scale*Joints[i].local.inverse()*Joints[i].GlobalInitPosition + GlobalPosition - handbone_position;

		if (i == 0)
		{

			Eigen::MatrixXf x = Eigen::MatrixXf::Identity(4, 4);


			float cx = cos(Params[15] / 180 * PI);
			float sx = sin(Params[15] / 180 * PI);


			x(1, 1) = cx; x(2, 2) = cx;
			x(1, 2) = -sx; x(2, 1) = sx;


			//这里单独拿出来，因为手部的全局旋转角度变化很大，因此会引起一些问题。若采用如下注释掉的代码，会产生一种现象
			//当x = 180；y = 0；z = 180，时候，手摸z = 180+50的效果是相当于绕z轴旋转-50度，y = 0+50的效果也是相当于 绕z旋转-50度，而不是认为的正50度；
			//但是x = 180+50的效果，还就是相当于绕x轴旋转正50度。
			//实验出的解法是：由于旋转矩阵是 x*y*z，若看作先绕x轴旋转，因此x轴作为实际转轴是等于它原本的轴；
			//                                            再绕经过x轴旋转后，自身的y轴，也就是 x旋转矩阵 * 原本的y轴，这个轴旋转y度；
			//                                          最后绕经过xy轴旋转后，自身的z轴，也就是 经过所有旋转变换后的z轴，这个轴旋转z度；
			//                       （这里采用Joints[0].rotation是因为z轴自身的旋转对自己的轴没影响，也可以换成x*y*Joints[i].dof_axis[2]）
			//
			//              以上都属猜想，具体的数学原理没太明白，但是实际检验的效果符合猜想。
			//Joints[i].CorrespondingAxis[0] << Joints[i].dof_axis[0] + GlobalPosition - handbone_position;
			//Joints[i].CorrespondingAxis[1] << Joints[i].dof_axis[1] + GlobalPosition - handbone_position;
			//Joints[i].CorrespondingAxis[2] << Joints[i].dof_axis[2] + GlobalPosition - handbone_position;

			Joints[i].CorrespondingAxis[0] << Joints[i].dof_axis[0] + GlobalPosition - handbone_position;
			Joints[i].CorrespondingAxis[1] << x*Joints[i].dof_axis[1] + GlobalPosition - handbone_position;
			Joints[i].CorrespondingAxis[2] << Joints[0].rotation*Joints[i].dof_axis[2] + GlobalPosition - handbone_position;
		}
		else
		{
			//其余关节点的坐标都是在该模型坐标系下的局部坐标，因此不需要乘以Local.inverse()转换到局部坐标系下。
			//另外，没有另外如果i==0做处理，是由于角度变化对于转轴的影响不大，所以通过实际观察，可以不做处理
			Joints[i].CorrespondingAxis[0] << Joints[i].global*Joints[i].dof_axis[0] + GlobalPosition - handbone_position;
			Joints[i].CorrespondingAxis[1] << Joints[i].global*Joints[i].dof_axis[1] + GlobalPosition - handbone_position;
			Joints[i].CorrespondingAxis[2] << Joints[i].global*Joints[i].dof_axis[2] + GlobalPosition - handbone_position;
		}

	}
}

void HandModel::Updata_Vertics()
{
	Eigen::Vector4f handbone_position;
	handbone_position << Joints[1].global*hand_scale*Joints[1].local.inverse()*Joints[1].GlobalInitPosition;

	Eigen::MatrixXf t = Eigen::MatrixXf::Zero(4, NumofVertices);
	Eigen::MatrixXf x = Eigen::MatrixXf::Ones(4, NumofVertices);
	x.block(0, 0, 3, NumofVertices) = Vectices.block(0, 0, NumofVertices, 3).transpose();
	for (int i = 0; i < NumofJoints; i++) {
		Eigen::MatrixXf y = Weights.block(0, i, NumofVertices, 1);// 在所有顶点 对于 该关节点的weight
		Eigen::MatrixXf y0 = y.replicate(1, 4);    //分别是行重复1遍，列重复4遍，结果为（num_vertices_，4）这么大小的矩阵
		Eigen::MatrixXf z = Joints[i].global *hand_scale* Joints[i].local.inverse() * x;
		t = t + z.cwiseProduct(y0.transpose());
	}
	vertices_update_ = t.transpose();

	for (int i = 0; i < vertices_update_.rows(); i++) {
		vertices_update_(i, 0) += GlobalPosition(0) - handbone_position.x();
		vertices_update_(i, 1) += GlobalPosition(1) - handbone_position.y();
		vertices_update_(i, 2) += GlobalPosition(2) - handbone_position.z();
	}
}

void HandModel::Updata(float* params)
{
	compute_rotation_matrix(params);
	compute_global_matrix();

	Updata_Joints_Axis();
	Updata_Vertics();
	//Updata_Jacobian();

	Compute_normal_And_visibel_vertices();

	Joint_matrix = Eigen::MatrixXf::Zero(NumofJoints, 3);
	for (int i = 0; i < NumofJoints; i++)
	{
		Joint_matrix(i, 0) = Joints[i].CorrespondingPosition(0);
		Joint_matrix(i, 1) = Joints[i].CorrespondingPosition(1);
		Joint_matrix(i, 2) = Joints[i].CorrespondingPosition(2);
	}
}

void HandModel::Updata_Jacobian()
{
	float kk = 3.141592f / 180.0;
	jacobian.setZero();
	//cout << jacobian.rows() << "," << jacobian.cols();

	for (int vert_idx = 0; vert_idx < NumofVertices; vert_idx++)
	{
		for (int joint_idx = 0; joint_idx < NumofJoints; joint_idx++)
		{
			if (Weights(vert_idx, joint_idx) > 0.0f)
			{
				//do some thing follow the kanimatic chains
				int current_joint_idx = joint_idx;
				float weight = Weights(vert_idx, joint_idx);

				while (current_joint_idx != -1)
				{
					Eigen::Vector3f vertice_position(vertices_update_(vert_idx, 0), vertices_update_(vert_idx, 1), vertices_update_(vert_idx, 2));
					Eigen::Vector3f joint_position(Joints[current_joint_idx].CorrespondingPosition(0), Joints[current_joint_idx].CorrespondingPosition(1), Joints[current_joint_idx].CorrespondingPosition(2));

					Eigen::Vector3f x_axis_position(Joints[current_joint_idx].CorrespondingAxis[0](0), Joints[current_joint_idx].CorrespondingAxis[0](1), Joints[current_joint_idx].CorrespondingAxis[0](2));
					Eigen::Vector3f y_axis_position(Joints[current_joint_idx].CorrespondingAxis[1](0), Joints[current_joint_idx].CorrespondingAxis[1](1), Joints[current_joint_idx].CorrespondingAxis[1](2));
					Eigen::Vector3f z_axis_position(Joints[current_joint_idx].CorrespondingAxis[2](0), Joints[current_joint_idx].CorrespondingAxis[2](1), Joints[current_joint_idx].CorrespondingAxis[2](2));

					Eigen::Vector3f w_x, w_y, w_z;
					Eigen::Vector3f S;

					w_x << (x_axis_position - joint_position);
					w_y << (y_axis_position - joint_position);
					w_z << (z_axis_position - joint_position);

					w_x.normalize();
					w_y.normalize();
					w_z.normalize();

					S << (vertice_position - joint_position);


					int params_len = Joints[current_joint_idx].params_length;
					for (int idx = 0; idx < params_len; idx++)
					{
						Eigen::Vector3f result;
						int params_idx = Joints[current_joint_idx].params_index[idx];

						switch (Joints[current_joint_idx].params_type[idx])
						{
						case dof_type(x_axis_rotate): {
							result << kk*w_x.cross(S);
							jacobian(vert_idx * 3, params_idx) += weight*result(0);
							jacobian(vert_idx * 3 + 1, params_idx) += weight*result(1);
							jacobian(vert_idx * 3 + 2, params_idx) += weight*result(2);
							break;
						}
						case dof_type(y_axis_rotate): {
							result << kk*w_y.cross(S);
							jacobian(vert_idx * 3, params_idx) += weight*result(0);
							jacobian(vert_idx * 3 + 1, params_idx) += weight*result(1);
							jacobian(vert_idx * 3 + 2, params_idx) += weight*result(2);
							break;
						}
						case dof_type(z_axis_rotate): {
							result << kk*w_z.cross(S);
							jacobian(vert_idx * 3, params_idx) += weight*result(0);
							jacobian(vert_idx * 3 + 1, params_idx) += weight*result(1);
							jacobian(vert_idx * 3 + 2, params_idx) += weight*result(2);
							break;
						}
						case dof_type(x_axis_trans): {
							result << 1, 0, 0;
							jacobian(vert_idx * 3, params_idx) += weight*result(0);
							jacobian(vert_idx * 3 + 1, params_idx) += weight*result(1);
							jacobian(vert_idx * 3 + 2, params_idx) += weight*result(2);
							break;
						}
						case dof_type(y_axis_trans): {
							result << 0, 1, 0;
							jacobian(vert_idx * 3, params_idx) += weight*result(0);
							jacobian(vert_idx * 3 + 1, params_idx) += weight*result(1);
							jacobian(vert_idx * 3 + 2, params_idx) += weight*result(2);
							break;
						}
						case dof_type(z_axis_trans): {
							result << 0, 0, 1;
							jacobian(vert_idx * 3, params_idx) += weight*result(0);
							jacobian(vert_idx * 3 + 1, params_idx) += weight*result(1);
							jacobian(vert_idx * 3 + 2, params_idx) += weight*result(2);
							break;
						}
						}

					}
					current_joint_idx = Joints[current_joint_idx].parent_joint_index;
				}
			}
		}
	}
}

void HandModel::save_target_vertices()
{
	std::ofstream f;
	f.open(".\\model\\target_Vertices.txt", std::ios::out);
	for (int i = 0; i < NumofVertices; i++) {
		f << vertices_update_(i, 0) << "  " << vertices_update_(i, 1) << "  " << vertices_update_(i, 2) << endl;
	}
	f.close();
	printf("Save Target vertices succeed!!!\n");
}

void HandModel::save_visible_vertices()
{
	std::ofstream f;
	f.open(".\\model\\visible_Vertices.txt", std::ios::out);
	f << Visible_vertices.size() << endl;
	for (int i = 0; i < Visible_vertices.size(); i++) {
		f << Visible_vertices[i](0) << "  " << Visible_vertices[i](1) << "  " << Visible_vertices[i](2) << endl;
	}
	f.close();
	printf("Save visible vertices succeed!!!\n");
}

void HandModel::load_visible_vertices()
{
	std::ifstream f;
	//f.open(".\\model\\visible_Vertices.txt", std::ios::in);
	f.open("Kinect_Point_cloud.txt", std::ios::in);
	f >> Load_visible_vertices_NUM;
	Load_visible_vertices = Eigen::MatrixXf::Zero(Load_visible_vertices_NUM, 3);
	for (int i = 0; i < Load_visible_vertices_NUM; i++) {
		f >> Load_visible_vertices(i, 0) >> Load_visible_vertices(i, 1) >> Load_visible_vertices(i, 2);
	}
	f.close();
	printf("Load visible vertices succeed!!!\n");
}

void HandModel::save_target_joints()
{
	std::ofstream f;
	f.open(".\\model\\target_joints.txt", std::ios::out);
	for (int i = 0; i < NumofJoints; i++) {
		f << Joints[i].CorrespondingPosition(0) << "  " << Joints[i].CorrespondingPosition(1) << "  " << Joints[i].CorrespondingPosition(2) << endl;
	}
	f.close();
	printf("Save Target Joints succeed!!!\n");
}

void HandModel::load_target_vertices()
{
	std::ifstream f;
	f.open(".\\model\\target_Vertices.txt", std::ios::in);
	Target_vertices = Eigen::MatrixXf::Zero(NumofVertices, 3);
	for (int i = 0; i < NumofVertices; i++) {
		f >> Target_vertices(i, 0) >> Target_vertices(i, 1) >> Target_vertices(i, 2);
	}
	f.close();
	printf("Load Target vertices succeed!!!\n");
}

void HandModel::load_target_joints()
{
	std::ifstream f;
	f.open(".\\model\\target_joints.txt", std::ios::in);
	Target_joints = Eigen::MatrixXf::Zero(NumofJoints, 3);
	for (int i = 0; i < NumofJoints; i++) {
		f >> Target_joints(i, 0) >> Target_joints(i, 1) >> Target_joints(i, 2);
	}
	f.close();
	printf("Load Target Joints succeed!!!\n");
}

void HandModel::Updata_joints_Jacobian()
{
	float kk = 3.141592f / 180.0;
	Joints_jacobian.setZero();

	for (int i = 0; i < NumofJoints; i++)
	{
		if (i == 0)
		{
			Joints_jacobian(i * 3, 24) = 1;           Joints_jacobian(i * 3, 25) = 0;          Joints_jacobian(i * 3, 26) = 0;
			Joints_jacobian(i * 3 + 1, 24) = 0;       Joints_jacobian(i * 3 + 1, 25) = 1;      Joints_jacobian(i * 3 + 1, 26) = 0;
			Joints_jacobian(i * 3 + 2, 24) = 0;       Joints_jacobian(i * 3 + 2, 25) = 0;      Joints_jacobian(i * 3 + 2, 26) = 1;
		}
		int current_indx = i;
		Eigen::Vector3f current_joint_position(Joints[i].CorrespondingPosition(0), Joints[i].CorrespondingPosition(1), Joints[i].CorrespondingPosition(2));
		while (current_indx >= 0)
		{
			int parent_joint_idx = Joints[current_indx].parent_joint_index;

			if (parent_joint_idx != -1)
			{
				Eigen::Vector3f parent_joint_position(Joints[parent_joint_idx].CorrespondingPosition(0), Joints[parent_joint_idx].CorrespondingPosition(1), Joints[parent_joint_idx].CorrespondingPosition(2));

				Eigen::Vector3f x_axis_position(Joints[parent_joint_idx].CorrespondingAxis[0](0), Joints[parent_joint_idx].CorrespondingAxis[0](1), Joints[parent_joint_idx].CorrespondingAxis[0](2));
				Eigen::Vector3f y_axis_position(Joints[parent_joint_idx].CorrespondingAxis[1](0), Joints[parent_joint_idx].CorrespondingAxis[1](1), Joints[parent_joint_idx].CorrespondingAxis[1](2));
				Eigen::Vector3f z_axis_position(Joints[parent_joint_idx].CorrespondingAxis[2](0), Joints[parent_joint_idx].CorrespondingAxis[2](1), Joints[parent_joint_idx].CorrespondingAxis[2](2));

				Eigen::Vector3f w_x, w_y, w_z;
				Eigen::Vector3f S;

				w_x << (x_axis_position - parent_joint_position);
				w_y << (y_axis_position - parent_joint_position);
				w_z << (z_axis_position - parent_joint_position);

				w_x.normalize();
				w_y.normalize();
				w_z.normalize();

				S << (current_joint_position - parent_joint_position);

				int params_len = Joints[parent_joint_idx].params_length;
				for (int idx = 0; idx < params_len; idx++)
				{
					Eigen::Vector3f result;
					int params_idx = Joints[parent_joint_idx].params_index[idx];

					switch (Joints[parent_joint_idx].params_type[idx])
					{
					case dof_type(x_axis_rotate): {
						result << kk*w_x.cross(S);
						Joints_jacobian(i * 3, params_idx) += result(0);
						Joints_jacobian(i * 3 + 1, params_idx) += result(1);
						Joints_jacobian(i * 3 + 2, params_idx) += result(2);
						break;
					}
					case dof_type(y_axis_rotate): {
						result << kk*w_y.cross(S);
						Joints_jacobian(i * 3, params_idx) += result(0);
						Joints_jacobian(i * 3 + 1, params_idx) += result(1);
						Joints_jacobian(i * 3 + 2, params_idx) += result(2);
						break;
					}
					case dof_type(z_axis_rotate): {
						result << kk*w_z.cross(S);
						Joints_jacobian(i * 3, params_idx) += result(0);
						Joints_jacobian(i * 3 + 1, params_idx) += result(1);
						Joints_jacobian(i * 3 + 2, params_idx) += result(2);
						break;
					}
					case dof_type(x_axis_trans): {
						result << 1, 0, 0;
						Joints_jacobian(i * 3, params_idx) += result(0);
						Joints_jacobian(i * 3 + 1, params_idx) += result(1);
						Joints_jacobian(i * 3 + 2, params_idx) += result(2);
						break;
					}
					case dof_type(y_axis_trans): {
						result << 0, 1, 0;
						Joints_jacobian(i * 3, params_idx) += result(0);
						Joints_jacobian(i * 3 + 1, params_idx) += result(1);
						Joints_jacobian(i * 3 + 2, params_idx) += result(2);
						break;
					}
					case dof_type(z_axis_trans): {
						result << 0, 0, 1;
						Joints_jacobian(i * 3, params_idx) += result(0);
						Joints_jacobian(i * 3 + 1, params_idx) += result(1);
						Joints_jacobian(i * 3 + 2, params_idx) += result(2);
						break;
					}
					}

				}

			}

			current_indx = parent_joint_idx;
		}
	}
}

void HandModel::MoveToJointTarget()
{
	Eigen::VectorXf e = Eigen::VectorXf::Zero(3 * NumofJoints, 1);
	for (int i = 0; i < NumofJoints; i++)
	{
		e(i * 3 + 0) = -Joint_matrix(i, 0) + Target_joints(i, 0);
		e(i * 3 + 1) = -Joint_matrix(i, 1) + Target_joints(i, 1);
		e(i * 3 + 2) = -Joint_matrix(i, 2) + Target_joints(i, 2);
	}


	Updata_joints_Jacobian();
	Eigen::MatrixXf J = Joints_jacobian;
	Eigen::MatrixXf Jt = Joints_jacobian.transpose();


	Eigen::Matrix<float, 27, 27> D = Eigen::Matrix<float, 27, 27>::Identity(27, 27);

	int omiga = 20;

	Eigen::VectorXf dAngles = Eigen::VectorXf::Zero(NumberofParams, 1);

	MatrixXf JtJ = Jt*J + omiga*D;
	VectorXf JTe = Jt*e;

	dAngles = JtJ.colPivHouseholderQr().solve(JTe);

	for (int i = 0; i < NumberofParams; i++)
		Params[i] += dAngles(i);

	Updata(Params);

	Eigen::VectorXf e_updata = Eigen::VectorXf::Zero(3 * NumofJoints, 1);
	for (int i = 0; i < NumofJoints; i++)
	{
		e_updata(i * 3 + 0) = -Joint_matrix(i, 0) + Target_joints(i, 0);
		e_updata(i * 3 + 1) = -Joint_matrix(i, 1) + Target_joints(i, 1);
		e_updata(i * 3 + 2) = -Joint_matrix(i, 2) + Target_joints(i, 2);
	}
	if (e_updata.norm() < 1)
	{
		Solved = true;
	}
	else
	{
		Solved = false;
	}
}

void HandModel::MoveToVerticeTarget()
{
	Eigen::VectorXf e = Eigen::VectorXf::Zero(3 * NumofVertices, 1);
	for (int i = 0; i < NumofVertices; i++)
	{
		e(i * 3 + 0) = -vertices_update_(i, 0) + Target_vertices(i, 0);
		e(i * 3 + 1) = -vertices_update_(i, 1) + Target_vertices(i, 1);
		e(i * 3 + 2) = -vertices_update_(i, 2) + Target_vertices(i, 2);
	}

	Updata_Jacobian();

	Eigen::MatrixXf J = jacobian;
	Eigen::MatrixXf Jt = jacobian.transpose();

	Eigen::VectorXf e_limit;
	Eigen::MatrixXf J_limit = Compute_joint_Limited(e_limit);

	int omiga_limit = 500;

	Eigen::Matrix<float, 27, 27> D = Eigen::Matrix<float, 27, 27>::Identity(27, 27);

	int rotate_omiga = 20;
	int abound_omiga = 100;

	D(0, 0) = rotate_omiga;
	D(1, 1) = abound_omiga;
	D(2, 2) = rotate_omiga;

	D(3, 3) = rotate_omiga;
	D(4, 4) = abound_omiga;
	D(5, 5) = rotate_omiga;

	D(6, 6) = rotate_omiga;
	D(7, 7) = abound_omiga;
	D(8, 8) = rotate_omiga;

	D(9, 9) = rotate_omiga;
	D(10, 10) = abound_omiga;
	D(11, 11) = rotate_omiga;

	D(12, 12) = rotate_omiga;
	D(13, 13) = rotate_omiga;
	D(14, 14) = rotate_omiga;

	D(15, 15) = rotate_omiga;
	D(16, 16) = rotate_omiga;
	D(17, 17) = rotate_omiga;

	D(18, 18) = rotate_omiga;
	D(19, 19) = rotate_omiga;
	D(20, 20) = rotate_omiga;
	D(21, 21) = rotate_omiga;
	D(22, 22) = rotate_omiga;
	D(23, 23) = rotate_omiga;
	D(24, 24) = 0.01;
	D(25, 25) = 0.01;
	D(26, 26) = 0.01;

	//int omiga = 20;

	Eigen::VectorXf dAngles = Eigen::VectorXf::Zero(NumberofParams, 1);

	MatrixXf JtJ = Jt*J + D + omiga_limit*J_limit.transpose()*J_limit;
	VectorXf JTe = Jt*e + omiga_limit*e_limit;

	dAngles = JtJ.colPivHouseholderQr().solve(JTe);

	cout << dAngles << endl;
	for (int i = 0; i < NumberofParams; i++)
		Params[i] += dAngles(i);

	Updata(Params);

	Eigen::VectorXf e_updata = Eigen::VectorXf::Zero(3 * NumofVertices, 1);

	for (int i = 0; i < NumofVertices; i++)
	{
		e_updata(i * 3 + 0) = -vertices_update_(i, 0) + Target_vertices(i, 0);
		e_updata(i * 3 + 1) = -vertices_update_(i, 1) + Target_vertices(i, 1);
		e_updata(i * 3 + 2) = -vertices_update_(i, 2) + Target_vertices(i, 2);
	}

	if (e_updata.norm() < 1)
	{
		Solved = true;
	}
	else
	{
		Solved = false;
	}
}

void HandModel::Updata_Jacobian_correspond(std::vector<int> & cor)
{
	float kk = 3.141592f / 180.0;
	int NUMofCorrespond = cor.size();

	jacobian_correspond = Eigen::MatrixXf::Zero(NUMofCorrespond * 3, NumberofParams);
	//cout << jacobian.rows() << "," << jacobian.cols();

	for (int cor_id = 0; cor_id < NUMofCorrespond; cor_id++)
	{
		int vert_idx = Visible_vertices_index[cor[cor_id]];
		for (int joint_idx = 0; joint_idx < NumofJoints; joint_idx++)
		{
			if (Weights(vert_idx, joint_idx) > 0.0f)
			{
				//do some thing follow the kanimatic chains
				int current_joint_idx = joint_idx;
				float weight = Weights(vert_idx, joint_idx);

				while (current_joint_idx != -1)
				{
					Eigen::Vector3f vertice_position(vertices_update_(vert_idx, 0), vertices_update_(vert_idx, 1), vertices_update_(vert_idx, 2));
					Eigen::Vector3f joint_position(Joints[current_joint_idx].CorrespondingPosition(0), Joints[current_joint_idx].CorrespondingPosition(1), Joints[current_joint_idx].CorrespondingPosition(2));

					Eigen::Vector3f x_axis_position(Joints[current_joint_idx].CorrespondingAxis[0](0), Joints[current_joint_idx].CorrespondingAxis[0](1), Joints[current_joint_idx].CorrespondingAxis[0](2));
					Eigen::Vector3f y_axis_position(Joints[current_joint_idx].CorrespondingAxis[1](0), Joints[current_joint_idx].CorrespondingAxis[1](1), Joints[current_joint_idx].CorrespondingAxis[1](2));
					Eigen::Vector3f z_axis_position(Joints[current_joint_idx].CorrespondingAxis[2](0), Joints[current_joint_idx].CorrespondingAxis[2](1), Joints[current_joint_idx].CorrespondingAxis[2](2));

					Eigen::Vector3f w_x, w_y, w_z;
					Eigen::Vector3f S;

					w_x << (x_axis_position - joint_position);
					w_y << (y_axis_position - joint_position);
					w_z << (z_axis_position - joint_position);

					w_x.normalize();
					w_y.normalize();
					w_z.normalize();

					S << (vertice_position - joint_position);


					int params_len = Joints[current_joint_idx].params_length;
					for (int idx = 0; idx < params_len; idx++)
					{
						Eigen::Vector3f result;
						int params_idx = Joints[current_joint_idx].params_index[idx];


						switch (Joints[current_joint_idx].params_type[idx])
						{
						case dof_type(x_axis_rotate): {
							result << kk*w_x.cross(S);
							jacobian_correspond(cor_id * 3, params_idx) += weight*result(0);
							jacobian_correspond(cor_id * 3 + 1, params_idx) += weight*result(1);
							jacobian_correspond(cor_id * 3 + 2, params_idx) += weight*result(2);
							break;
						}
						case dof_type(y_axis_rotate): {
							result << kk*w_y.cross(S);
							jacobian_correspond(cor_id * 3, params_idx) += weight*result(0);
							jacobian_correspond(cor_id * 3 + 1, params_idx) += weight*result(1);
							jacobian_correspond(cor_id * 3 + 2, params_idx) += weight*result(2);
							break;
						}
						case dof_type(z_axis_rotate): {
							result << kk*w_z.cross(S);
							jacobian_correspond(cor_id * 3, params_idx) += weight*result(0);
							jacobian_correspond(cor_id * 3 + 1, params_idx) += weight*result(1);
							jacobian_correspond(cor_id * 3 + 2, params_idx) += weight*result(2);
							break;
						}
						case dof_type(x_axis_trans): {
							result << 1, 0, 0;
							jacobian_correspond(cor_id * 3, params_idx) += weight*result(0);
							jacobian_correspond(cor_id * 3 + 1, params_idx) += weight*result(1);
							jacobian_correspond(cor_id * 3 + 2, params_idx) += weight*result(2);
							break;
						}
						case dof_type(y_axis_trans): {
							result << 0, 1, 0;
							jacobian_correspond(cor_id * 3, params_idx) += weight*result(0);
							jacobian_correspond(cor_id * 3 + 1, params_idx) += weight*result(1);
							jacobian_correspond(cor_id * 3 + 2, params_idx) += weight*result(2);
							break;
						}
						case dof_type(z_axis_trans): {
							result << 0, 0, 1;
							jacobian_correspond(cor_id * 3, params_idx) += weight*result(0);
							jacobian_correspond(cor_id * 3 + 1, params_idx) += weight*result(1);
							jacobian_correspond(cor_id * 3 + 2, params_idx) += weight*result(2);
							break;
						}
						}


					}
					current_joint_idx = Joints[current_joint_idx].parent_joint_index;
				}
			}
		}
	}
}

void HandModel::MoveToDownSamoleCorrespondingVertices(pcl::PointCloud<pcl::PointXYZ>& p, std::vector<int>& cor,int *idx_img, bool with_sil)
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

	Eigen::MatrixXf J = jacobian_correspond;
	Eigen::MatrixXf Jt = jacobian_correspond.transpose();

	Eigen::VectorXf e_sil;
	Eigen::MatrixXf J_sil = Compute_Silhouette_Limited(e_sil, idx_img);

	Eigen::VectorXf e_limit;
	Eigen::MatrixXf J_limit = Compute_joint_Limited(e_limit);

	int omiga_limit = 50;

	Eigen::Matrix<float, 27, 27> D = Eigen::Matrix<float, 27, 27>::Identity(27, 27);
	int rotate_omiga = 20;
	int abound_omiga = 100;

	D(0, 0) = rotate_omiga;
	D(1, 1) = abound_omiga;
	D(2, 2) = rotate_omiga;

	D(3, 3) = rotate_omiga;
	D(4, 4) = abound_omiga;
	D(5, 5) = rotate_omiga;

	D(6, 6) = rotate_omiga;
	D(7, 7) = abound_omiga;
	D(8, 8) = rotate_omiga;

	D(9, 9) = rotate_omiga;
	D(10, 10) = abound_omiga;
	D(11, 11) = rotate_omiga;

	D(12, 12) = rotate_omiga;
	D(13, 13) = rotate_omiga;
	D(14, 14) = rotate_omiga;

	D(15, 15) = rotate_omiga;
	D(16, 16) = rotate_omiga;
	D(17, 17) = rotate_omiga;

	D(18, 18) = rotate_omiga;
	D(19, 19) = rotate_omiga;
	D(20, 20) = rotate_omiga;
	D(21, 21) = rotate_omiga;
	D(22, 22) = rotate_omiga;
	D(23, 23) = rotate_omiga;
	D(24, 24) = 0.01;
	D(25, 25) = 0.01;
	D(26, 26) = 0.01;

	Eigen::VectorXf dAngles = Eigen::VectorXf::Zero(NumberofParams, 1);


	if (with_sil)
	{
		MatrixXf JtJ = Jt*J + D + omiga_limit*J_limit.transpose()*J_limit + 5*J_sil.transpose()*J_sil;
		VectorXf JTe = Jt*e + omiga_limit*e_limit +5*J_sil.transpose()*e_sil;
		dAngles = JtJ.colPivHouseholderQr().solve(JTe);
	}
	else
	{
		MatrixXf JtJ = Jt*J + D + omiga_limit*J_limit.transpose()*J_limit;// + J_sil.transpose()*J_sil;
		VectorXf JTe = Jt*e + omiga_limit*e_limit;// +J_sil.transpose()*e_sil;
		dAngles = JtJ.colPivHouseholderQr().solve(JTe);
	}


	//cout << "change is " << J_sil.transpose()*J_sil << endl;

	for (int i = 0; i < NumberofParams; i++)
		Params[i] += dAngles(i);


	Updata(Params);



	Eigen::VectorXf e_updata = Eigen::VectorXf::Zero(3 * NumofCorrespond, 1);
	for (int i = 0; i < NumofCorrespond; i++)
	{
		e_updata(i * 3 + 0) = p.points[i].x - vertices_update_(Visible_vertices_index[cor[i]], 0);
		e_updata(i * 3 + 1) = p.points[i].x - vertices_update_(Visible_vertices_index[cor[i]], 1);
		e_updata(i * 3 + 2) = p.points[i].x - vertices_update_(Visible_vertices_index[cor[i]], 2);
	}

	if (e_updata.norm() < 1)
	{
		Solved = true;
	}
	else
	{
		Solved = false;
	}

}

void HandModel::Compute_normal_And_visibel_vertices()
{
	Visible_vertices.clear();
	Visible_vertices_index.clear();
	Visible_vertices_2D.clear();

	Vertices_normal.setZero();
	Vector3f A, B, C, BA, BC;
	for (int i = 0; i < NumofFaces; i++)
	{
		//这里我假设，如果假设错了，那么叉乘时候，就BC*BA变成BA*BC
		//            A
		//          /  \
				//         B — C
		A << vertices_update_(FaceIndex(i, 0), 0), vertices_update_(FaceIndex(i, 0), 1), vertices_update_(FaceIndex(i, 0), 2);
		B << vertices_update_(FaceIndex(i, 1), 0), vertices_update_(FaceIndex(i, 1), 1), vertices_update_(FaceIndex(i, 1), 2);
		C << vertices_update_(FaceIndex(i, 2), 0), vertices_update_(FaceIndex(i, 2), 1), vertices_update_(FaceIndex(i, 2), 2);

		BC << C - B;
		BA << A - B;

		Vector3f nom(BA.cross(BC));

		nom.normalize();

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

	for (int i = 0; i < Vertices_normal.rows(); i++)
	{
		Vertices_normal.row(i).normalize();

		if (-(Vertices_normal(i, 2)) >= 0)
		{
			Eigen::Vector3f visible_v(vertices_update_(i, 0), vertices_update_(i, 1), vertices_update_(i, 2));
			Eigen::Vector2i visible_2D;
			visible_2D << (int)(camera->world_to_depth_image(visible_v)(0)), (int)(camera->world_to_depth_image(visible_v)(1));

			Visible_vertices.push_back(visible_v);
			Visible_vertices_2D.push_back(visible_2D);
			Visible_vertices_index.push_back(i);
		}
	}

	//cout << "visible vertices find done!" << endl;
}


MatrixXf HandModel::Compute_joint_Limited(Eigen::VectorXf & e_limit)
{
	MatrixXf J_limit = MatrixXf::Zero(27, 27);

	e_limit = Eigen::VectorXf::Zero(27, 1);
	//0
	{
		int i = 0;
		float q_max = 90;
		float q_min = 0;
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

	//1
	{
		int i = 1;
		float q_max = 20;
		float q_min = -20;
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

	//2
	{
		int i = 2;
		float q_max = 90;
		float q_min = 0;
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

	//3
	{
		int i = 3;
		float q_max = 90;
		float q_min = 0;
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

	//4
	{
		int i = 4;
		float q_max = 20;
		float q_min = -20;
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

	//5
	{
		int i = 5;
		float q_max = 90;
		float q_min = 0;
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


	//6
	{
		int i = 6;
		float q_max = 90;
		float q_min = 0;
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

	//7
	{
		int i = 7;
		float q_max = 0;
		float q_min = 0;
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


	//8
	{
		int i = 8;
		float q_max = 90;
		float q_min = 0;
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


	//9
	{
		int i = 9;
		float q_max = 90;
		float q_min = 0;
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


	//10
	{
		int i = 10;
		float q_max = 20;
		float q_min = -20;
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


	//11
	{
		int i = 11;
		float q_max = 90;
		float q_min = 0;
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

	//14
	{
		int i = 14;
		float q_max = 90;
		float q_min = 0;
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

	//19
	{
		int i = 19;
		float q_max = 60;
		float q_min = 0;
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

	//20
	{
		int i = 20;
		float q_max = 90;
		float q_min = 0;
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


	//21
	{
		int i = 21;
		float q_max = 90;
		float q_min = 0;
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

	//22
	{
		int i = 22;
		float q_max = 90;
		float q_min = 0;
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

	//23
	{
		int i = 23;
		float q_max = 90;
		float q_min = 0;
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


	return J_limit;
}

MatrixXf HandModel::Compute_Silhouette_Limited(Eigen::VectorXf & e_sil,int *idx_img)
{
	vector<pair<Eigen::Matrix<float, 2, 27>, Eigen::Vector2f>> outside_silhouette;

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
				Eigen::Matrix<float, 2, 27> J_2D = MatrixXf::Zero(2, 27);    //J_2D = J_perspective * J_3D

				float kk = 3.141592f / 180.0;
				for (int joint_idx = 0; joint_idx < NumofJoints; joint_idx++)
				{
					if (Weights(vert_idx, joint_idx) > 0.0f)
					{
						//do some thing follow the kanimatic chains
						int current_joint_idx = joint_idx;
						float weight = Weights(vert_idx, joint_idx);

						while (current_joint_idx != -1)
						{
							Eigen::Vector3f vertice_position(vertices_update_(vert_idx, 0), vertices_update_(vert_idx, 1), vertices_update_(vert_idx, 2));
							Eigen::Vector3f joint_position(Joints[current_joint_idx].CorrespondingPosition(0), Joints[current_joint_idx].CorrespondingPosition(1), Joints[current_joint_idx].CorrespondingPosition(2));

							Eigen::Vector3f x_axis_position(Joints[current_joint_idx].CorrespondingAxis[0](0), Joints[current_joint_idx].CorrespondingAxis[0](1), Joints[current_joint_idx].CorrespondingAxis[0](2));
							Eigen::Vector3f y_axis_position(Joints[current_joint_idx].CorrespondingAxis[1](0), Joints[current_joint_idx].CorrespondingAxis[1](1), Joints[current_joint_idx].CorrespondingAxis[1](2));
							Eigen::Vector3f z_axis_position(Joints[current_joint_idx].CorrespondingAxis[2](0), Joints[current_joint_idx].CorrespondingAxis[2](1), Joints[current_joint_idx].CorrespondingAxis[2](2));

							Eigen::Vector3f w_x, w_y, w_z;
							Eigen::Vector3f S;

							w_x << (x_axis_position - joint_position);
							w_y << (y_axis_position - joint_position);
							w_z << (z_axis_position - joint_position);

							w_x.normalize();
							w_y.normalize();
							w_z.normalize();

							S << (vertice_position - joint_position);


							int params_len = Joints[current_joint_idx].params_length;
							for (int idx = 0; idx < params_len; idx++)
							{
								Eigen::Vector3f result;
								Eigen::Vector2f result_2d;
								int params_idx = Joints[current_joint_idx].params_index[idx];


								switch (Joints[current_joint_idx].params_type[idx])
								{
								case dof_type(x_axis_rotate): {
									result << kk*w_x.cross(S);
									result_2d = J_perspective*result;
									J_2D(0, params_idx) += weight*result_2d(0);
									J_2D(1, params_idx) += weight*result_2d(1);
									break;
								}
								case dof_type(y_axis_rotate): {
									result << kk*w_y.cross(S);
									result_2d = J_perspective*result;
									J_2D(0, params_idx) += weight*result_2d(0);
									J_2D(1, params_idx) += weight*result_2d(1);
									break;
								}
								case dof_type(z_axis_rotate): {
									result << kk*w_z.cross(S);
									result_2d = J_perspective*result;
									J_2D(0, params_idx) += weight*result_2d(0);
									J_2D(1, params_idx) += weight*result_2d(1);
									break;
								}
								case dof_type(x_axis_trans): {
									result << 1, 0, 0;
									result_2d = J_perspective*result;
									J_2D(0, params_idx) += weight*result_2d(0);
									J_2D(1, params_idx) += weight*result_2d(1);
									break;
								}
								case dof_type(y_axis_trans): {
									result << 0, 1, 0;
									result_2d = J_perspective*result;
									J_2D(0, params_idx) += weight*result_2d(0);
									J_2D(1, params_idx) += weight*result_2d(1);
									break;
								}
								case dof_type(z_axis_trans): {
									result << 0, 0, 1;
									result_2d = J_perspective*result;
									J_2D(0, params_idx) += weight*result_2d(0);
									J_2D(1, params_idx) += weight*result_2d(1);
									break;
								}
								}


							}
							current_joint_idx = Joints[current_joint_idx].parent_joint_index;
						}
					}
				}

				J_and_e.first = J_2D;

				outside_silhouette.push_back(J_and_e);
			}
		}
	}

	int size = outside_silhouette.size();

	if (size > 0)
	{
		MatrixXf J_sil = MatrixXf::Zero(2 * size, 27);

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
		cout << "all rendered pixel are in the silhouette " << endl;

		MatrixXf J_sil = MatrixXf::Zero(1, 27);

		e_sil = Eigen::VectorXf::Zero(1, 1);

		return J_sil;
	}
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