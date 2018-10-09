#include "GL/freeglut.h"         //OpenGL的头文件Includ顺序会导致出现编译错误
#include "MyKinect.h"
#include "PointCloud.h"
#include"HandModel.h"
#include <time.h>
#include <fstream>
#include <tchar.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<math.h>
#include "DistanceTransform.h"

using namespace std;
struct Control {
	int x;
	int y;
	bool mouse_click;
	GLfloat rotx;
	GLfloat roty;
	double gx;
	double gy;
	double gz;

	Control() :x(0), y(0), rotx(0.0), roty(0.0), mouse_click(false),
		gx(0), gy(0), gz(0) {

	}
};
Camera *camera = new Camera();

Control control;
HandModel *handmodel = new HandModel(camera);

myKinect mykinect(camera);
DataFrame dataframe;
HandFinder handfinder(camera);
PointCloud pointcloud;
clock_t start;
clock_t ends_clock;

bool with_Kinect = false;
bool with_sil = false;
bool show_mesh = true;
bool has_glove = false;
//共享内存的相关定义
HANDLE hMapFile;
LPCTSTR pBuf;
#define BUF_SIZE 1024
TCHAR szName[] = TEXT("Global\\MyFileMappingObject");    //指向同一块共享内存的名字
float *GetSharedMemeryPtr;
float *GetGloveData = new float[26];


pcl::PointCloud<pcl::PointXYZ>::Ptr Handmodel_visible_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<int> cloud_correspond;
int itr = 0;

void load_handmodel_visible_cloud(pcl::PointCloud<pcl::PointXYZ>& cloud, HandModel &hm)
{
	cloud.points.clear();
	if (hm.Visible_vertices.size() == 0)
	{
		cerr << "hand model have no Visible_vertices!!!!  please cheak if some thing wrong ? " << endl;
	}
	else
	{
		for (int i = 0; i < hm.Visible_vertices.size(); i++)
		{
			pcl::PointXYZ p;
			p.x = hm.Visible_vertices[i](0);
			p.y = hm.Visible_vertices[i](1);
			p.z = hm.Visible_vertices[i](2);
			cloud.points.push_back(p);
		}
	}
}
void find_correspondences(std::vector<int> & correspondences_out)
{
	if (Handmodel_visible_cloud->points.size() == 0)
	{
		cerr << "hand model have no Visible_vertices!!!!  please cheak if some thing wrong ? " << endl;
	}
	else
	{
		correspondences_out.resize(pointcloud.pointcloud_downsample.points.size());

		pcl::KdTreeFLANN<pcl::PointXYZ> search_kdtree;
		search_kdtree.setInputCloud(Handmodel_visible_cloud);

		const int k = 1;
		std::vector<int> k_indices(k);
		std::vector<float> k_squared_distances(k);
		for (int i = 0; i < pointcloud.pointcloud_downsample.points.size(); ++i)
		{
			search_kdtree.nearestKSearch(pointcloud.pointcloud_downsample, i, k, k_indices, k_squared_distances);
			correspondences_out[i] = k_indices[0];
		}
	}
}

void MixShowResult(cv::Mat input1, cv::Mat input2);

void judge_initParams();
float Comput_e(float* params);

#pragma region OpenGL

#pragma region  Keybroad_event(show mesh or not)


/* executed when a regular key is pressed */
void keyboardDown(unsigned char key, int x, int y) {
	switch (key) {
	case  27:   // ESC
		exit(0);
	case 'b':
		with_Kinect = true;
		break;
	case 'e':
		with_Kinect = false;
		with_sil = false;
		break;
	case 'w':
		with_sil = true;
		break;
	case 'n':
		with_sil = false;
		break;
	case 'm':
		show_mesh = !show_mesh;
		break;
	}
}

/* executed when a regular key is released */
void keyboardUp(unsigned char key, int x, int y) {}

#pragma endregion  Keybroad_event(show mesh or not)

void light() {
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);
	// 定义太阳光源，它是一种白色的光源  
	GLfloat sun_light_position[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	GLfloat sun_light_ambient[] = { 0.25f, 0.25f, 0.15f, 1.0f };
	GLfloat sun_light_diffuse[] = { 0.7f, 0.7f, 0.55f, 1.0f };
	GLfloat sun_light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_POSITION, sun_light_position); //指定第0号光源的位置   
	glLightfv(GL_LIGHT0, GL_AMBIENT, sun_light_ambient); //GL_AMBIENT表示各种光线照射到该材质上，  
														 //经过很多次反射后最终遗留在环境中的光线强度（颜色）  
	glLightfv(GL_LIGHT0, GL_DIFFUSE, sun_light_diffuse); //漫反射后~~  
	glLightfv(GL_LIGHT0, GL_SPECULAR, sun_light_specular);//镜面反射后~~~  

	glEnable(GL_LIGHT0); //使用第0号光照   
}
/* reshaped window */
void reshape(int width, int height) {

	GLfloat fieldOfView = 90.0f;
	glViewport(0, 0, (GLsizei)width, (GLsizei)height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fieldOfView, (GLfloat)width / (GLfloat)height, 0.1, 500.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/* executed when button 'button' is put into state 'state' at screen position ('x', 'y') */
void mouseClick(int button, int state, int x, int y) {
	control.mouse_click = 1;
	control.x = x;
	control.y = y;
}

/* render the scene */

void draw_Mesh()
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	{
		glColor3f(0.4, 0.8, 0.3);
		glBegin(GL_TRIANGLES);
		for (int i = 0; i < handmodel->NumofFaces; i++)
		{

			glNormal3f(handmodel->Face_norm[i](0), handmodel->Face_norm[i](1), handmodel->Face_norm[i](2));

			glVertex3f(handmodel->vertices_update_(handmodel->FaceIndex(i, 0), 0), handmodel->vertices_update_(handmodel->FaceIndex(i, 0), 1), handmodel->vertices_update_(handmodel->FaceIndex(i, 0), 2));
			glVertex3f(handmodel->vertices_update_(handmodel->FaceIndex(i, 1), 0), handmodel->vertices_update_(handmodel->FaceIndex(i, 1), 1), handmodel->vertices_update_(handmodel->FaceIndex(i, 1), 2));
			glVertex3f(handmodel->vertices_update_(handmodel->FaceIndex(i, 2), 0), handmodel->vertices_update_(handmodel->FaceIndex(i, 2), 1), handmodel->vertices_update_(handmodel->FaceIndex(i, 2), 2));
		}
		glEnd();
	}
}
void draw_Vertex()
{
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
	glPointSize(2);
	glBegin(GL_POINTS);
	glColor3d(1.0, 1.0, 0.0);
	for (int i = 0; i < handmodel->NumofVertices; i++) {
		glVertex3d(handmodel->vertices_update_(i, 0), handmodel->vertices_update_(i, 1), handmodel->vertices_update_(i, 2));
	}
	glEnd();
}
void draw_skeleton()
{
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
	//画骨架
	for (int i = 0; i < handmodel->NumofJoints; i++) {
		//画点开始 
		glColor3f(1.0, 0.0, 0.0);
		glPushMatrix();
		glTranslatef(handmodel->Joints[i].CorrespondingPosition(0), handmodel->Joints[i].CorrespondingPosition(1), handmodel->Joints[i].CorrespondingPosition(2));
		glutSolidSphere(5, 31, 10);
		glPopMatrix();

		//画点结束，使用push和popmatrix是因为保证每个关节点的偏移都是相对于全局坐标中心点做的变换。

		int parent_joint_index = handmodel->Joints[i].parent_joint_index;
		//画线开始  //不画wrist到arm的那条线
		if (parent_joint_index != -1) {
			glLineWidth(5);
			glColor3f(0.0, 1.0, 0);
			glBegin(GL_LINES);
			glVertex3f(handmodel->Joints[i].CorrespondingPosition(0), handmodel->Joints[i].CorrespondingPosition(1), handmodel->Joints[i].CorrespondingPosition(2));
			glVertex3f(handmodel->Joints[parent_joint_index].CorrespondingPosition(0), handmodel->Joints[parent_joint_index].CorrespondingPosition(1), handmodel->Joints[parent_joint_index].CorrespondingPosition(2));
			glEnd();
		}
	}
}
void draw_Visible_vertex()
{
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
	glPointSize(2);
	glBegin(GL_POINTS);
	glColor3d(1.0, 1.0, 0.0);
	for (int i = 0; i < handmodel->Visible_vertices.size(); i++) {
		glVertex3d(handmodel->Visible_vertices[i](0), handmodel->Visible_vertices[i](1), handmodel->Visible_vertices[i](2));
	}
	glEnd();

}
void draw_CloudPoint()
{
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
	glPointSize(2);
	glBegin(GL_POINTS);
	glColor3f(1.0f, 0.0f, 0.0f);
	for (int i = 0; i < pointcloud.pointcloud_from_depth.points.size(); i++) {
		glVertex3d(pointcloud.pointcloud_from_depth.points[i].x, pointcloud.pointcloud_from_depth.points[i].y, pointcloud.pointcloud_from_depth.points[i].z);
	}
	glEnd();
}
void draw_Cooresponding_connection()
{
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
	for (int i = 0; i < cloud_correspond.size(); i++)
	{
		glLineWidth(2);
		glColor3f(1.0, 1.0, 1);
		glBegin(GL_LINES);
		glVertex3d(pointcloud.pointcloud_downsample.points[i].x, pointcloud.pointcloud_downsample.points[i].y, pointcloud.pointcloud_downsample.points[i].z);
		glVertex3d(Handmodel_visible_cloud->points[cloud_correspond[i]].x, Handmodel_visible_cloud->points[cloud_correspond[i]].y, Handmodel_visible_cloud->points[cloud_correspond[i]].z);
		glEnd();
	}
}
void draw() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	gluPerspective(180, 1.5, -1000, 1000);
	glLoadIdentity();
	control.gx = handmodel->GlobalPosition(0);
	control.gy = handmodel->GlobalPosition(1);
	control.gz = handmodel->GlobalPosition(2);
	double r = 200;
	double x = r*sin(control.roty)*cos(control.rotx);
	double y = r*sin(control.roty)*sin(control.rotx);
	double z = r*cos(control.roty);
	//cout<< x <<" "<< y <<" " << z<<endl;
	gluLookAt(x + control.gx, y + control.gy, z + control.gz, control.gx, control.gy, control.gz, 0.0, 1.0, 0.0);//个人理解最开始是看向-z的，之后的角度是在global中心上叠加的，所以要加


	draw_Mesh();
	draw_skeleton();
	draw_CloudPoint();
	if (with_Kinect)
	{
		draw_Cooresponding_connection();
	}

	glFlush();
	glutSwapBuffers();
}


void mouseMotion(int x, int y) {
	control.rotx = (x - control.x)*0.05;
	control.roty = (y - control.y)*0.05;

	//cout<< control.rotx <<" " << control.roty << endl;
	glutPostRedisplay();
}


void idle() {
	start = clock();

	if (itr == 0)
	{
		mykinect.fetch_data(dataframe, handfinder, pointcloud);

		//以下是distance_使用的测试代码，测试表明，能够找到最近的非零点
		//{
			//cv::Mat  hand = handfinder.sensor_hand_silhouette;     
			//circle(hand, cvPoint(100, 200), 10, Scalar(255, 255, 255), -1, 8);
			//circle(hand, cvPoint(400, 200), 10, Scalar(255, 255, 255), -1, 8);
			//int idx = handfinder.idxs_image[200*512+100];   
			//int idx2 = handfinder.idxs_image[200*512+400];
			//line(hand, cvPoint(100, 200), cvPoint(idx % 512, idx / 512), 255, 5);
			//line(hand, cvPoint(400, 200), cvPoint(idx2 % 512, idx2 / 512), 255, 5);
			//circle(hand, cvPoint(idx % 512, idx / 512), 10, Scalar(255, 255, 255), -1, 8);
			//circle(hand, cvPoint(idx2 % 512, idx2 / 512), 10, Scalar(255, 255, 255), -1, 8);
			//imshow("hand", hand);
			//cvWaitKey(1);
		//}

		has_glove = false;
		for (int i = 0; i < 26; i++)
		{
			if (GetSharedMemeryPtr[i] != 0)  has_glove = true;
			handmodel->init_Params[i] = GetSharedMemeryPtr[i];
		}

		handmodel->init_Params[0] = pointcloud.PointCloud_center_x;
		handmodel->init_Params[1] = pointcloud.PointCloud_center_y;
		handmodel->init_Params[2] = pointcloud.PointCloud_center_z;

		judge_initParams();
		handmodel->Updata(handmodel->Params);

	}


	if (with_Kinect)
	{
		//a function to judge init_Params from glovedata or form previous_Params
		//judge_initParams();

		while (!handmodel->Solved)
		{
			load_handmodel_visible_cloud(*Handmodel_visible_cloud, *handmodel);
			find_correspondences(cloud_correspond);

			handmodel->MoveToDownSampleCorrespondingVertices(itr, pointcloud.pointcloud_downsample, cloud_correspond, handfinder.idxs_image, with_sil,has_glove);

			itr++;
		}

		if (handmodel->track_failure)
		{
			handmodel->Updata(handmodel->init_Params);
		}
	}


	MixShowResult(handmodel->Generate_handimg(), handfinder.sensor_hand_silhouette);

	itr = 0;
	handmodel->Solved = false;
	ends_clock = clock();
	//cout << "Running Time : " << (double)(ends_clock - start) / CLOCKS_PER_SEC << endl;

	glutPostRedisplay();
}
/* initialize OpenGL settings */
void initGL(int width, int height) {

	reshape(width, height);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0f);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	light();

}

#pragma endregion 


int main(int argc, char** argv)
{

#pragma region SharedMemery
	hMapFile = CreateFileMapping(
		INVALID_HANDLE_VALUE,    // use paging file
		NULL,                    // default security
		PAGE_READWRITE,          // read/write access
		0,                       // maximum object size (high-order DWORD)
		BUF_SIZE,                // maximum object size (low-order DWORD)
		szName);                 // name of mapping object

	if (hMapFile == NULL)
	{
		_tprintf(TEXT("Could not create file mapping object (%d).\n"),
			GetLastError());
		return 1;
	}
	pBuf = (LPTSTR)MapViewOfFile(hMapFile,   // handle to map object
		FILE_MAP_ALL_ACCESS, // read/write permission
		0,
		0,
		BUF_SIZE);

	if (pBuf == NULL)
	{
		_tprintf(TEXT("Could not map view of file (%d).\n"),
			GetLastError());

		CloseHandle(hMapFile);

		return 1;
	}

	GetSharedMemeryPtr = (float*)pBuf;
#pragma endregion SharedMemery


	HRESULT hr = mykinect.InitializeDefaultSensor();
	pointcloud.camera = camera;


	if (FAILED(hr))
	{
		return -1;
	}

	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Interactron");

	// register glut call backs
	glutKeyboardFunc(keyboardDown);
	glutKeyboardUpFunc(keyboardUp);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseMotion);
	glutReshapeFunc(reshape);
	glutDisplayFunc(draw);
	glutIdleFunc(idle);
	glutIgnoreKeyRepeat(true); // ignore keys held down


	initGL(800, 600);

	glutMainLoop();

	return 0;

}


void MixShowResult(cv::Mat input1, cv::Mat input2)
{
	int height = input2.rows;
	int width = input2.cols;
	cv::Mat colored_input1 = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat colored_input2 = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat dst;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (input1.at<uchar>(i, j) != 0)
			{
				colored_input1.at < cv::Vec3b>(i, j)[0] = 0;
				colored_input1.at < cv::Vec3b>(i, j)[1] = 0;
				colored_input1.at < cv::Vec3b>(i, j)[2] = 255;
			}
			else
			{

				colored_input1.at < cv::Vec3b>(i, j)[0] = 255;
				colored_input1.at < cv::Vec3b>(i, j)[1] = 255;
				colored_input1.at < cv::Vec3b>(i, j)[2] = 255;

			}

			if (input2.at<uchar>(i, j) != 0)
			{
				colored_input2.at < cv::Vec3b>(i, j)[0] = 0;
				colored_input2.at < cv::Vec3b>(i, j)[1] = 255;
				colored_input2.at < cv::Vec3b>(i, j)[2] = 0;
			}
			else
			{

				colored_input2.at < cv::Vec3b>(i, j)[0] = 255;
				colored_input2.at < cv::Vec3b>(i, j)[1] = 255;
				colored_input2.at < cv::Vec3b>(i, j)[2] = 255;

			}

		}
	}

	cv::addWeighted(colored_input1, 0.5, colored_input2, 0.5, 0.0, dst);
	cv::imshow("Mixed Result", dst);
	cvWaitKey(1);
}


void judge_initParams()
{

	float merge_Params[26];
	
	merge_Params[0] = handmodel->previous_Params[0];
	merge_Params[1] = handmodel->previous_Params[1];
	merge_Params[2] = handmodel->previous_Params[2];
	merge_Params[3] = handmodel->previous_Params[3];
	merge_Params[4] = handmodel->previous_Params[4];
	merge_Params[5] = handmodel->previous_Params[5];

	for(int i = 6;i<26;i++)  merge_Params[i] = handmodel->init_Params[i];

	float e_merge, e_glove , e_previous = 0;

	e_glove = Comput_e(handmodel->init_Params);

	if (handmodel->previous_Params[2] != 0)
	{
		e_merge = Comput_e(merge_Params);
		e_previous = Comput_e(handmodel->previous_Params);

	/*	cout << "======>> " << e_glove << endl
			<< "==============>> " << e_merge << endl
			<< "====================>> " << e_previous << endl;*/

		if (e_merge <= e_previous && e_merge <= e_glove)
		{
			for (int i = 0; i < 26; ++i) handmodel->Params[i] = merge_Params[i];
			//cout << "using merge Params  :  " << e_merge << endl;
			return;
		}

		if (e_previous < e_merge && e_previous < e_glove)
		{
			for (int i = 0; i < 26; ++i) handmodel->Params[i] = handmodel->previous_Params[i];
			//cout << "using previous Params  :  " << e_previous << endl;
			return;
		}
	}


	for (int i = 0; i < 26; i++)
	{
		handmodel->Params[i] = handmodel->init_Params[i];
		//cout << "using init Params :  "<< e_glove <<endl;
	}

	return;

}

float Comput_e(float* params)
{
	
	handmodel->Updata(params);
	load_handmodel_visible_cloud(*Handmodel_visible_cloud, *handmodel);
	find_correspondences(cloud_correspond);

	int NumofCorrespond = cloud_correspond.size();

	Eigen::VectorXf e = Eigen::VectorXf::Zero(3 * NumofCorrespond, 1);

	for (int i = 0; i < NumofCorrespond; i++)
	{
		e(i * 3 + 0, 0) = pointcloud.pointcloud_downsample.points[i].x - handmodel->Visible_vertices[cloud_correspond[i]].x();
		e(i * 3 + 1, 0) = pointcloud.pointcloud_downsample.points[i].y - handmodel->Visible_vertices[cloud_correspond[i]].y();
		e(i * 3 + 2, 0) = pointcloud.pointcloud_downsample.points[i].z - handmodel->Visible_vertices[cloud_correspond[i]].z();
	}

	//int Visible_vertex_NUM = handmodel->Visible_vertices_2D.size();

	float e_final = 0;

	for (int i = 0; i < e.rows() / 3; ++i)
	{
		e_final += sqrt(e(i * 3 + 0, 0)*e(i * 3 + 0, 0) + e(i * 3 + 1, 0)*e(i * 3 + 1, 0) + e(i * 3 + 2, 0)*e(i * 3 + 2, 0));
	}
	//for (int i = 0; i < Visible_vertex_NUM; i++)
	//{
	//	Eigen::Vector2i pixel_2D_position(handmodel->Visible_vertices_2D[i]);
	//	if ((pixel_2D_position(0) >= 0) &&
	//		(pixel_2D_position(0) <= 512 - 1) &&
	//		(pixel_2D_position(1) >= 0) &&
	//		(pixel_2D_position(1) <= 424 - 1))
	//	{
	//		Eigen::Vector2i pixel_2D_closest;
	//		pixel_2D_closest << handfinder.idxs_image[pixel_2D_position(1) * 512 + pixel_2D_position(0)] % 512, handfinder.idxs_image[pixel_2D_position(1) * 512 + pixel_2D_position(0)] / 512;
	//		float closest_distance = (pixel_2D_closest(0) - pixel_2D_position(0))* (pixel_2D_closest(0) - pixel_2D_position(0)) + (pixel_2D_closest(1) - pixel_2D_position(1))*(pixel_2D_closest(1) - pixel_2D_position(1));
	//		e_2D += sqrt(closest_distance);
	//	}
	//}


	return e_final;
}