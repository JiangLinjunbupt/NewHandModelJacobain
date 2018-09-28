#include "GL/freeglut.h"         //OpenGL的头文件Includ顺序会导致出现编译错误
#include "MyKinect.h"
#include "PointCloud.h"
#include"HandModel.h"
#include <time.h>
#include <fstream>
#include <tchar.h>
#include <pcl/kdtree/kdtree_flann.h>

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

Control control;
HandModel *handmodel = new HandModel();


Camera *camera = new Camera();
myKinect mykinect(camera);
DataFrame dataframe;
HandFinder handfinder(camera);
PointCloud pointcloud;
clock_t start;
clock_t ends_clock;

bool with_Kinect = false;
//共享内存的相关定义
HANDLE hMapFile;
LPCTSTR pBuf;
#define BUF_SIZE 1024
TCHAR szName[] = TEXT("Global\\MyFileMappingObject");    //指向同一块共享内存的名字
float *GetSharedMemeryPtr;
float *GetGloveData = new float[27];


pcl::PointCloud<pcl::PointXYZ>::Ptr Handmodel_visible_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<int> cloud_correspond;
int itr = 0;

void load_handmodel_visible_cloud(pcl::PointCloud<pcl::PointXYZ>& cloud, HandModel &hm)
{
	cloud.points.clear();
	for (int i = 0; i < hm.Visible_vertices.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = hm.Visible_vertices[i](0);
		p.y = hm.Visible_vertices[i](1);
		p.z = hm.Visible_vertices[i](2);
		cloud.points.push_back(p);
	}
}
void find_correspondences(std::vector<int> & correspondences_out)
{
	correspondences_out.resize(pointcloud.pointcloud_downsample.points.size());

	pcl::KdTreeFLANN<pcl::PointXYZ> search_kdtree;
	search_kdtree.setInputCloud(Handmodel_visible_cloud);

	const int k = 1;
	std::vector<int> k_indices(k);
	std::vector<float> k_squared_distances(k);
	for (size_t i = 0; i < pointcloud.pointcloud_downsample.points.size(); ++i)
	{
		search_kdtree.nearestKSearch(pointcloud.pointcloud_downsample, i, k, k_indices, k_squared_distances);
		correspondences_out[i] = k_indices[0];
	}
}

void Save_Point_cloud()
{
	for (int i = 0; i < 24; i++)
	{
		GetGloveData[i] = GetSharedMemeryPtr[i];
	}
	GetGloveData[24] = pointcloud.PointCloud_center_x;
	GetGloveData[25] = pointcloud.PointCloud_center_y;
	GetGloveData[26] = pointcloud.PointCloud_center_z;

	ofstream f;
	f.open("Kinect_Point_cloud4.txt", ios::out);
	f << pointcloud.pointcloud_downsample.points.size() << endl;
	for (int i = 0; i < pointcloud.pointcloud_downsample.points.size(); i++)
	{
		f << pointcloud.pointcloud_downsample.points[i].x << "  " << pointcloud.pointcloud_downsample.points[i].y << "  " << pointcloud.pointcloud_downsample.points[i].z << endl;
	}
	f.close();
	std::cout << " Kinect Point Cloud save success , the Point Num is : " << pointcloud.pointcloud_downsample.points.size() << endl;


	ofstream f2;
	f2.open("gloveParams4.txt", ios::out);
	for (int i = 0; i < 27; i++)
	{
		f2 << GetGloveData[i] << "  ";
	}
	f2 << endl;
	f2.close();
	std::cout << " glove Params save success " << endl;

}

#pragma region OpenGL

#pragma region  Keybroad_event(show mesh or not)


/* executed when a regular key is pressed */
void keyboardDown(unsigned char key, int x, int y) {
	switch (key) {
	case  27:   // ESC
		exit(0);
	case 's':
		Save_Point_cloud();
		break;
	case 'b':
		with_Kinect = true;
		break;
	case 'e':
		with_Kinect = false;
		break;
	}
}

/* executed when a regular key is released */
void keyboardUp(unsigned char key, int x, int y) {}

#pragma endregion  Keybroad_event(show mesh or not)


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

/* executed when the mouse moves to position ('x', 'y') */
/* render the scene */
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



																												 ///* render the scene here */
	{
		glPointSize(2);
		glBegin(GL_POINTS);
		glColor3d(1.0, 1.0, 0.0);
		for (int i = 0; i < handmodel->NumofVertices; i++) {
			glVertex3d(handmodel->vertices_update_(i, 0), handmodel->vertices_update_(i, 1), handmodel->vertices_update_(i, 2));
		}
		glEnd();


		//glPointSize(2);
		//glBegin(GL_POINTS);
		//glColor3d(0.0, 1.0, 0.0);
		//for (int i = 0; i < handmodel->NumofVertices; i++) {
		//	glVertex3d(handmodel->Target_vertices(i, 0), handmodel->Target_vertices(i, 1), handmodel->Target_vertices(i, 2));
		//}
		//glEnd();


		//for (int i = 0; i < handmodel->NumofVertices; i = i+10) {
		//	glVertex3d(handmodel->Target_vertices(i, 0), handmodel->Target_vertices(i, 1), handmodel->Target_vertices(i, 2));
		//	glLineWidth(1);
		//	glColor3f(1.0, 1.0, 1.0);
		//	glBegin(GL_LINES);
		//	glVertex3d(handmodel->Target_vertices(i, 0), handmodel->Target_vertices(i, 1), handmodel->Target_vertices(i, 2));
		//	glVertex3d(handmodel->vertices_update_(i, 0), handmodel->vertices_update_(i, 1), handmodel->vertices_update_(i, 2));
		//	glEnd();
		//}
	}


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

		//glLineWidth(2);
		//glColor3f(1.0, 1.0, 1);
		//glBegin(GL_LINES);
		//glVertex3f(handmodel->Joints[i].CorrespondingPosition(0), handmodel->Joints[i].CorrespondingPosition(1), handmodel->Joints[i].CorrespondingPosition(2));
		//glVertex3f(handmodel->Target_joints(i, 0), handmodel->Target_joints(i, 1), handmodel->Target_joints(i, 2));
		//glEnd();

		//画线结束
	}


	//for (int i = 0; i < handmodel->NumofJoints; i++) {
	//	//画点开始 
	//	glColor3f(0.0, 1.0, 0.0);
	//	glPushMatrix();
	//	glTranslatef(handmodel->Target_joints(i,0), handmodel->Target_joints(i,1), handmodel->Target_joints(i,2));
	//	glutSolidSphere(5, 31, 10);
	//	glPopMatrix();

	//	//画点结束，使用push和popmatrix是因为保证每个关节点的偏移都是相对于全局坐标中心点做的变换。
	//}

	//glPointSize(2);
	//glBegin(GL_POINTS);
	//glColor3d(1.0, 1.0, 0.0);
	//for (int i = 0; i < handmodel->Visible_vertices.size(); i++) {
	//	glVertex3d(handmodel->Visible_vertices[i](0), handmodel->Visible_vertices[i](1), handmodel->Visible_vertices[i](2));
	//}
	//glEnd();


	glPointSize(2);
	glBegin(GL_POINTS);
	glColor3d(1.0, 0.0, 0.0);
	for (int i = 0; i < pointcloud.pointcloud_from_depth.points.size(); i++) {
		glVertex3d(pointcloud.pointcloud_from_depth.points[i].x, pointcloud.pointcloud_from_depth.points[i].y, pointcloud.pointcloud_from_depth.points[i].z);
	}
	glEnd();


	//glPointSize(2);
	//glBegin(GL_POINTS);
	//glColor3d(1.0, 0.0, 0.0);
	//for (int i = 0; i < downSample_visible_cloud->points.size(); i++) {
	//	glVertex3d(downSample_visible_cloud->points[i].x, downSample_visible_cloud->points[i].y, downSample_visible_cloud->points[i].z);
	//}
	//glEnd();


	//for (int i = 0; i < cloud_correspond.size(); i++)
	//{
	//	glLineWidth(2);
	//	glColor3f(1.0, 1.0, 1);
	//	glBegin(GL_LINES);
	//	glVertex3d(pointcloud.pointcloud_downsample.points[i].x, pointcloud.pointcloud_downsample.points[i].y, pointcloud.pointcloud_downsample.points[i].z);
	//	glVertex3d(Handmodel_visible_cloud->points[cloud_correspond[i]].x, Handmodel_visible_cloud->points[cloud_correspond[i]].y, Handmodel_visible_cloud->points[cloud_correspond[i]].z);
	//	glEnd();
	//}


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

		for (int i = 0; i < 24; i++)
		{
			handmodel->Params[i] = GetSharedMemeryPtr[i];
		}
		handmodel->Params[16] = -handmodel->Params[16];
		handmodel->Params[17] = -handmodel->Params[17];
		handmodel->Params[18] = -handmodel->Params[18];

		handmodel->Params[24] = pointcloud.PointCloud_center_x;
		handmodel->Params[25] = pointcloud.PointCloud_center_y;
		handmodel->Params[26] = pointcloud.PointCloud_center_z;
		handmodel->Updata(handmodel->Params);
	}


	if (with_Kinect)
	{
		while (itr < 20)
		{
			load_handmodel_visible_cloud(*Handmodel_visible_cloud, *handmodel);
			find_correspondences(cloud_correspond);

			handmodel->MoveToDownSamoleCorrespondingVertices(pointcloud.pointcloud_downsample, cloud_correspond);

			itr++;
		}
	}

	//load_handmodel_visible_cloud(*Handmodel_visible_cloud, *handmodel);
	//find_correspondences(cloud_correspond);
	itr = 0;



	//pointcloud.DepthMatToPointCloud(dataframe.depth, &handfinder);

	/*cv::imshow("color_show", dataframe.color);
	cv::imshow("depth_show", handfinder.sensor_hand_silhouette);
	cv::waitKey(1);*/

	//cv::imshow("depth_show", handfinder.sensor_hand_silhouette);

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