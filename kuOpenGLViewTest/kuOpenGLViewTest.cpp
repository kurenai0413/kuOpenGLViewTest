#pragma once
#include <opencv2/opencv.hpp>
#include <GLFW/glfw3.h>

#pragma comment(lib,"opencv_world310d.lib")
#pragma comment(lib,"glfw3.lib")
#pragma comment(lib,"opengl32.lib")

using namespace cv;
using namespace std;

// Setting an error callback
void init();
static void error_callback(int error, const char* description);
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
bool LoadIntrinsicParam(char * Filename);
bool LoadExtrinsicParam(char * Filename);
void DispIntrinsicParam();
void DispExtrinsicParam();
void DrawAxes(float length);

Mat					CamFrame;
Mat					IntrinsicMat;
Mat					DistParam;
Mat					RotationVec;
Mat					RotationMat;
Mat					TranslationVec;
Mat					ExtrinsicMat;
Mat					InvExtrinsicMat;

void main()
{
	glfwSetErrorCallback(error_callback);

	CamFrame = imread("CamFrame.bmp",1);

	init();

	if (!glfwInit())
		exit(EXIT_FAILURE);

	GLFWwindow * window = glfwCreateWindow(1280, 480, "Stereo", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(window);
	//glfwSwapInterval(1);		// the number of screen updates to wait from the time
	glfwSetKeyCallback(window, key_callback);

	while (!glfwWindowShouldClose(window))
	{
		// draw something here

		Mat					GLFlipedFrame;			// Fliped camera frame for GL display 
		flip(CamFrame, GLFlipedFrame, 0);

		glDrawPixels(640, 480, GL_BGR_EXT, GL_UNSIGNED_BYTE, GLFlipedFrame.ptr());

		glViewport(0, 0, 640, 480);
		
		DrawAxes(0.5);

		glfwSwapBuffers(window);
		glfwPollEvents();	// This function processes only those events that are already 
							// in the event queue and then returns immediately
	}

	glfwDestroyWindow(window);
	glfwTerminate();

	exit(EXIT_SUCCESS);
}

void init()
{
	int i, j;

	IntrinsicMat.create(3, 3, CV_32FC1);
	DistParam.create(1, 4, CV_32FC1);
	RotationVec.create(3, 1, CV_32FC1);
	RotationMat.create(3, 3, CV_32FC1);
	TranslationVec.create(3, 1, CV_32FC1);
	ExtrinsicMat.create(4, 4, CV_32FC1);
	InvExtrinsicMat.create(4, 4, CV_32FC1);

	LoadIntrinsicParam("IntParam_Vuzix.txt");
	DispIntrinsicParam();
	LoadExtrinsicParam("ExtParam_Vuzix.txt");
	DispExtrinsicParam();

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			ExtrinsicMat.at<float>(j, i) = RotationMat.at<float>(j, i);
			ExtrinsicMat.at<float>(j, 3) = TranslationVec.at<float>(j, 0);
			ExtrinsicMat.at<float>(3, j) = 0;
		}
	}
	ExtrinsicMat.at<float>(3, 3) = 1;

	invert(ExtrinsicMat, InvExtrinsicMat, DECOMP_LU);
}

static void error_callback(int error, const char* description)
{
	fputs(description, stderr);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
}

bool LoadIntrinsicParam(char * Filename)
{
	FILE	*	fp;
	errno_t		err;

	err = fopen_s(&fp, Filename, "r");
	if (err != 0)
	{
		return false;
	}
	else
	{
		for (int i = 0; i < 3; i++)
		{
			fscanf_s(fp, "%f %f %f\n", &IntrinsicMat.at<float>(i, 0),
									   &IntrinsicMat.at<float>(i, 1),
									   &IntrinsicMat.at<float>(i, 2));
		}
		fscanf_s(fp, "%f %f %f %f\n", &DistParam.at<float>(0, 0),
									  &DistParam.at<float>(0, 1),
									  &DistParam.at<float>(0, 2),
									  &DistParam.at<float>(0, 3));
		fclose(fp);

		return true;
	}
}

bool LoadExtrinsicParam(char * Filename)
{
	FILE	*	fp;
	errno_t		err;

	err = fopen_s(&fp, Filename, "r");
	if (err != 0)
	{
		return false;
	}
	else
	{
		for (int i = 0; i < 3; i++)
		{
			fscanf_s(fp, "%f %f %f %f\n", &RotationMat.at<float>(i, 0),
									      &RotationMat.at<float>(i, 1),
									      &RotationMat.at<float>(i, 2),
										  &TranslationVec.at<float>(i, 0));
		}
		fclose(fp);

		return true;
	}
	return false;
}

void DispIntrinsicParam()
{
	cout << "Intrinsic Parameters: " << endl;
	for (int i = 0; i < 3; i++)
	{
		cout << IntrinsicMat.at<float>(i, 0) << " " << IntrinsicMat.at<float>(i, 1) << " "
			 << IntrinsicMat.at<float>(i, 2) << endl;
	}
	cout << DistParam.at<float>(0, 0) << " " << DistParam.at<float>(0, 1) << " "
		 << DistParam.at<float>(0, 2) << " " << DistParam.at<float>(0, 3) << endl << endl;
}

void DispExtrinsicParam()
{
	cout << "Extrinsic Parameters: " << endl;
	for (int i = 0; i < 3; i++)
	{
		cout << RotationMat.at<float>(i, 0) << " " << RotationMat.at<float>(i, 1) << " "
			 << RotationMat.at<float>(i, 2) << endl;
	}
	cout << TranslationVec.at<float>(0, 0) << " " << TranslationVec.at<float>(1, 0) << " "
		 << TranslationVec.at<float>(2, 0) << endl << endl;
}

void DrawAxes(float length)
{
	glPushAttrib(GL_POLYGON_BIT | GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDisable(GL_LIGHTING);

	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(length, 0, 0); // X-axis => red

	glColor3f(0, 1, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, length, 0); // Y-axis => green

	glColor3f(0, 0, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, length); // Z-axis => blue
	glEnd();

	glPopAttrib();
}