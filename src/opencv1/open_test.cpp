#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "opencv2/imgproc.hpp"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <iostream>

using namespace cv;
using namespace std;

int H_MIN = 102;				/*int H_MIN = 33;*/
int H_MAX = 129;			/*int H_MAX = 79;*/
int S_MIN = 178;				/*int S_MIN = 36;*/
int S_MAX = 256;			/*int S_MAX = 115;*/
int V_MIN = 75;				/*int V_MIN = 93;*/
int V_MAX = 256;			/*int V_MAX = 227;*/
int E_FAC = 15;
int D_FAC = 5;
Point center;

void calibrationBars(int, void*) {}

Mat image_Calib(Mat &img) {
	Mat hsv, thresh;
	cvtColor(img, hsv, COLOR_BGR2HSV);
	inRange(hsv, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), thresh);
	GaussianBlur(thresh, thresh, Size(9, 9), 2, 2);
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(E_FAC, E_FAC));
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(D_FAC, D_FAC));
	erode(thresh, thresh, erodeElement);
	dilate(thresh, thresh, dilateElement);
	imshow("Calibrated", thresh);
	return thresh;
}

void contouring(Mat &color,Mat &thresh) {
	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;
	findContours(thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<vector<Point> >hull(contours.size());
	vector<Point> points;
	for (int i = 0; i < contours.size(); i++)
	{
		convexHull(Mat(contours[i]), hull[i], true);
	}
	Mat drawing = Mat::zeros(thresh.size(), CV_8UC3);
	Mat drawing_g;
	if (hull.size()>0) {
		double greatest_Area;
		double temp = 0;
		int contour_index;
		for (int i = 0;i < hull.size();i++)
		{
			greatest_Area = contourArea(hull[i], false);
			if (greatest_Area > temp)
			{
				temp = greatest_Area;
				contour_index = i;
			}
		}
		points = hull[contour_index];
		drawContours(color, hull, contour_index, Scalar(0, 255, 0), 3, 8, vector<Vec4i>(), 0, Point());
		//drawContours(color, contours, contour_index, Scalar(0, 255, 255), 2, 8, vector<Vec4i>(), 1, Point());
		//drawContours(drawing, hull, contour_index, Scalar(0, 255, 255), 2, 8, vector<Vec4i>(), 1, Point());
		cvtColor(drawing, drawing_g, COLOR_BGR2GRAY);
		for (int i = 0;i < points.size();i++) {
			circle(color, points[i], 5, Scalar(255, 0, 255), FILLED, LINE_8);
		}
	}
	long int col_sum = 0;
	long int row_sum = 0;
	long int count = 0;
	int nRows = drawing_g.rows;
	int nCols = drawing_g.cols*drawing_g.channels();
	uchar* p;
	for (int i = 0; i < nRows; i++)
	{
		p = drawing_g.ptr<uchar>(i);
		for (int j = 0; j < nCols; j++)
		{
			if (p[j] != 0) {
				col_sum += j;
				row_sum += i;
				count++;
			};
		}
	}
	if (count > 0) {
		center = Point(col_sum / count, row_sum / count);
	}
	else
	{
		center = Point(drawing_g.cols / 2, drawing_g.rows / 2);
	}
	circle(color, center, 5, Scalar(0, 0, 255), FILLED, LINE_8);
	//ROS_INFO_STREAM("Object is located at: <" <<center.x <<"," <<center.y <<"> ");
	//imshow("Contour", drawing);
}

int main(int argc, char **argv){
	ros::init(argc, argv,"object_pos");
	ros::NodeHandle nh;
	ros::Publisher pubCenter=nh.advertise<std_msgs::Float32MultiArray>("std_msgs/Float32MultiArray", 1000);
	ROS_INFO("main start");
	double c,r;
	VideoCapture cap(0);
	if (!cap.isOpened()){
		ROS_ERROR("camera not open");
		return -1;
	}
	Mat calibrated; namedWindow("Calibration", WINDOW_NORMAL);
	createTrackbar("H_MIN", "Calibration", &H_MIN, H_MAX, calibrationBars);
	createTrackbar("H_MAX", "Calibration", &H_MAX, H_MAX, calibrationBars);
	createTrackbar("S_MIN", "Calibration", &S_MIN, S_MAX, calibrationBars);
	createTrackbar("S_MAX", "Calibration", &S_MAX, S_MAX, calibrationBars);
	createTrackbar("V_MIN", "Calibration", &V_MIN, V_MAX, calibrationBars);
	createTrackbar("V_MAX", "Calibration", &V_MAX, V_MAX, calibrationBars);
	createTrackbar("E_FAC", "Calibration", &E_FAC, 25, calibrationBars);
	createTrackbar("D_FAC", "Calibration", &D_FAC, 25, calibrationBars);
	ros::Rate rate(10);
	while(ros::ok())
	{
		std_msgs::Float32MultiArray msg;
		Mat color;
		//ROS_INFO("loop start");
		cap>>color;
		c = color.cols;
		r = color.rows;
		//ROS_INFO("image captured");
		calibrated = image_Calib(color);
		contouring(color, calibrated);
		imshow("Original", color);
		msg.data.push_back(center.x);
		msg.data.push_back(center.y);
		msg.data.push_back(c);
		msg.data.push_back(r);
		pubCenter.publish(msg);
		waitKey(30);
		
		ros::spinOnce();
	}
	return 0;
}
