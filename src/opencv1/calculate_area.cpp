#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include "opencv2/imgproc.hpp"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <fstream>
#include <iostream>

using namespace cv;
using namespace std;

int H_MIN = 0;			
int H_MAX = 255;			
int S_MIN = 0;			
int S_MAX = 255;			
int V_MIN = 0;				
int V_MAX = 255;
int E_FAC = 1;
int D_FAC = 1;

void calibrationBars(int, void*) {};

Mat image_Calib(Mat &img) {
	Mat hsv, thresh;
	cvtColor(img, hsv, COLOR_BGR2HSV);
	inRange(hsv, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), thresh);
	GaussianBlur(thresh, thresh, Size(7, 7), 2, 2);
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(E_FAC, E_FAC));
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(D_FAC, D_FAC));
	erode(thresh, thresh, erodeElement);
	dilate(thresh, thresh, dilateElement);
	imshow("Calibrated", thresh);
	return thresh;
}

int calculateDist(Mat &color,Mat &thresh) {
	Mat img = color.clone();
	vector<int> points_x;
	vector<Vec4i> hierarchy;
	double greatest_Area, greatest_x,lowest_x;
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
		for(int i =0;i<points.size();i++)
		{
			points_x.push_back(points[i].x);; 
		} 
		greatest_x = *max_element(points_x.begin(), points_x.end());
		lowest_x = *min_element(points_x.begin(), points_x.end());
		//for (int i= 0;i<points.size();i++)
		//{
			//double tempg = 0;
			//double templ = 640;
			//if(points[i].x>tempg)
					//{
						//tempg = points[i].x;
						//greatest_x = tempg;
						
					//}
			//if(points[i].x<templ)
					//{
						//templ = points[i].x;
						//lowest_x = templ;
					//} 
		//}		
		//drawContours(img, hull, contour_index, Scalar(0, 255, 0), 3, 8, vector<Vec4i>(), 0, Point());
		drawContours(img, contours, contour_index, Scalar(0, 255, 255), 2, 8, vector<Vec4i>(), 1, Point());
		imshow("Img", img);
	}
	//ROS_INFO_STREAM("<<" <<greatest_x <<" , " <<lowest_x <<">>");
	return(greatest_x-lowest_x);
}

int main(int argc, char **argv){
	ros::init(argc, argv,"OCDAL");
	ros::NodeHandle nh;
	Mat color = imread("12.jpg");
	std::ostringstream disTxt;
	Mat calibrated; 
	int pixelWidth;
	double dist;
	namedWindow("Calibration", WINDOW_NORMAL);
	createTrackbar("H_MIN", "Calibration", &H_MIN, H_MAX, calibrationBars);
	createTrackbar("H_MAX", "Calibration", &H_MAX, H_MAX, calibrationBars);
	createTrackbar("S_MIN", "Calibration", &S_MIN, S_MAX, calibrationBars);
	createTrackbar("S_MAX", "Calibration", &S_MAX, S_MAX, calibrationBars);
	createTrackbar("V_MIN", "Calibration", &V_MIN, V_MAX, calibrationBars);
	createTrackbar("V_MAX", "Calibration", &V_MAX, V_MAX, calibrationBars);
	createTrackbar("E_FAC", "Calibration", &E_FAC, 25, calibrationBars);
	createTrackbar("D_FAC", "Calibration", &D_FAC, 25, calibrationBars);
	while(ros::ok())
	{
		calibrated = image_Calib(color);
		pixelWidth = calculateDist(color, calibrated);
		dist = 567*101.6/(10*pixelWidth);
		//ROS_INFO_STREAM(pixelWidth);
		ROS_INFO_STREAM(dist);
		//imshow("Original", color);
		waitKey(30);
		ros::spinOnce();
	}
	return 0;
}
