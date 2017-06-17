#include <ros/ros.h>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char **argv){
	ros::init(argc, argv,"OCDAL");
	ros::NodeHandle nh;
	VideoCapture cap(1); //Webcam 0, USB Cam 1
	Mat color,gray,edges,imgGrad,thresh;
	int distance;
	Mat bgr[3];
	if (!cap.isOpened()){
		ROS_ERROR("camera not open");
		return -1;
	}
	while(ros::ok())
	{
		if (!cap.isOpened()){
				break;
			}
		cap>>color;
		//cvtColor(color, gray, CV_BGR2GRAY);
		//Mat dist = Mat::zeros(color.size(),gray.type());
		//for(int x =0;x<color.cols;x++){
			//for(int y = 0;y<color.rows;y++){
					//Vec3f intensity = color.at<Vec3b>(y, x);
					//uchar b = intensity.val[0];
					//uchar g = 255-intensity.val[1];
					//uchar r = intensity.val[2];
					//dist.at<uchar>(y,x) = sqrt(b*b+r*r+g*g);
				//}			
			//}
		//imshow("Dist Img", dist);
		split(color,bgr);
		GaussianBlur(bgr[2],bgr[2], Size(7, 7),0);
		vector<vector<Point> > contours;
		vector<vector<Point> > approx;
		vector<Vec4i> hierarchy;
		findContours(bgr[2], contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
		for(int i = 0;i<contours.size();i++)
			approxPolyDP(contours[i],approx[i],100,true);
		for(int i = 0;i<approx.size();i++){
			drawContours(bgr[2], contours, i, Scalar(0, 255, 255), 2, 8, vector<Vec4i>(), 1, Point());
		}
		//Sobel(bgr[2],imgGrad, CV_8U, 1, 1,3,1, 0,BORDER_DEFAULT);
		//threshold(bgr[2], thresh, 75, 255, ADAPTIVE_THRESH_MEAN_C | THRESH_OTSU);
		//imshow("Threshold Img", thresh);
		//imshow("Sobel Img", imgGrad);
		imshow("Contours",bgr[2]);
		//imshow("Original", color);
		waitKey(10);
		ros::spinOnce();
			
	}
}
