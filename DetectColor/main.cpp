#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <KinectConnector.h>
#include <vector>
#include <queue>


using namespace std;
using namespace cv;

int mousex, mousey;

Mat depthImg;
Mat img;
Mat segment;
Mat indexImg;
Mat pointImg;

int color_space = 11;
// red pink purple lightblue bluegreen green lightgreen yellow yelloworange
char color_name[12][20] = { "Red", "Pink", "LightPink", "Purple","Blue", "LightBlue" ,"BlueGreen" ,"Green" ,"LightGreen","Yellow" , "YellowOrange" ,"Orange" };
//int lowerb[] = { 230,210,210,187,152,134,124,95,62,34,22 ,3};
// int upperb[] = { 250,230,230,204,160,150,133,118,75,43,31,20};
 int lowerb[] = { 230,210,210,170,152,134,124,95,62,34,22 ,3 };
 int upperb[] = { 250,230,230,190,160,150,133,118,75,43,31,20 };

 Scalar color_scalar[12]; //{ SCL( 1.0, 0.0 ,0.0) , SCL(1.0,0.0 ,0.5) ,SCL(0.5,0.0 ,0.5),SCL(0.5,0.0 ,0.5) };

//int lowerb_s = { 230}

void CallBackFunc(int event, int x, int y, int flags, void* param) {
	Mat* rgb = (Mat*)param;
	Mat asd;
	cvtColor((*rgb), asd, CV_BGR2HSV);
	if (event == CV_EVENT_MOUSEMOVE) {
		mousex = x;
		mousey = y;
	}
}

#define WHITE_THRESH 40
bool is_white(Vec3b v) {
	int r, g, b;
	r = v[2];
	g = v[1];
	b = v[0];
	return abs(r - g) <= WHITE_THRESH && abs(r - b) <= WHITE_THRESH && abs(g - b) <= WHITE_THRESH;
}


void get_color() {
	Mat hsv;
	cvtColor(img, hsv, CV_BGR2HSV_FULL);
	blur(hsv, hsv, Size(7, 7));
	Vec3b pt = hsv.at<Vec3b>(mousey, mousex);
	Vec3b rgb = img.at<Vec3b>(mousey, mousex);
	printf("i = %d , j = %d | RGB %d %d %d HSV: %d %d %d\n",mousey,mousex, rgb[2], rgb[1], rgb[0], pt[0], pt[1], pt[2]);
}
typedef vector<vector<vector<bool> > > VBool3D;
typedef vector<vector<bool> >  VBool2D;
typedef vector<bool>  VBool1D;
#define NB 8
void bfs(int k , VBool2D &cc ,VBool2D &ck_final, VBool2D &ck_final_ed) {
	// 8
	short ti[] = { 1,1,1,0,-1,-1,-1,0 };
	short tj[] = { -1,0,1,-1,-1,0,1,1 };
	//4
	//short ti[] = { 0,1,0,-1 };
	//short tj[] = { 1,0,-1,0 };


	VBool2D ck(cc.size(), VBool1D(cc[0].size(),false));
	

	
	for (int i = 0; i < cc.size(); i++)
		for (int j = 0; j < cc[i].size();j++)
			if (cc[i][j]&&!ck[i][j]) {
				int count_point = 1;
				vector<pair<int, int> > ck2vec;
				vector<pair<int, int> > ck_ed;
				queue<pair<int, int> > q;
				q.push(make_pair(i, j));
				ck[i][j] = true;
				long long hi=i, hj = j;
				int minj = 10000000;
				while (q.size() > 0)
				{
					int xi = q.front().first;
					int xj = q.front().second;
					q.pop();
					int count = 0;
					for (int i = 0; i < NB; i++)
						if (xi + ti[i] >= 0 && xj + tj[i] >= 0 && xi + ti[i] < cc.size() && xj + tj[i] < cc[i].size())
							if (cc[xi + ti[i]][xj + tj[i]])
								count++;
					if (count >=4)
					{
						for (int i = 0; i < NB; i++)
							if (xi + ti[i] >= 0 && xj + tj[i] >= 0 && xi + ti[i] < cc.size() && xj + tj[i] < cc[i].size() && !ck[xi + ti[i]][xj + tj[i]]) {
								q.push(make_pair(xi + ti[i], xj + tj[i]));
								ck2vec.push_back(make_pair(xi + ti[i], xj + tj[i]));
								ck[xi + ti[i]][xj + tj[i]] = true;
								hi += xi + ti[i];
								hj += xj + tj[i];
								if (minj > xj + tj[i])  minj = xj + tj[i];
								count_point++;
							}
					}
					else {

						ck_ed.push_back(make_pair(xi, xj));
					}

				}
				if (count_point > 3000) {
					//if(k==1 || k==4)
				//	printf("! %s ! -------------> %d \n", color_name[k], count_point);
					int n = ck2vec.size();
					for (int i = 0; i < n; i++)
						ck_final[ck2vec[i].first][ck2vec[i].second] = true;
					
					n = ck_ed.size();
					for (int i = 0; i < n; i++) {
						ck_final_ed[ck_ed[i].first][ck_ed[i].second] = true;
						circle(img, Point(ck_ed[i].second, ck_ed[i].first), 1, Scalar(255, 0, 0), 1, 8, 0);
					//	printf("x");
					}

					String str = color_name[k];
					Point point(minj, hi/count_point);
					putText(img, str, point, FONT_HERSHEY_DUPLEX,  1, color_scalar[k], 3, 8, false);
				}

			}


}
void color_segmentation(int mode) {
	Mat hsv;
	VBool3D each_color(color_space, VBool2D( img.rows, VBool1D(img.cols,false)));
	cvtColor(img, hsv, CV_BGR2HSV_FULL);
	blur(hsv, hsv, Size(10, 10));
	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			Vec3b org = hsv.at<Vec3b>(i, j);
			
			int hue = org[0];
			int sat = org[1];
			int val = org[2];

			bool check = false;
			for (int k = 0; k < color_space; k++) {
				if (lowerb[k] <= hue && hue <= upperb[k]) {
					check = true;
					if (sat <= 80) check = false;
					if (!is_white(img.at<Vec3b>(i, j))) check = true;
					if (val < 90 && ( k != 7 && k!=3) ) check = false;
					if (check) each_color[k][i][j] = true;
					if (check && sat > 210 && k == 2) each_color[k][i][j] = false;
					if (check && sat < 180 && k == 1) each_color[k][i][j] = false;
					
					
				
					
				}
			}
			if (!check) {
				segment.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
		}
	}
	VBool2D  last_left(img.rows,VBool1D(img.cols,false));
	VBool2D  last_left_ed(img.rows, VBool1D(img.cols, false));
	for (int i = 0; i < color_space; i++)
		bfs(i, each_color[i], last_left, last_left_ed);
	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			if (!last_left[i][j]) {
				segment.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
			if (last_left_ed[i][j]) { 
				segment.at<Vec3b>(i, j) = Vec3b(255, 255, 255);  	
				circle(segment, Point(j, i), 1, Scalar(255, 0,0 ), 1, 8, 0);
			}
		}

	}

}

int main() {
	//KinectConnector kin = KinectConnector();
	//if (!kin.Connect()) return 1;
	
	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
		return -1;

	cap.set(CAP_PROP_FRAME_WIDTH,960);
	cap.set(CAP_PROP_FRAME_HEIGHT, 960);

	namedWindow("ImageDisplay", 1);
	//kin.GrabData(depthImg, img, indexImg, pointImg);
	
	cap >> img;

	setMouseCallback("ImageDisplay", CallBackFunc, &img);
	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	while (true) {
		//kin.GrabData(depthImg, img, indexImg, pointImg);
		cap >> img;
		segment = img.clone();
		color_segmentation(0);
		morphologyEx(segment, segment, MORPH_CLOSE, kernel, Point(-1, -1), 10);
	//	color_segmentation(1);
		imshow("ImageDisplay", img);
		imshow("segmented", segment);
		get_color();
		if (waitKey(40) == 32) return 0;
	}
}
/*
#include "opencv2/opencv.hpp"
using namespace cv;

int main(int, char**)
{
	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
		return -1;

	//Mat edges;
	//namedWindow("edges", 1);
	for (;;)
	{
		Mat frame;
		cap >> frame; // get a new frame from camera
		cvtColor(frame, edges, CV_BGR2GRAY);
		GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
		Canny(edges, edges, 0, 30, 3);
		imshow("edges", edges);
	imshow("frame", frame);
	if (waitKey(30) >= 0) break;
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}*/