#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include <NuiApi.h>
//////#include <NuiImageCamera.h>
//#include <NuiSensor.h>
////#include <KinectConnector.h>
#include <vector>
#include <queue>
#include <fstream>
#include <cstring>
#define AREA_H 400 
#define AREA_W 800
#define NB 8
#define PI 3.14159265359
#define PAR_COUNT 1000
#define OBJ_X 100
#define OBJ_R_Y 170
#define OBJ_L_Y 230 
#define POINT_OBJ_L Point2i(OBJ_X, OBJ_L_Y)
#define POINT_OBJ_R Point2i(OBJ_X, OBJ_R_Y)

using namespace std;
using namespace cv;

int mousex, mousey;

Mat img;
Mat segment0;
Mat segment1;
Mat temp_img;

char color_name[12] = "LightBlue";
int lowerb = 135;
int upperb = 145;
const double focal_len = 594.72;
const double obj_real = 52 * 17;//cm
double obj_area_pix = 1; // pixel  (1 is the start value , not real value)
double sd = 3;

int lowerb_l = 0;
int upperb_l = 0;
int lowerb_r = 0;
int upperb_r = 0;


Scalar color_scalar;
void select_new_color(bool isL) {
	int theshold = 10;
	Mat hsv;
	cvtColor(img, hsv, CV_BGR2HSV_FULL);
	blur(hsv, hsv, Size(7, 7));
	Vec3b pt = hsv.at<Vec3b>(mousey, mousex);
	if (isL) {
		lowerb_l = pt[0] - theshold;
		upperb_l = pt[0] + theshold;
	}
	else {
		lowerb_r = pt[0] - theshold;
		upperb_r = pt[0] + theshold;
	}

}
void set_color(int i) {
	if(i ==0)
	{	
		strcpy_s( color_name, "LightBlue" );
		lowerb = lowerb_l;
		upperb = upperb_l;
	}
	else if( i ==1 )
	{
		strcpy_s( color_name, "Green");
		lowerb = lowerb_r;
		upperb = upperb_r;
	}
}
void MyFilledCircle(Mat& img, cv::Point center,Scalar color = Scalar(0,255,255) ,int thickness = 8)
{
	
	int lineType = 8;

	circle(img,
		center,
		10 / 32.0,
		color,
		thickness,
		lineType);
}

void CallBackFunc(int event, int x, int y, int flags, void* param) {
	//Mat* rgb = (Mat*)param;
	//Mat asd;
	//cvtColor((*rgb), asd, CV_BGR2HSV);
	if (event == CV_EVENT_MOUSEMOVE) {
		mousex = x;
		mousey = y;
	}
	if (event == EVENT_LBUTTONDOWN)
		select_new_color(true);
	if (event == EVENT_RBUTTONDOWN)
		select_new_color(false);
}

float distOfTwoPoints(Point2i a, Point2i b) {
	
	return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}
bool is_Wanted_corlor(Vec3b v) {
	int h, s;
	h = v[0];
	s = v[1];
	return h > lowerb && h < upperb;
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
	printf("\n i = %d , j = %d | RGB %d %d %d HSV: %d %d %d", mousey, mousex, rgb[2], rgb[1], rgb[0], pt[0], pt[1], pt[2]);
}


typedef vector<vector<vector<bool> > > VBool3D;
typedef vector<vector<bool> >  VBool2D;
typedef vector<bool>  VBool1D;

void bfs(int k, VBool2D &cc, VBool2D &ck_final, VBool2D &ck_final_ed) {
	// 8
	short ti[] = { 1,1,1,0,-1,-1,-1,0 };
	short tj[] = { -1,0,1,-1,-1,0,1,1 };
	//4
	//short ti[] = { 0,1,0,-1 };
	//short tj[] = { 1,0,-1,0 };

	VBool2D ck(cc.size(), VBool1D(cc[0].size(), false));

	int max_count_point = 1;
	for (int i = 0; i < cc.size(); i++)
		for (int j = 0; j < cc[i].size(); j++)
			if (cc[i][j] && !ck[i][j]) {
				int count_point = 1;
				vector<pair<int, int> > ck2vec;
				vector<pair<int, int> > ck_ed;
				queue<pair<int, int> > q;
				q.push(make_pair(i, j));
				ck[i][j] = true;
				long long hi = i, hj = j;
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
					if (count >= 4)
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
				if (count_point > 500) {
					if (count_point > max_count_point)
						max_count_point = count_point;
					int n = ck2vec.size();
					for (int i = 0; i < n; i++)
						ck_final[ck2vec[i].first][ck2vec[i].second] = true;


					n = ck_ed.size();
					for (int i = 0; i < n; i++) {
						ck_final_ed[ck_ed[i].first][ck_ed[i].second] = true;
						circle(img, Point(ck_ed[i].second, ck_ed[i].first), 1, Scalar(255, 0, 0), 1, 8, 0);
					}

					String str = color_name;
					Point point(minj, hi / count_point);
					putText(img, str, point, FONT_HERSHEY_DUPLEX, 1, color_scalar, 3, 8, false);
				}

			}
	obj_area_pix = max_count_point;

}
void color_segmentation(int mode,Mat &segment,Mat &img) {
	set_color(mode);
	Mat hsv;
	VBool2D bool_color(img.rows, VBool1D(img.cols, false));
	cvtColor(img, hsv, CV_BGR2HSV_FULL);
	blur(hsv, hsv, Size(10, 10));
	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			Vec3b org = hsv.at<Vec3b>(i, j);
			int hue = org[0];
			int sat = org[1];
			int val = org[2];

			bool check = false;
			if (lowerb <= hue && hue <= upperb) {
				check = true;
				if (sat <= 80) check = false;
				if (!is_white(img.at<Vec3b>(i, j))) check = true;
				if (check) bool_color[i][j] = true;
			}
			if (!check) {
				segment.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
		}
	}
	VBool2D  last_left(img.rows, VBool1D(img.cols, false));
	VBool2D  last_left_ed(img.rows, VBool1D(img.cols, false));

	bfs(0, bool_color, last_left, last_left_ed);
	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			if (!last_left[i][j]) {
				segment.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
			if (last_left_ed[i][j]) {
				segment.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
				circle(segment, Point(j, i), 1, Scalar(255, 0, 0), 1, 8, 0);
			}
		}

	}

}
void make_image_of_state(vector<pair<Point2i, double> > &particle , Mat& img){
	for (int i = 0; i < particle.size(); i++)
		if(particle[i].second != 0.0)
			MyFilledCircle(img, particle[i].first, Scalar(0, 0, 255));
}
void rand_particle(int n, vector<pair<Point2i, double> > &particle) {
	for (int i = 0; i < n; i++)
		particle.push_back(make_pair(Point2i(rand()%AREA_W, rand()%AREA_H), 1.0 / n));
}
Mat cal_position(double depthL ,double depthR) {
	Mat res(AREA_H, AREA_W, CV_8UC3);
	vector<pair<Point2i, double> > particle;
	rand_particle(PAR_COUNT, particle);
	

	for (int i = 0; i < PAR_COUNT; i++) {
		if (abs(distOfTwoPoints(particle[i].first, POINT_OBJ_L) - depthL) > 20)
			particle[i].second = 0.0;
		if (abs(distOfTwoPoints(particle[i].first, POINT_OBJ_R) - depthR) > 20)
			particle[i].second = 0.0;
	}
	make_image_of_state(particle, res);
	MyFilledCircle(res, POINT_OBJ_L,  Scalar(255, 0, 0),20);
	MyFilledCircle(res, POINT_OBJ_R,  Scalar(0, 255, 0),20);
	imshow("state",res);
	return res;
}
double cal_depth() {
	double ans = obj_real * focal_len / sqrtf(obj_area_pix/PI);
	return ans;
}
int main() {
	ofstream fout0("output0.csv");
	ofstream fout1("output1.csv");
	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
		return -1;

	namedWindow("ImageDisplay", 1);

	cap >> img;
	double distance = 0;
	int s = 0;
	setMouseCallback("ImageDisplay", CallBackFunc, &img);
	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	while (true) {
		cap >> img;
		temp_img = img.clone();
		get_color();

		segment0 = temp_img.clone();
		color_segmentation(0, segment0, temp_img);
		double depthL = cal_depth();

		segment1 = temp_img.clone();
		color_segmentation(1, segment1, temp_img);
		double depthR = cal_depth();
		
		imshow("ImageDisplay", img);
		//imshow("segmented0", segment0);
		//imshow("segmented1", segment1);

		printf(" Dept: %f  %f", depthL,depthR);
		
		cal_position(depthL,depthR);
		
		int cmd = waitKey(40);
		if (cmd == 32) {
			fout0 << distance << ", ";
			fout1 << distance << ", ";
			for (int i = 1; i <= 100; i++) {
				cap >> img;
				temp_img = img.clone();
				if (i % 10 == 0) {	
					printf("\nCapture %d", i);
					segment0 = temp_img.clone();
					segment1 = temp_img.clone();

					color_segmentation(0, segment0, temp_img);
					fout0 << obj_area_pix << ",";
					imshow("segmented0", segment0);

					color_segmentation(1, segment1, temp_img);
					fout1 << obj_area_pix << ", ";
					imshow("segmented1", segment1);

					imshow("ImageDisplay", img);
				}
				waitKey(40);
				//fout <<endl;
			}
			fout0 << endl;
			fout1 << endl;
			s++;
		}
		else if (cmd == 's') {
			cout << endl;
			cout << "Set distance" << endl;
			cin >> distance;
			cout << endl;
		}
		else if (cmd == 'q') {
			return 0;
		}

	}

}