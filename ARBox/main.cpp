#include <windows.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <nuiapi.h>
#include <nuiimagecamera.h>
#include <nuisensor.h>
#include <kinectconnector.h>
#include <vector>
#include <queue>
#include <stdlib.h>

using namespace std;
using namespace cv;

int mousex, mousey;
bool mouseclickbgr = false;
bool mouseclickdepth = false;
KinectConnector kin;
cv::Mat  pointImg;
cv::Mat  planeImg;
cv::Mat  ceilImg;
cv::Mat  intrinsics, distortion;
cv::Mat  displayImg;
cv::Mat  cImage;
cv::Mat depthImg;
std::vector<cv::Point> depcoImg;
// floor		1st plane		
Vec3b color_planes[] = { Vec3b(250,0,0),Vec3b(0,250,0),Vec3b(0,0,250),Vec3b(250,250,0) ,Vec3b(0,250,250) };

void MyFilledCircle(Mat img, cv::Point center)
{
	int thickness = 8;
	int lineType = 8;

	circle(img,
		center,
		100 / 32.0,
		Scalar(0, 255, 255),
		thickness,
		lineType);
}

float distPointToPlane(float& D, Vec3f& N, Vec3f& P) {
	return abs(N.dot(P) + D) / sqrt(N.dot(N));
}
float distOfTwoPoints(Vec3f a, Vec3f b) {
	
	return sqrt((a[0] - b[0])*(a[0] - b[0]) + (a[1] - b[1])*(a[1] - b[1]) + (a[2] - b[2])*(a[2] - b[2]));
}

void findLineInTwoPlane(Vec3f n1,Vec3f n2,float d1,float d2,Vec3f& vLine,Vec3f& pLine) {
	float det = ((n1.dot(n1))*(n2.dot(n2))) - (n1.dot(n2)*n1.dot(n2));
	float c1 = (d1 * n2.dot(n2) - d2 * n1.dot(n2)) / det;
	float c2 = (d2 * n1.dot(n1) - d1 * n1.dot(n2)) / det;
	vLine = n1.cross(n2);
	pLine = c1*n1 + c2*n2;
}
void findPointFromThreePlane(Vec3f& n1, Vec3f& n2, Vec3f& n3,float& d1,float& d2 ,float& d3,Vec3f& out_p) {
	Vec3f P = (-d1) * (n2.cross(n3)) + (-d2) * (n3.cross(n1)) + (-d3) * (n1.cross(n2));
	float Q = n1.dot(n2.cross( n3));
	out_p = P / Q;
}

void findPlaneFromThreePoint(Vec3f& a, Vec3f& b, Vec3f& c, Vec3f& vN, float& D) {
	Vec3f vP = a - c;
	Vec3f vQ = b - c;
	vN = vP.cross(vQ);
	D = -vN.dot(c);
}
void drawCube(Vec2i Pdd[8], USHORT depthVal[8]) {
	cImage.copyTo(displayImg);

	vector<Point> depth_point(8);
	vector<Point> color_point(8);

	
	bool isValidColorCube = true;
	for (int i = 0; i < 8; i++) {
		color_point[i] = kin.mapperDtoC(Pdd[i],depthVal[i]);
		if (!color_point[i].x &&!color_point[i].y)
			isValidColorCube = false;
		depth_point[i].x = Pdd[i][0];
		depth_point[i].y = Pdd[i][1];
	}
	
	//draw in depth
	line(planeImg, depth_point[0], depth_point[1], Scalar(0, 255, 255), 5);
	line(planeImg, depth_point[0], depth_point[2], Scalar(0, 255, 255), 5);
	line(planeImg, depth_point[0], depth_point[3], Scalar(0, 255, 255), 5);
	line(planeImg, depth_point[1], depth_point[5], Scalar(0, 255, 255), 5);
	line(planeImg, depth_point[1], depth_point[6], Scalar(0, 255, 255), 5);
	line(planeImg, depth_point[5], depth_point[2], Scalar(0, 255, 255), 5);
	line(planeImg, depth_point[5], depth_point[7], Scalar(0, 255, 255), 5);
	line(planeImg, depth_point[6], depth_point[7], Scalar(0, 255, 255), 5);
	line(planeImg, depth_point[6], depth_point[3], Scalar(0, 255, 255), 5);
	line(planeImg, depth_point[7], depth_point[4], Scalar(0, 255, 255), 5);
	line(planeImg, depth_point[2], depth_point[4], Scalar(0, 255, 255), 5);
	line(planeImg, depth_point[3], depth_point[4], Scalar(0, 255, 255), 5);

	// draw in color 
	if (isValidColorCube) {
		line(displayImg, color_point[0], color_point[1], Scalar(0, 255, 255), 5);
		line(displayImg, color_point[0], color_point[2], Scalar(0, 255, 255), 5);
		line(displayImg, color_point[0], color_point[3], Scalar(0, 255, 255), 5);
		line(displayImg, color_point[1], color_point[5], Scalar(0, 255, 255), 5);
		line(displayImg, color_point[1], color_point[6], Scalar(0, 255, 255), 5);
		line(displayImg, color_point[5], color_point[2], Scalar(0, 255, 255), 5);
		line(displayImg, color_point[5], color_point[7], Scalar(0, 255, 255), 5);
		line(displayImg, color_point[6], color_point[7], Scalar(0, 255, 255), 5);
		line(displayImg, color_point[6], color_point[3], Scalar(0, 255, 255), 5);
		line(displayImg, color_point[7], color_point[4], Scalar(0, 255, 255), 5);
		line(displayImg, color_point[2], color_point[4], Scalar(0, 255, 255), 5);
		line(displayImg, color_point[3], color_point[4], Scalar(0, 255, 255), 5);
	}
	cv::imshow("cubeImg", displayImg);
}


void post_production(vector<Vec3f>& normals,vector<float>& const_d) {
	Vec3f vecLine1 = normalize(normals[0].cross(normals[3]));
	if (vecLine1[2] > 0)
		vecLine1 *= -1;
	
	Vec3f vecLine2 = normalize(normals[0].cross(normals[2]));
	if (vecLine2[2] > 0)
		vecLine2 *= -1;
	
	Vec3f P[8];
	findPointFromThreePlane(normals[0], normals[2],normals[3],const_d[0], const_d[2], const_d[3],P[0]);
	findPointFromThreePlane(normals[1], normals[2], normals[3], const_d[1], const_d[2], const_d[3], P[1]);
	
	// Vec of first two points
	Vec3f vecLine3 = normalize(P[0] - P[1]);
	float distP = distOfTwoPoints(P[0], P[1]);

	const int BOXSIZE = 100;
	const int TRANS_DIST = 200;
	P[0] += TRANS_DIST * vecLine1;
	P[1] = P[0] - distP * vecLine3;
	P[2] = P[0] - BOXSIZE * vecLine1;
	P[3] = P[0] - BOXSIZE * vecLine2;
	P[4] = P[0] - BOXSIZE * (vecLine1 + vecLine2);
	P[5] = P[1] - BOXSIZE * vecLine1;
	P[6] = P[1] - BOXSIZE * vecLine2;
	P[7] = P[1] - BOXSIZE * (vecLine1 + vecLine2);
	Vector4 *skeletonPoint = new Vector4();
	Vec2i Pdd[8];
	USHORT depthVal[8] = {0};
	for (int i = 0; i < 8; i++)
	{
		skeletonPoint = new Vector4();
		skeletonPoint->x = P[i][0];
		skeletonPoint->y = P[i][1];
		skeletonPoint->z = P[i][2];
		
		Pdd[i] = kin.mapper(skeletonPoint);
		//depthVal[i] = (USHORT)distOfTwoPoints(Vec3f(P[i][0] / 8 ,P[i][1] / (8), P[i][2] / (8) ), Vec3f(0, 0, 0));
		MyFilledCircle(planeImg, cv::Point(Pdd[i][0], Pdd[i][1]));
	}
	drawCube(Pdd,depthVal);
}
			
void reOrderPlanes(vector<Vec3f>& normals, vector<float>& const_d) {
	// make sure top (1)
	for (int i = 2; i < 4; i++) {
		Vec3f crProduct = (normalize(normals[0]).cross(normalize(normals[i])));
		float dotProduct = crProduct.dot(crProduct);
		//printf("%f\n", dotProduct);
		if (dotProduct < 0.1)
		{
			Vec3f tempv= normals[1];
			normals[1] = normals[i];
			normals[i] = tempv;
			float tempf = const_d[1];
			const_d[1] = const_d[i];
			const_d[i] = tempf;
		//	printf("ttt\n");
		}
	}
	// make sure sides 
	if (normals[2][2] > 0) {
		normals[2] *= -1;
		const_d[2] *= -1;
	}
	if (normals[3][2]>0) {
		normals[3] *= -1;
		const_d[3] *= -1;
	}

	
	if (normals[2][0]>0) {
		Vec3f tempv = normals[3];
		normals[3] = normals[2];
		normals[2] = tempv;
		float tempf = const_d[3];
		const_d[3] = const_d[2];
		const_d[2] = tempf;
		//printf("sss\n");

	}


}
void improved_solution() {
	int random = 100;
	const int row = pointImg.rows;
	const int col = pointImg.cols;
	const float thresholdDistance = 10.0f;
	const float floor_thresh = 30.0f;

	vector<Vec3f> normals(4);
	vector<float> const_d(4);
	vector<Vec3f> search_space;
	vector<Point2i> coordinates;
	vector<Vec3f> temp_space;
	vector<Point2i> temp_coor;
	RNG rng;

	// get all points to search_space
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			
			if (depthImg.at<USHORT>(i, j) > 0 && depthImg.at<USHORT>(i, j) < 4000) {
				search_space.push_back(pointImg.at<Vec3f>(i, j));
				coordinates.push_back(Point2i(i, j));
			}
			planeImg.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			
			
			
		}
	}

	// find floor
	int max_count = 0;
	Vec3f selected_normal;
	float selected_d;
	int vecsize = search_space.size();
	for (int k = 0; k < 30; k++) {
		
		Vec3f vN;
		float D;
		if(vecsize > 0)
		findPlaneFromThreePoint(search_space[rng.uniform(0, vecsize)], search_space[rng.uniform(0, vecsize)], search_space[rng.uniform(0, vecsize)], vN, D);
		
		int  countPoint = 0;
		for (int index = 0; index < vecsize; index++)
			if (distPointToPlane(D, vN, search_space[index]) < floor_thresh)
				countPoint++;
		if (max_count < countPoint) {
			max_count = countPoint;
			selected_normal = vN;
			selected_d = D;
		}
	}
	for (int index = 0; index < vecsize; index++) {
		if (distPointToPlane(selected_d, selected_normal, search_space[index]) < floor_thresh) {
			// temp_space.push_back(search_space[index]);
			// temp_coor.push_back(coordinates[index]);
			Point2i p = coordinates[index];
			planeImg.at<Vec3b>(p.x, p.y) = color_planes[0];
		}
		else {
			temp_space.push_back(search_space[index]);
			temp_coor.push_back(coordinates[index]);
		}
	}
	coordinates = temp_coor;
	search_space = temp_space;
	temp_coor.clear();
	temp_space.clear();
	normals[0] = selected_normal;
	const_d[0] = selected_d;

	// ransac for other 3 planes
	for (int plane_i = 1; plane_i <= 3; plane_i++) {
		// initilize var
		max_count = 0;
		vecsize = search_space.size();
		if (plane_i == 3) random = 200;
		for (int k = 0; k < random; k++) {
			
			Vec3f vN;
			float D;
			if(vecsize >0)
			findPlaneFromThreePoint(search_space[rng.uniform(0, vecsize)], search_space[rng.uniform(0, vecsize)], search_space[rng.uniform(0, vecsize)], vN, D);

			int  countPoint = 0;
			for (int index = 0; index < vecsize; index++)
				if (distPointToPlane(D, vN, search_space[index]) < thresholdDistance)
					countPoint++;
			if (max_count < countPoint) {
				max_count = countPoint;
				selected_normal = vN;
				selected_d = D;
			}
		}

		for (int index = 0; index < vecsize; index++) {
			if (distPointToPlane(selected_d, selected_normal, search_space[index]) < thresholdDistance) {
				// temp_space.push_back(search_space[index]);
				// temp_coor.push_back(coordinates[index]);
				Point2i p = coordinates[index];
				planeImg.at<Vec3b>(p.x, p.y) = color_planes[plane_i];
			}
			else {
				temp_space.push_back(search_space[index]);
				temp_coor.push_back(coordinates[index]);
			}
		}
		search_space = temp_space;
		coordinates = temp_coor;
		temp_space.clear();
		temp_coor.clear();
		normals[plane_i] = selected_normal;
		const_d[plane_i] = selected_d;
	}
	reOrderPlanes(normals, const_d);
	post_production(normals, const_d);


}

int main(int argc, char** argv) {
	Scalar red(255, 0, 0);
	Scalar green(0, 255, 0);
	Scalar blue(0, 0, 255);

	FileStorage fs("out_camera_data_kinnect.xml", FileStorage::READ);
	fs["Camera_Matrix"] >> intrinsics;
	fs["Distortion_Coefficients"] >> distortion;

	if (intrinsics.rows != 3 || intrinsics.cols != 3 || distortion.rows != 5 || distortion.cols != 1) {
		cout << "Run calibration (in ../calibrate/) first!" << endl;
		return 1;
	}

	// kinnect open
	kin = KinectConnector();
	if (!kin.Connect()) {
		cout << "Kinect not connected" << endl;
		return 1;
	}
	Mat  depthImgN, indexImg;

	kin.GrabData(depthImg, cImage, indexImg, pointImg, depcoImg);
	namedWindow("ImageDisplay", 1);
	namedWindow("ImageDepth", 1);

	//  cout << cImage.rows << " " << cImage.cols << '\n';
	//  waitKey();

	ceilImg.create(depthImg.rows, depthImg.cols, CV_8UC3);
	planeImg.create(depthImg.rows, depthImg.cols, CV_8UC3);

	int count = 0;
	while (true) {
		//cap >> image;
		kin.GrabData(depthImg, cImage, indexImg, pointImg, depcoImg);

		cv::normalize(depthImg, depthImgN, 0, 255, NORM_MINMAX, CV_8UC1);

		imshow("ImageDepth", depthImgN);
		//imshow("ImageDisplay", cImage);
		//imshow("pointImg", pointImg);

		improved_solution();

		 imshow("planeImg", planeImg);
		// Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));

		// morphologyEx(ceilImg, ceilImg, MORPH_CLOSE, kernel, Point(-1, -1), 10);


		// drawCube(ceilImg);
		//draw_intersected_points_to_plane(ceilImg);

		char chWait;
		if ((chWait = waitKey(20)) == 32) {
			break;
		}
		else if (chWait == 'p') {
			while (waitKey(20) != 'p');

		}
	}
	return 0;
}


