#pragma once
#include "Callibration.h"

using namespace cv;
using namespace std;

class TwoPictureManipulation
{
public:
	TwoPictureManipulation(void);
	~TwoPictureManipulation(void);
	static void drawEpilines(Mat img1, Mat img2, Mat fundemental, unsigned int NumberOfEpilinesPrinted);
	static vector < vector<cv::Vec3f> > getAllEpilineVector(Mat img2, Mat img1, Mat fundemental, int whichPicture=1);
	static vector<Mat> rectify(Mat img1,Mat img2,Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Size imageSize, Mat R, Mat T,Mat &OutputQ,Rect &OutputROI);
	static vector<Mat> DispMapFast(Mat img1,Mat img2,Rect roi);
	static vector<Mat> DispMapSlow(Mat img1,Mat img2,Rect roi);
	static Mat reproject2(Mat disparityMap, Mat Q, Mat img_rgb);
	static void Measurement(Mat img1, Mat xyz, bool average);
	static vector<Mat> DispMapGPU(Mat img1, Mat img2, Rect roi);
};

