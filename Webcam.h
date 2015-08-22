#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

class Webcam
{
private:

	vector < VideoCapture* > vCamList;
	vector < vector < Mat > > takenPictures;

	void printFrame(Mat frame, string name,Size ChessboardSize);
	
	
public:
	
	void takePictures(Size ChessboardSize);
	Webcam(int i);
	~Webcam(void);
	void show();
	vector < vector < Mat > > getTakenPicture();
	vector<Mat> getTakenPicture(int CameraNumber); 
	static void showPictures(vector<Mat> images);
	static void showTwoPictures(Mat img1,Mat img2);
	vector<Mat> takenPicture;
	void deleteTakenPictures();
	void helpVector();
	void saveTakenPicturesInFile(int CameraNumber);
	void streamSC(int cameraNumber, Mat* outputPicture); 
	void saveVideo();
	static vector<Mat> readVideo(string videoname);

};

