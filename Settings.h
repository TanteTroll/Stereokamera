#pragma once
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <string>


using namespace cv;
using namespace std;
class Settings
{
public:
	static bool globalDebugshowPictures;
	static bool webcamShowChessboard;
	static bool webcamShowFeatures;
	static int webcamFirstWebcam;
	static string calibrateFilenameXML;
	static string calibrateFilenamePictures;
	static string calibrateFilenamePicturesStereo;
	static string calibrateFilenamePicturesCamA;
	static string calibrateFilenamePicturesCamB;
	static string calibrateFilenamePicturesNotTested;
	static float skalierfaktor;

	static bool webcamFlippedCams[10];
	static bool webcamFlippedCamsContain(int number);
	
	static bool Settings::saveNotTestedPictures;
	static int cameraresWidth;
	static int cameraresHight;
	static int featureFindMinHessian;

	static void Settings::uiChangeSettings();
	Settings();
	~Settings(void);
};
