#pragma once
#include <opencv2/core/core.hpp>
#include <string>
#include "Chessboard.h"
using namespace cv;
using namespace std;

class Callibration
{
		
public:
	struct scKalib
	{
		Mat intrinsischeParameter;
		Mat verzerrungsParameter;
		vector<Mat> vRotationsmatrizen;
		vector<Mat> vTranslatoinsmatrizen; 
		int usedCam;
	};
	struct dcKalib
	{
		scKalib KameraA;
		scKalib KameraB;
		Mat Rotation;
		Mat Translation;
		Mat Essential;
		Mat Fundamental;
	};
	
	static void uiChangeSettings();

	struct scKalib scSolution;
	struct dcKalib dcSolution;

	void showMainUI();
	
	Callibration(string pathToSettingXML);
	Callibration();
	~Callibration(void);
private:
	void UIMakeChessboard();
	Chessboard::Perspective *scCam;
	Chessboard::Perspective *dcCamA;
	Chessboard::Perspective *dcCamB;

	Chessboard *usedChessboard;
	void startCallibration();
#pragma region singleCam
	void uiSingleCam();	//HauptUI
	void scPrintParameter(int i, struct Callibration::scKalib *scSolptr);
	void scAddPictureFromFile(string pathName);
	void scAddPicturesFromWebcam(int i);
	void scStartCalib();
	void scuiAddPictures();
	void scSave();
	void scSaveGoodPictures(string pathName);
	void scuiUseCaliForStereo();
	void scUseCaliForStereo(struct Callibration::scKalib *scSolptr);
#pragma endregion
#pragma region Twocams
	void uiTwoCam();	//HauptUI
	void dcPrintParameter(int i);
	void dcAddPictureFromFile();
	void dcAddPicturesFromWebcam();
	void dcStartCalib();
	void dcuiAddPictures();
	void dcSave();
	void dcSaveGoodPictures();
	void dcDeletePictures();
#pragma endregion
	void showUIText();
	void showGoodPictures(Chessboard::Perspective * persp);
};





