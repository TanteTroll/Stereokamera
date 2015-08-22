#include "stdafx.h"
#include "Webcam.h"
#include <iostream>
#include <stdlib.h>
#include "opencv\cv.h"
#include "Chessboard.h"
#include "SinglePictureManipulation.h"
#include "Settings.h"
Webcam::~Webcam()
{
	for(unsigned int j=0; j< vCamList.size(); ++j)
	{//fuer jede Kamera
		vCamList[j]->release();
	}
	vCamList.clear();
	
}
Webcam::Webcam(int numberOfWebcams)
{
	cout<<"Oeffne Webcam"<<endl;
	if (numberOfWebcams < 1) throw 1;
	for ( int j = Settings::webcamFirstWebcam ; j < numberOfWebcams+Settings::webcamFirstWebcam ; j++ )
	{
		VideoCapture * activeCam = new VideoCapture(j);
		if(!activeCam->isOpened())
		{
			cout<<"Kamera konnte nicht geoeffnet werden"<<endl;
			throw 1;
		}
		activeCam->set(CV_CAP_PROP_FRAME_WIDTH,Settings::cameraresWidth);
		activeCam->set(CV_CAP_PROP_FRAME_HEIGHT,Settings::cameraresHight);
		vCamList.push_back(activeCam);
	}
}
void Webcam::deleteTakenPictures()
{
	takenPictures.clear();
}
void Webcam::takePictures(Size ChessboardSize=Size(0,0) )
{
	int key;	//Nutzereingabe überwachen
	cout << "Press Enter to save a picture. Press Escape to finish." << endl;
	while(1)
	{
		try{
		//Bilder aus Kamera holen
		vector < Mat > activePicture;
		for(unsigned int j=0; j< vCamList.size(); ++j)
		{   //fuer jede Kamera
			Mat frame;
			
			*vCamList[j]>>frame;

			if(Settings::webcamFlippedCamsContain(j) )
			{
				flip(frame, frame, 0);
			}

			Mat picture;
			resize(frame,picture,Size(800,600));
			char winName[20];
			sprintf(winName,"Kamera%d",j);
			printFrame(picture,winName,ChessboardSize);
			//Speichern aller Bilder disen durchganges
			activePicture.push_back(frame);
		}
		if (key=waitKey(100))
		{
			switch (key)
			{
			case 27:
				{
					cout<<"Aufnahme Beendet!"<<endl;
					return;
				}
			case 13:
				{
					if (takenPictures.size()>100)cout<<"Zuviele Bidler gespeichert";
					else
					{
						cout << "Pictures saved" << endl;
						takenPictures.push_back(activePicture);
					}
					break;
				}
			default:break;
			}//endswitch
		}//end keyPress
		}catch(...){;};
	}//endWhile
	destroyAllWindows();
}
void Webcam::streamSC(int cameraNumber, Mat* outputPicture)
{
	try{
		//Bilder aus Kamera holen
		vector < Mat > activePicture;

		Mat frame;
		*vCamList[cameraNumber]>>frame;

		if(Settings::webcamFlippedCamsContain(cameraNumber) )
		{
			flip(frame, frame, 0);
		}

		*outputPicture=frame;
		}catch(...){;};
}

void Webcam::printFrame(Mat frame, string name,Size ChessboardSize=Size(0,0))
{

	Mat picture;

	//Kein Schachbrett malen
	if ( ChessboardSize.height>3 && ChessboardSize.width>3 && Settings::webcamShowChessboard == 1 )
	{
		//Schachbrett soll gemalt werden
		vector<Point2f> vChessboardPoints;
		Chessboard cbTMP(ChessboardSize.height,ChessboardSize.width);
		bool patternfound = findChessboardCorners(frame, cbTMP.chessboardSize, vChessboardPoints,
	CALIB_CB_NORMALIZE_IMAGE+CV_CALIB_CB_ADAPTIVE_THRESH+CV_CALIB_CB_FILTER_QUADS+CALIB_CB_FAST_CHECK);
		//konnte keine Schachbrett finden
		if (!patternfound)
		{
			frame.copyTo(picture);
		}
		else
		{
			//zugehöriges graues Bild erstellen
			frame.copyTo(picture);
			drawChessboardCorners(picture,cbTMP.chessboardSize,vChessboardPoints,patternfound);
		}
		
	}
	else if (Settings::webcamShowFeatures)
	{
		Mat tmp;
		frame.copyTo(tmp);
		picture=SinglePictureManipulation::getFeaturesAsMat(tmp);
	}
	else
	{
		frame.copyTo(picture);
	}
	imshow( name, picture);

}

void Webcam::show()
{
	takePictures(Size(0,0) );
}


vector<Mat> Webcam::getTakenPicture(int CameraNumber=1)
{
	if (CameraNumber<1) throw 1;
	vector<Mat> returnVektor;
	for(unsigned int j=0; j< takenPictures.size(); ++j)
	{
		returnVektor.push_back(takenPictures[j][CameraNumber-1]);
	}
	return returnVektor;
}
vector < vector < Mat > > Webcam::getTakenPicture()
{
	return takenPictures;
}
void Webcam::showPictures(vector <Mat> images)
{
	char winName[20];
	for (unsigned int i = 0;i< images.size();i++)
	{
		sprintf(winName,"Window%d",i);
		imshow(winName,images[i]);
	}
	waitKey(0);
	for (unsigned int i = 0;i< images.size();i++)
	{
		sprintf(winName,"Window%d",i);
		destroyWindow(winName);
	}
}
void Webcam::helpVector()
{
	cout<<"v.erase(i); Entfernt das Element i aus v."<<endl;
	cout<<"v.insert(i, val); Fügt ein neues Element mit dem Wert val vor das Element i in v ein."<<endl;
	cout<<"v.clear(); Entfernt alle Elemente aus v; leert v."<<endl;
	cout<<"v.pop_back(); Entfernt das letzte Element aus v."<<endl;
}
void Webcam::showTwoPictures(Mat img1, Mat img2)
{
	Mat imgshow1, imgshow2;
	resize(img1,imgshow1,Size(800/8*5,600/8*5));
	resize(img2,imgshow2,Size(800/8*5,600/8*5));

	Mat picture(600/8*5, 800/4*5, CV_8UC3);
	Mat left(picture, Rect(0, 0, 800/8*5, 600/8*5));
	Mat right(picture, Rect(800/8*5, 0, 800/8*5, 600/8*5));

	imgshow1.copyTo(left);
	imgshow2.copyTo(right);

	imshow("ZweiBilder",picture);
	waitKey(0);
	return;
}
void Webcam::saveTakenPicturesInFile(int CameraNumber)
{
	
	if (CameraNumber<1) throw 1;
	string saveName;
	char number[20];
	vector<Mat> returnVektor;
	for(unsigned int j=0; j< takenPictures.size(); ++j)
	{
		returnVektor.push_back(takenPictures[j][CameraNumber-1]);
	}
	for (unsigned int i=0;i< returnVektor.size();++i)
	{
		
		sprintf(number,"%d",i);
		saveName=Settings::calibrateFilenamePictures+Settings::calibrateFilenamePicturesNotTested+number+".jpg";
		try
		{
			cout << "Saving: " << saveName <<endl;
			imwrite(saveName, returnVektor[i]);
		}
		catch(...)
		{cout<<"nicht genuegend Rechte um Bild zu speichern?"<<endl;}
	}

}

void Webcam::saveVideo()
{
	bool captureActive=false;
	int key;	//Nutzereingabe überwachen
	Size frameSize(Settings::cameraresWidth,Settings::cameraresHight);
	VideoWriter oVideoWriter ("MyVideo.avi", CV_FOURCC('M','J','P','G'), 20, frameSize, true);
	cout << "Enter druecken um Aufnahme zu beginnen."<<endl<< "!Geht nur bei einer Kamera richtig!" << endl;
	//Auch bei anderen Kameras --> in ne for schleife wie bei den Bildern
	while(1)
	{
		try{
		
		for(unsigned int j=0; j< vCamList.size(); ++j)
		{   //fuer jede Kamera
			Mat frame;
			//Bilder aus Kamera holen
			*vCamList[j]>>frame;

			if(Settings::webcamFlippedCamsContain(j) )
			{
				flip(frame, frame, 0);
			}

			Mat picture;
			resize(frame,picture,Size(800,600));
			char winName[20];
			sprintf(winName,"Kamera%d",j);
			imshow(winName,picture);
			//Speichern aller Bilder disen durchganges
			if (captureActive)
			{
				oVideoWriter.write(frame);
				
			}
	//Nutzereingabe
		}
		if (key=waitKey(200))
		{
			switch (key)
			{
			case 27:
				{
					cout<<"Aufnahme Beendet!"<<endl;
					return;
				}
			case 13:
				{
					captureActive=captureActive?false:true;
					cout<<"Es wird aufgenommen: "<<captureActive<<endl;
				}
			default:break;
			}//endswitch
		}//end keyPress
		}catch(...){;};
	}//endWhile
	destroyAllWindows();
}
vector<Mat> Webcam::readVideo(string videoname)
{
	VideoCapture* capture;
	try
	{
		capture = new VideoCapture(videoname);
	}
	catch(...)
	{
		cout<<"kein Video gefunden"<<endl;
	}

	
	vector<Mat> returnVector;
	for(int i=0;capture->isOpened();i++)
	{
		Mat frame;
		capture->read(frame);
		if (!frame.data )break;
		returnVector.push_back(frame);
	}
	capture->release();
	delete capture;
	return returnVector;
}
