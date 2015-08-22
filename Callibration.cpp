#include "stdafx.h"
#include "Callibration.h"
#include "Webcam.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv\cv.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "Settings.h"
#include "Chessboard.h"
#include <string>
using namespace cv;
using namespace std;


Callibration::~Callibration()
{
	
	//destroy 
	//usedPerspective
	//usedChessboard
	//Chessboard::Perspective *scCam;
	//Chessboard::Perspective *dcCamA;
	//Chessboard::Perspective *dcCamB;
}//TODO
Callibration::Callibration()
{ 
	UIMakeChessboard();
	scCam=new Chessboard::Perspective(usedChessboard);
	dcCamA=new Chessboard::Perspective(usedChessboard);
	dcCamB=new Chessboard::Perspective(usedChessboard);
	showMainUI();
}
Callibration::Callibration(string pathToSettingXML)
{ 
	//TODO
	std::cout<<"Versuche Datei "<<pathToSettingXML<<" zu oeffnen."<<endl;

	FileStorage fs(pathToSettingXML, FileStorage::READ);
	if( !fs.isOpened())
	{
		std::cout<<"Datei "<<pathToSettingXML<<" konnte nicht geoeffnet werden."<<endl;
		throw;
	}
	FileNode n = fs["SingleCamData"];
	n["scIntrinsischeParameter"] >> scSolution.intrinsischeParameter;
	n["scVerzerrungsParameter"] >> scSolution.verzerrungsParameter;
	n["vscRotationsmatrizen"] >> scSolution.vRotationsmatrizen;
	n["vscTranslationsmatrizen"] >> scSolution.vTranslatoinsmatrizen;

	n = fs["DoubleCamExtrinsic"];

	n["dcRotation"] >> dcSolution.Rotation;
	n["dcTranslation"] >> dcSolution.Translation;
	n["dcEssentiell"] >> dcSolution.Essential;
	n["dcFundamental"] >> dcSolution.Fundamental;
	FileNode m = n["DoubleCamAData"];

	m["dcCAMAIntrinsischeParameter"] >> dcSolution.KameraA.intrinsischeParameter;
	m["dcCAMAVerzerrungsParameter"] >> dcSolution.KameraA.verzerrungsParameter;
	m["vdcCAMARotationsmatrizen"] >> dcSolution.KameraA.vRotationsmatrizen;
	m["vdcCAMATranslationsmatrizen"] >> dcSolution.KameraA.vTranslatoinsmatrizen;

	m = n["DoubleCamBData"];

	m["dcCAMBIntrinsischeParameter"] >> dcSolution.KameraB.intrinsischeParameter;
	m["dcCAMBVerzerrungsParameter"] >> dcSolution.KameraB.verzerrungsParameter;
	m["vdcCAMBRotationsmatrizen"] >> dcSolution.KameraB.vRotationsmatrizen;
	m["vdcCAMBTranslationsmatrizen"] >> dcSolution.KameraB.vTranslatoinsmatrizen;

	fs.release();
}
void Callibration::UIMakeChessboard()
{
	unsigned int hight, width;
	float squareSize;
	std::cout<<"Breite des Schachbrettes eingeben"<<endl;
	cin>>width;
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(),'\n');
	std::cout<<"Hoehe des Schachbrettes eingeben"<<endl;
	cin>>hight;
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(),'\n');
	std::cout<<"Breite eines Kaestchens eingeben"<<endl;
	cin>>squareSize;
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(),'\n');
	usedChessboard=new Chessboard(hight,width,squareSize);

}
void Callibration::showMainUI()
{
	
	
	int input=0;
	while (input>=0)
	{
		showUIText();
		cin>>input;
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(),'\n');
		
		switch (input)
		{
		case 1: uiSingleCam();break;
		case 2: uiTwoCam();break;

		case 10: uiChangeSettings();break;

		default:break;
		}
		std::cout<<endl;
	}
}
void Callibration::showUIText()
{
	cv::destroyAllWindows();
	std::cout<<"-------------------------------------------------------"<<endl;
	std::cout<<"[1] Kalibrierung einzelner Kamera verwalten"<<endl;
	std::cout<<"[2] Stereokalibrierung verwalten"<<endl;
	std::cout<<"-------------------------------------------------------"<<endl;
	std::cout<<"[10] Einstellungen aendern"<<endl;
	std::cout<<"-------------------------------------------------------"<<endl;
	std::cout<<"[-1] Programm Abbrechen"<<endl;
	std::cout<<"-------------------------------------------------------"<<endl;
}

void Callibration::uiChangeSettings()
{
	Settings::uiChangeSettings();
}


void Callibration::showGoodPictures(Chessboard::Perspective * persp)
{
	Webcam::showPictures(persp->goodPictures);
}

#pragma region StereoCams
void Callibration::dcPrintParameter(int i)
{
	if(i==1)
	{
		cout<<"-- 1. Kamera --------------------------"<<endl;
		std::cout<<"IntrinsischeParameter"<<endl;
		cout<<dcSolution.KameraA.intrinsischeParameter<<endl;
		std::cout<<"VerzerrungsParameter"<<endl;
		cout<<dcSolution.KameraA.verzerrungsParameter<<endl;
		cout<<"-- 2. Kamera --------------------------"<<endl;
		std::cout<<"IntrinsischeParameter"<<endl;
		cout<<dcSolution.KameraB.intrinsischeParameter<<endl;
		std::cout<<"VerzerrungsParameter"<<endl;
		cout<<dcSolution.KameraB.verzerrungsParameter<<endl;
	}
	else if(i==2)
	{
		cout<<"-- 1. Kamera --------------------------"<<endl;
		std::cout<<"Anzahl Rotationsmatrizen"<<endl;
		cout<<dcSolution.KameraA.vRotationsmatrizen.size()<<endl;
		std::cout<<"Extrinische Translationsmatrizen"<<endl;
		cout<<dcSolution.KameraA.vTranslatoinsmatrizen.size()<<endl;
		cout<<"-- 2. Kamera --------------------------"<<endl;
		std::cout<<"Anzahl Rotationsmatrizen"<<endl;
		cout<<dcSolution.KameraB.vRotationsmatrizen.size()<<endl;
		std::cout<<"Extrinische Translationsmatrizen"<<endl;
		cout<<dcSolution.KameraB.vTranslatoinsmatrizen.size()<<endl;
		cout<<"-- 1. Kamera -> 2. Kamera -------------"<<endl;
		std::cout<<"Rotationsmatrizen"<<endl;
		cout<<dcSolution.Rotation<<endl;
		std::cout<<"Translationsmatrizen"<<endl;
		cout<<dcSolution.Translation<<endl;
	}
	else if(i==3)
	{
		std::cout<<"Objekt Punkte: "<<endl;
		
		std::cout<<endl<<"Insgesamt "<< dcCamA->vvObjectPoints.size() <<" Objekt Punkte in Kamera 1"<<endl; 
		std::cout<<endl<<"Insgesamt "<< dcCamB->vvObjectPoints.size() <<" Objekt Punkte in Kamera 2"<<endl;
		cout<<endl;
		std::cout<<endl<<"Bild Punkte"<<endl;
		std::cout<<endl<<"Insgesamt "<< dcCamA->vvImagePoints.size() <<" Bild Punkte in Kamera 1"<<endl; 
		std::cout<<endl<<"Insgesamt "<< dcCamB->vvImagePoints.size() <<" Bild Punkte in Kamera 2"<<endl;
		cout<<endl;
		std::cout<<endl<<"Bild Groesse"<<endl;
		std::cout<<endl<<"Bildgroesse von Kamera 1 "<< dcCamA->imageSize <<endl; 
		std::cout<<endl<<"Bildgroesse von Kamera 2 "<< dcCamB->imageSize <<endl;
	}
}
void Callibration::dcSave()
{
	string filename = Settings::calibrateFilenameXML;
	std::cout<<"Speichere nach "<<filename<<endl;
	FileStorage fs(filename, FileStorage::WRITE);
	
	fs << "DoubleCamCallibrationData"<< "{";
	fs << "chessboardSize" << usedChessboard->chessboardSize;
	fs << "SquareSize" << usedChessboard->SquareSize;
	fs << "}";
	fs << "DoubleCamExtrinsic"<< "{";
	fs << "dcRotation" << dcSolution.Rotation;
	fs << "dcTranslation" << dcSolution.Translation;
	fs << "dcEssentiell" << dcSolution.Essential;
	fs << "dcFundamental" << dcSolution.Fundamental;
	fs << "DoubleCamAData"<< "{";
	fs << "dcCAMAIntrinsischeParameter" << dcSolution.KameraA.intrinsischeParameter;
	fs << "dcCAMAVerzerrungsParameter" << dcSolution.KameraA.verzerrungsParameter;
	fs << "vdcCAMARotationsmatrizen" << dcSolution.KameraA.vRotationsmatrizen;
	fs << "vdcCAMATranslationsmatrizen" << dcSolution.KameraA.vTranslatoinsmatrizen;
	fs << "}";
	fs << "DoubleCamBData"<< "{";
	fs << "dcCAMBIntrinsischeParameter" << dcSolution.KameraB.intrinsischeParameter;
	fs << "dcCAMBVerzerrungsParameter" << dcSolution.KameraB.verzerrungsParameter;
	fs << "vdcCAMBRotationsmatrizen" << dcSolution.KameraB.vRotationsmatrizen;
	fs << "vdcCAMBTranslationsmatrizen" << dcSolution.KameraB.vTranslatoinsmatrizen;
	fs << "}";
	fs << "}";
}

void Callibration::uiTwoCam()
{while(1){
	std::cout<<"Anzahl gueltiger Bilder: "<<dcCamA->vvImagePoints.size()<<endl;
	std::cout<<"-------------------------------------------------------"<<endl;
	std::cout<<"[1] Kalibrierung starten"<<endl;
	std::cout<<"[2] Bilder hinzufuegen"<<endl;
	std::cout<<"[3] Parameter speichern"<<endl;
	std::cout<<"[4] Bilder in Datei speichern(ueberspeichert alte Bilder)"<<endl;
	std::cout<<"[5] Letzts Bildpaar loeschen"<<endl;
	std::cout<<"-------------------------------------------------------"<<endl;
	std::cout<<"[20] Ausgabe der intrinischen Parameter"<<endl;
	std::cout<<"[21] Ausgabe der extrinischen Parameter"<<endl;
	std::cout<<"[22] Ausgabe der Kalibrierungswerte"<<endl;
	std::cout<<"-------------------------------------------------------"<<endl;
	cout<<"[sonst] Abbrechen"<<endl;

	int input=0;
	cin>>input;
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(),'\n');

	switch (input)
	{
		case 1: dcStartCalib();break;
		case 2: dcuiAddPictures();break;
		case 3: dcSave();break;
		case 4: dcSaveGoodPictures();break;
		case 5: dcDeletePictures();break;
		case 20:dcPrintParameter(1);break;
		case 21:dcPrintParameter(2);break;
		case 22:dcPrintParameter(3);break;
		default:return;
	}	

}}
void Callibration::dcStartCalib()
{
	double rms;
	
	if (!dcSolution.KameraA.intrinsischeParameter.data  || !dcSolution.KameraA.verzerrungsParameter.data)
	{
		std::cout<<"Starte Kallibrierung Kamera 1"<<endl;
		rms=calibrateCamera(
				dcCamA->vvObjectPoints,
				dcCamA->vvImagePoints, 
				dcCamA->imageSize, 
				dcSolution.KameraA.intrinsischeParameter, 
				dcSolution.KameraA.verzerrungsParameter, 
				dcSolution.KameraA.vRotationsmatrizen, 
				dcSolution.KameraA.vTranslatoinsmatrizen,
				CV_CALIB_FIX_ASPECT_RATIO,
				TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-5));
		std::cout<<"Kallibrierung Kamera 1 beendet!"<<endl;
		Callibration::scPrintParameter(1,&dcSolution.KameraA);
		std::cout<<"Geometrischer Fehler betraegt "<< rms<<endl;
	}
	if (!dcSolution.KameraB.intrinsischeParameter.data  || !dcSolution.KameraB.verzerrungsParameter.data)
	{
		std::cout<<"Starte Kallibrierung Kamera 2"<<endl;
		rms=calibrateCamera(
				dcCamB->vvObjectPoints,
				dcCamB->vvImagePoints, 
				dcCamB->imageSize, 
				dcSolution.KameraB.intrinsischeParameter, 
				dcSolution.KameraB.verzerrungsParameter, 
				dcSolution.KameraB.vRotationsmatrizen, 
				dcSolution.KameraB.vTranslatoinsmatrizen,
				CV_CALIB_FIX_ASPECT_RATIO,
				TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-5));
		std::cout<<"Kallibrierung Kamera 2 beendet!"<<endl;
		Callibration::scPrintParameter(1,&dcSolution.KameraB);
		std::cout<<"Geometrischer Fehler betraegt "<< rms<<endl;
	}
	std::cout<<"Starte Kallibrierung Extrinsische Parameter"<<endl;
	rms=stereoCalibrate(
		dcCamA->vvObjectPoints,
		dcCamA->vvImagePoints,
		dcCamB->vvImagePoints, 
		dcSolution.KameraA.intrinsischeParameter,
		dcSolution.KameraA.verzerrungsParameter,
		dcSolution.KameraB.intrinsischeParameter,
		dcSolution.KameraB.verzerrungsParameter,
		dcCamA->imageSize,
		dcSolution.Rotation,
		dcSolution.Translation,
		dcSolution.Essential,
		dcSolution.Fundamental,
		TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-5),
		CV_CALIB_FIX_INTRINSIC+CV_CALIB_USE_INTRINSIC_GUESS 
		);
	cout<<dcCamA->imageSize<<endl;
	std::cout<<"Kallibrierung beendet!"<<endl;
	Callibration::dcPrintParameter(2);
	std::cout<<"Geometrischer Fehler betraegt "<< rms<<endl;
}
void Callibration::dcuiAddPictures()
{while(1){

	cout<<"[1] Einzelnes Bild aus Datei einlesen"<<endl;
	cout<<"[2] Bild aus Webcam"<<endl;
	cout<<"[sonst] Abbrechen"<<endl;

	int input=0;
	cin>>input;
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(),'\n');

	switch (input)
	{
		case 1: dcAddPictureFromFile();break;
		case 2: dcAddPicturesFromWebcam();break;
		default:return;
	}	
}}
void Callibration::dcAddPictureFromFile()
{
	char number[20];
	string pictureNameA="dcCamAPic";
	string pictureNameB="dcCamBPic";
	string pathName, saveNameA, saveNameB;
	Mat imageA, imageB;
	
	for (unsigned int i = 0;i< 10000 ;++i)
	{
		sprintf(number,"%d",i);
		saveNameA=Settings::calibrateFilenamePictures+pathName+pictureNameA+number+".jpg";
		saveNameB=Settings::calibrateFilenamePictures+pathName+pictureNameB+number+".jpg";
		try
		{
			imageA = imread(saveNameA, CV_LOAD_IMAGE_COLOR); 
			imageB = imread(saveNameB, CV_LOAD_IMAGE_COLOR); 
		}
		catch(...)
		{
			std::cout <<  "Oeffnen der Datei fehlgeschlagen!" << std::endl ;
			break;
		}
		if(!imageA.data || !imageB.data) 
		{
			std::cout <<  "Oeffnen der Datei fehlgeschlagen!" << std::endl ;
			break;
		}
		if(Settings::globalDebugshowPictures)
		{
			Webcam::showTwoPictures(imageA,imageB);
		}
		if(	dcCamA->addPicture(imageA)	)
		{
			
			if(	dcCamB->addPicture(imageB) )
			{
				continue;//erfolgreich
			}
			else
			{
				dcCamA->deleteLastPicture();
				cout<<"Letztes Bild wird geloescht,da in der zweiten Kamera kein Match gefunden wurde!"<<endl;
			}
		}
		cout<<endl;
	}
}
void Callibration::dcSaveGoodPictures()
{	
	char number[20];
	string pictureNameA="dcCamAPic";
	string pictureNameB="dcCamBPic";
	string pathName, saveNameA, saveNameB;
	vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
	for (unsigned int i = 0;i< dcCamA->goodPictures.size();++i)
	{
		sprintf(number,"%d",i);
		saveNameA=Settings::calibrateFilenamePictures+pathName+pictureNameA+number+".jpg";
		saveNameB=Settings::calibrateFilenamePictures+pathName+pictureNameB+number+".jpg";
		try
		{
			cout << "Saving: " << saveNameA  <<endl;
			imwrite(saveNameA, dcCamA->goodPictures[i],compression_params);
			cout << "Saving: " << saveNameB  <<endl;
			imwrite(saveNameB, dcCamB->goodPictures[i],compression_params);
		}
		catch(...)
		{cout<<"Speichern hat nicht geklappt."<<endl;}
	}
}
void Callibration::dcDeletePictures()
{
	dcCamA->deleteLastPicture();
	dcCamB->deleteLastPicture();
}

void Callibration::dcAddPicturesFromWebcam()
{
	Webcam *wec=new Webcam(2);
	int AnzBilder=0;
	wec->takePictures(usedChessboard->chessboardSize);
	vector<Mat> imagesA = wec->getTakenPicture(1);
	vector<Mat> imagesB = wec->getTakenPicture(2);
	delete wec;
	if (dcCamA->goodPictures.size() != dcCamB->goodPictures.size() )
	{
		cout<<"Alles kaputt in den bisherigen Bildern! Nochmal neu machen! Bilder werden geloescht!"<<endl;
		dcCamA->goodPictures.clear();
		dcCamB->goodPictures.clear();
	}
	if (imagesA.size() != imagesB.size() )
	{
		cout<<"Alles kaputt in den aufgenommenen Bildern! Nochmal neu machen! Bilder werden geloescht!"<<endl;
		wec->deleteTakenPictures();
	}
	if (imagesA.size()==0)return;

	for(unsigned int j=0; j< imagesA.size(); ++j) //alle Bilder
	{
		if(Settings::globalDebugshowPictures)
		{
			Webcam::showTwoPictures(imagesA[j],imagesB[j]);
		}

		if(	dcCamA->addPicture(imagesA[j])	)
		{
			if(	dcCamB->addPicture(imagesB[j])	)
			{
				//Beide Bilder aufgenommen
				continue;//erfolgreich
			}
			else
			{
				//Bild A wurde aufgenommen Bild B aber nicht
				//--> Bild A muss gelöscht werden
				dcCamA->deleteLastPicture();
				cout<<"Letztes Bild wird geloescht,da in der zweiten Kamera kein Match gefunden wurde!"<<endl;
			}
			dcPrintParameter(3);
		}
		cout<<endl;
		cout<<"Anzahl Bilder: "<<dcCamA->vvObjectPoints.size()<<" hinzugefuegt."<<endl;
	}
	cout<<"Bildaufnahme Beendet! "<<AnzBilder<<" Bilder hinzugefuegt."<<endl;
	destroyAllWindows();
	
}
#pragma endregion
#pragma region einzelen Kamera
void Callibration::scSaveGoodPictures(string pathName)
{	
	
	char number[20];
	string pictureName="scCamAPic";
	string saveName;
	vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
	cout<<"test";
	for (unsigned int i = 0;i< scCam->goodPictures.size();++i)
	{
		sprintf(number,"%d",i);
		saveName=Settings::calibrateFilenamePictures+pathName+pictureName+number+".jpg";
		try
		{
			cout << "Saving: " << saveName <<endl;
			imwrite(saveName, scCam->goodPictures[i],compression_params);
		}
		catch(...)
		{cout<<"nicht genuegend Rechte um Bild zu speichern?"<<endl;}
	}
}

void Callibration::uiSingleCam()
{while(1){
	std::cout<<"Anzahl gueltiger Bilder: "<<scCam->vvImagePoints.size()<<endl;
	std::cout<<"-------------------------------------------------------"<<endl;
	std::cout<<"[1] Kalibrierung starten"<<endl;
	std::cout<<"[2] Bilder hinzufuegen"<<endl;
	std::cout<<"[3] Kalibrierdaten speichern"<<endl;
	std::cout<<"[11] Bilder speichern als Kamera A(Ueberspeichert evtl. alte Bilder)"<<endl;
	std::cout<<"[12] Bilder speichern als Kamera B(Ueberspeichert evtl. alte Bilder)"<<endl;
	std::cout<<"[13] Kalibrierungsdaten an Stereokalibrierung geben und loeschen"<<endl;
	std::cout<<"-------------------------------------------------------"<<endl;
	std::cout<<"[20] Ausgabe der intrinischen Parameter"<<endl;
	std::cout<<"[21] Ausgabe der extrinischen Parameter"<<endl;
	std::cout<<"[22] Ausgabe der Kalibrierungswerte"<<endl;
	std::cout<<"-------------------------------------------------------"<<endl;
	cout<<"[sonst] Abbrechen"<<endl;

	int input=0;
	cin>>input;
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(),'\n');

	switch (input)
	{
		case 1: scStartCalib();break;
		case 2: scuiAddPictures();break;
		case 3: scSave();break;
		case 11: scSaveGoodPictures(Settings::calibrateFilenamePicturesCamA);break;
		case 12: scSaveGoodPictures(Settings::calibrateFilenamePicturesCamB);break;
		case 13: scuiUseCaliForStereo();break;
		case 20:scPrintParameter(1,&scSolution);break;
		case 21:scPrintParameter(2,&scSolution);break;
		case 22:scPrintParameter(3,&scSolution);break;
		default:return;
	}	

}}

void Callibration::scStartCalib()
{
	
	if (scCam->vvObjectPoints.size() == 0)
	{
		std::cout <<"Zu wenige Bilder um Kallibrierung durchzufuehren"<<endl;
		return;
	}
	cout<<scCam->imageSize<<endl;
	std::cout<<"Starte Kallibrierung!"<<endl;
	float rms=calibrateCamera(
			scCam->vvObjectPoints,
			scCam->vvImagePoints, 
			scCam->imageSize, 
			scSolution.intrinsischeParameter, 
			scSolution.verzerrungsParameter, 
			scSolution.vRotationsmatrizen, 
			scSolution.vTranslatoinsmatrizen,
			CV_CALIB_FIX_ASPECT_RATIO);
	std::cout<<"Kallibrierung beendet!"<<endl;
	std::cout<<"Geometrischer Fehler betraegt: "<<rms<<endl;
}
void Callibration::scuiUseCaliForStereo()
{
	cout<<"[1] Als Kamera A"<<endl;
	cout<<"[2] Als Kamera B"<<endl;
	cout<<"[sonst] Abbrechen"<<endl;

	int input=0;
	cin>>input;
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(),'\n');

	switch (input)
	{
		case 1: scUseCaliForStereo(&dcSolution.KameraA);break;
		case 2: scUseCaliForStereo(&dcSolution.KameraB);break;
		default:return;
	}
}

void Callibration::scUseCaliForStereo(struct Callibration::scKalib *dcCamX)
{
	Mat emptyMat;
	scSolution.intrinsischeParameter.copyTo(dcCamX->intrinsischeParameter);
	scSolution.verzerrungsParameter.copyTo(dcCamX->verzerrungsParameter);
	dcCamX->vRotationsmatrizen	 =	scSolution.vRotationsmatrizen;
	dcCamX->vTranslatoinsmatrizen=	scSolution.vTranslatoinsmatrizen;
	scSolution.intrinsischeParameter=emptyMat;
	scSolution.verzerrungsParameter=emptyMat;
	scSolution.vRotationsmatrizen.erase(scSolution.vRotationsmatrizen.begin(),scSolution.vRotationsmatrizen.end() );
	scSolution.vTranslatoinsmatrizen.erase(scSolution.vTranslatoinsmatrizen.begin(),scSolution.vTranslatoinsmatrizen.end() );
	cout<<scSolution.intrinsischeParameter<<endl;
	cout<<scSolution.verzerrungsParameter<<endl;
	 
	dcCamX=NULL;
	delete scCam;
	scCam=new Chessboard::Perspective(usedChessboard);
}

void Callibration::scPrintParameter(int i, struct Callibration::scKalib *scSolptr)
{
	if(i==1)
	{
		std::cout<<"IntrinsischeParameter"<<endl;
		std::cout<<scSolptr->intrinsischeParameter<<endl;

		std::cout<<"VerzerrungsParameter"<<endl;
		std::cout<<scSolptr->verzerrungsParameter<<endl;
	}
	else if(i==2)
	{
		std::cout<<"Extrinische Rotationsmatrizen"<<endl;
		for(unsigned int j=0; j< scSolptr->vRotationsmatrizen.size(); ++j)
		{
			std::cout<< scSolptr->vRotationsmatrizen[j]<<endl;  
		}
		std::cout<<"Extrinische Translationsmatrizen"<<endl;
		for(unsigned int j=0; j< scSolution.vTranslatoinsmatrizen.size(); ++j)
		{
			std::cout<< scSolptr->vTranslatoinsmatrizen[j]<<endl;  
		}
	}
	else if(i==3)
	{
		unsigned int counter = 0;
		std::cout<<"Objekt Punkte: "<<endl;
		for(unsigned int j=0; j< scCam->vvObjectPoints.size(); ++j)
		{
			for(unsigned int k=0; k< scCam->vvObjectPoints[j].size(); ++k)
				{
					std::cout<<scCam->vvObjectPoints[j][k]<<endl;
					counter++;
				}
		}
		std::cout<<endl<<"Insgesamt "<< counter <<" Eintraege"<<endl; 
		counter = 0;
		std::cout<<endl<<"Bild Punkte"<<endl;
		for(unsigned int j=0; j< scCam->vvImagePoints.size(); ++j)
		{
			for(unsigned int k=0; k< scCam->vvImagePoints[j].size(); ++k)
			{
				std::cout<<scCam->vvImagePoints[j][k]<<endl;
				counter++;
			}
		}
		std::cout<<endl<<"Insgesamt "<< counter <<" Eintraege"<<endl; 
		counter = 0;
		std::cout<<endl<<"Bild Groesse"<<endl;
		std::cout << scCam->imageSize;
	}
}

void Callibration::scuiAddPictures()
{while(1){
	cout<<"[1] Bilder aus Datei \"KameraA\\\"einlesen"<<endl;
	cout<<"[2] Bilder aus Datei \"KameraB\\\"einlesen"<<endl;
	cout<<"[3] Bild aus Webcam 1"<<endl;
	cout<<"[4] Bild aus Webcam 2"<<endl;
	cout<<"[sonst] Abbrechen"<<endl;

	int input=0;
	cin>>input;
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(),'\n');

	switch (input)
	{
		case 1: scAddPictureFromFile(Settings::calibrateFilenamePicturesCamA);break;
		case 2: scAddPictureFromFile(Settings::calibrateFilenamePicturesCamB);break;
		case 3: scAddPicturesFromWebcam(0);break;
		case 4: scAddPicturesFromWebcam(1);break;
		default:return;
	}
}}

void Callibration::scAddPicturesFromWebcam(int i)
{
	Settings::webcamFirstWebcam=Settings::webcamFirstWebcam+i;
	Webcam *wec=new Webcam(1);
	wec->takePictures(usedChessboard->chessboardSize);
	vector<Mat> images = wec->getTakenPicture(1);
	if(Settings::saveNotTestedPictures)wec->saveTakenPicturesInFile(1);
	delete wec;
	scCam->addPictures(images);
	Settings::webcamFirstWebcam=Settings::webcamFirstWebcam-i;
}

void Callibration::scAddPictureFromFile(string pathName)
{
	char number[20];
	string pictureName="scCamAPic";
	string saveName;
	Mat image;
	
	for (unsigned int i = 0;i< 10000 ;++i)
	{
		sprintf(number,"%d",i);
		saveName=Settings::calibrateFilenamePictures+pathName+pictureName+number+".jpg";
		try
		{
			image = imread(saveName, CV_LOAD_IMAGE_COLOR); 
		}
		catch(...)
		{
			std::cout <<  "Oeffnen der Datei fehlgeschlagen!" << std::endl ;
			break;
		}
		if(! image.data ) 
		{
			std::cout <<  "Oeffnen der Datei fehlgeschlagen!" << std::endl ;
			break;
		}
		scCam->addPicture(image);
	}
}

void Callibration::scSave()
{
	try{
	string filename = Settings::calibrateFilenameXML;
	std::cout<<"Speichere nach "<<filename<<endl;
	FileStorage fs(filename, FileStorage::WRITE);
	fs << "SingleCamCallibrationData"<<"{";
	fs << "chessboardSize" << usedChessboard->chessboardSize;
	fs << "SquareSize" << usedChessboard->SquareSize;
	fs << "}";
	fs << "SingleCamData"<<"{";
	fs << "scIntrinsischeParameter" << scSolution.intrinsischeParameter;
	fs << "scVerzerrungsParameter" << scSolution.verzerrungsParameter;
	fs << "vscRotationsmatrizen" << scSolution.vRotationsmatrizen;
	fs << "vscTranslationsmatrizen" << scSolution.vTranslatoinsmatrizen;
	fs << "}";

	fs.release();  
	}catch(...)
	{cout<<"Konnte Datei icht oeffnen!"<<endl;}
}
#pragma endregion



