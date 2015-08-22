#include "stdafx.h"
#include "SinglePictureManipulation.h"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <iostream>
#include <stdlib.h>
#include "Settings.h"
#include "Webcam.h"
#include "TwoPictureManipulation.h"
using namespace cv;
using namespace std;

SinglePictureManipulation::SinglePictureManipulation(void)
{
}


SinglePictureManipulation::~SinglePictureManipulation(void)
{
}
Mat SinglePictureManipulation::getFeaturesAsMat(Mat image)
{
	if( !image.data  )
		 { std::cout<< " --(!) Error reading images " << std::endl; throw; }
	Mat imgKeypoints;
	
		GoodFeaturesToTrackDetector detector(2000,0.000001,1.,3,1);

	std::vector<KeyPoint> keypoints;
	detector.detect( image, keypoints );
	  
	drawKeypoints( image, keypoints, imgKeypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	
	return imgKeypoints;
}


std::vector<KeyPoint> SinglePictureManipulation::getFeatures(Mat image)
{
	 if( !image.data  )
		 { std::cout<< " --(!) Error reading images " << std::endl; throw; }

	  int minHessian = Settings::featureFindMinHessian;//Only features, whose hessian is larger than hessianThreshold are retained by the detector. Therefore, the larger the value, the less keypoints you will get. A good default value could be from 300 to 500, depending from the image contrast.

	  SurfFeatureDetector detector( minHessian );

	  std::vector<KeyPoint> keypoints;
	  detector.detect( image, keypoints );
	  
	  
	  return keypoints;
}

vector < vector<KeyPoint> > SinglePictureManipulation::getFeatures (vector <Mat> vimages)
{
	vector < vector<KeyPoint> > vReturn;
	for (unsigned int i = 0;i< vimages.size();++i)
	{
		vReturn.push_back( getFeatures(vimages[i]) );
	}
	return(vReturn);
}
vector < Mat > SinglePictureManipulation::getFeaturesAsMat (vector <Mat> vimages)
{
	vector < Mat > vReturn;
	for (unsigned int i = 0;i< vimages.size();++i)
	{
		vReturn.push_back( getFeaturesAsMat(vimages[i]) );
	}
	return(vReturn);
}

	
void SinglePictureManipulation::get3dFromPicture()
{
	
	Mat picture,frame;
	bool addRemovePt = false;
	//Kameraparameter einlesen für undistort
	Mat cameraMatrix, distCoeff, cameraNew;
	std::cout<<"Versuche Datei "<<Settings::calibrateFilenameXML<<" zu oeffnen."<<endl;
	FileStorage fs(Settings::calibrateFilenameXML, FileStorage::READ);
	if( !fs.isOpened())
	{
		std::cout<<"Datei "<<Settings::calibrateFilenameXML<<" konnte nicht geoeffnet werden."<<endl;
		throw;
	}
	FileNode n = fs["DoubleCamExtrinsic"];
	FileNode m = n["DoubleCamAData"];
	m["dcCAMAIntrinsischeParameter"] >> cameraMatrix;
	m["dcCAMAVerzerrungsParameter"] >> distCoeff;
	cout<<"Kameraparameter erfolgreich eingelesen!"<<endl;
	
	
	//Bilder aus Video einlesen
	Mat curPic,prevPic,curPic_tmp;
	vector<Point2f> curpoints, prevpoints, addpoints;
	vector <Mat> video;
	
	try
	{
		video = Webcam::readVideo("MyVideo.avi");
	}
	catch(...)
	{
		cout<<"kein Video vorhanden"<<endl;
	}
	for (int framenumber = 0; framenumber<video.size();framenumber++)
	{
		
		//Bidler einlesen
		cout<<" Bilder werden eingelesen"<<endl;
		curPic.copyTo(prevPic);	//altes Bild
		prevpoints.clear();
		prevpoints=curpoints;
		curpoints.clear();	//neues Bild
		video.at(framenumber).copyTo(curPic);
		cvtColor(curPic , curPic_tmp , CV_BGR2GRAY);
		
		//Bild entzerren
		cameraMatrix.copyTo(cameraNew);
		undistort(curPic_tmp, curPic, cameraMatrix, distCoeff,cameraNew);

		//Initialisierung
		cout<<"Variableninitialisierung"<<endl;
		const int NUMBER_OF_KEYPOINTS = 500;	//keypoints anzahl
		Size winSize(10,10);					//abgesuchtes Fenster
		TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
		
		//Neue Keypoints hinzufügen
		addpoints.clear();
		cout<<"Keypoints suchen"<<endl;
		goodFeaturesToTrack(curPic, addpoints , NUMBER_OF_KEYPOINTS, 0.01, 10, Mat(), 3, 0, 0.04);
		cornerSubPix(curPic, addpoints, winSize, Size(-1,-1), termcrit);
		cout<<"neue Keypoints anhaengen"<<endl;
		for( int i=0; (prevpoints.size()<NUMBER_OF_KEYPOINTS)&&(i<addpoints.size());i++)
		{
			if( find(prevpoints.begin(),prevpoints.end(),addpoints[i])==prevpoints.end() )
				prevpoints.push_back(addpoints[i]);
		}//prevpoints.insert(prevpoints.end(),addpoints.begin(),addpoints.end() );

		//erstes Bild überprüfen und ggf. abbrechen
		if ( !prevPic.data ) continue;

		//DEBUG
		if(1)
		{
			imshow("1",prevPic);imshow("2",curPic);waitKey(0);destroyAllWindows();
		}
		//korrespondierende Punkte finden
		cout<<"korrespondierende Punkte finden"<<endl;
		vector<uchar> status;
		vector<float> err;
		calcOpticalFlowPyrLK(prevPic, curPic, prevpoints, curpoints, status, err,
			winSize, 3, termcrit, 0);
	        
		//nicht gefundene Punkte rauswerfen
		cout<<"nicht gefundene Punkte rauswerfen"<<endl;
		int numberOfLostPoints=0;
		for (int i=0;i< curpoints.size();i++)
		{
			if(!status[i])//kein Punkt gefunden
			{
				curpoints.erase(curpoints.begin()+i);
				prevpoints.erase(prevpoints.begin()+i);
				numberOfLostPoints++;
			}
		}
		cout<<numberOfLostPoints<<" Punkte verloren"<<endl;

		//Fundamentalmatrix und Essentielle Matrix finden
		cout<<"Fundamentalmatrix finden"<<endl;
		cout<<"DEBUG"<<endl;
		cout<<prevpoints.size()<<endl;
		cout<<curpoints.size()<<endl;

		Mat F =	findFundamentalMat(prevpoints,curpoints,FM_RANSAC, 3, 0.99);
		cout<<"Berechne Essentielle Matrix"<<endl;
		cout<<F<<endl<<endl<<cameraNew<<endl;
		Mat E = cameraNew.t()*F*cameraNew;

		//R und T aus Essentieller Matrix
		cout<<"R und T aus Essentieller Matrix"<<endl;
		SVD svd(E,SVD::MODIFY_A);
		Matx33d W(		0,-1, 0,
						1, 0, 0,
						0, 0, 1 );
		Mat_<double> R = svd.u * Mat(W).t() * svd.vt; //or svd.u * Mat(W) * svd.vt; 
		Mat_<double> t = svd.u.col(2); //or -svd.u.col(2)

		Matx34d P1(		R(0,0),    R(0,1), R(0,2), t(0),
						R(1,0),    R(1,1), R(1,2), t(1),
						R(2,0),    R(2,1), R(2,2), t(2)		);

	}
	//TwoPictureManipulation::rectify( prevPic, curPic, cameraNew, cameraNew ,
	////Bild anzeigen
	//resize(frame,picture,Size(800,600));
	//imshow("testeiger",picture);
	//waitKey(30);


	return;
}