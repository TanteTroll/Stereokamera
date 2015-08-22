// OpenCV_CameraCalibration.cpp : Definiert den Einstiegspunkt fuer die Konsolenanwendung.
//
#include "stdafx.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/highgui.h>
#include "opencv\cv.h"
#include "Callibration.h"
#include "Settings.h"
#include "Webcam.h"
#include <stdio.h>
#include <iostream>
#include "SinglePictureManipulation.h"
#include "TwoPictureManipulation.h"
#include <time.h>
#include <vector>

using namespace cv;
using namespace std;

Callibration *Kalli;
vector <Mat> uiGetPictures()
{
	vector<Mat> returnVector;
	while(1){

	cout<<"[1] Letztes Bildpaar nutzen"<<endl;
	cout<<"[2] Neue Bilder aus Webcam"<<endl;
	cout<<"[sonst] Abbrechen"<<endl;

	int input=0;
	cin>>input;
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(),'\n');
	
	switch (input)
	{
		case 1: 
			{
				returnVector.push_back(imread("Pictures\\IOPictures\\BildEins.jpg", CV_LOAD_IMAGE_COLOR)) ;
				returnVector.push_back(imread("Pictures\\IOPictures\\BildZwei.jpg", CV_LOAD_IMAGE_COLOR)) ;
				return returnVector;
			}
		case 2: 
			{
				Webcam *wec=new Webcam(2);
				wec->takePictures(Size(0,0));
				vector<Mat> imgA = wec->getTakenPicture(1);
				vector<Mat> imgB = wec->getTakenPicture(2);
				if (imgA.data() && imgB.data())
				{
					returnVector.push_back(imgA[0]);
					returnVector.push_back(imgB[0]);
					vector<int> compression_params;
					compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
					compression_params.push_back(9);
	
					imwrite("Pictures\\IOPictures\\BildEins.jpg", imgA[0],compression_params);
					imwrite("Pictures\\IOPictures\\BildZwei.jpg", imgB[0],compression_params);
					return returnVector;
				}
				else continue;
			}
		default:continue;
	}	
}}
void UI()
{
	int input=0;
	vector<Mat>tmp, dmap;
	Mat Q;
	Mat Punkte3D;
	Rect roi;
	while(1)
	{
		destroyAllWindows();
		cout<<"[100] Einstellungen aendern"<<endl;
		cout<<"[1] Neu Kalibrieren"<<endl;
		cout<<"[2] Kalibration aus Datei: "<<Settings::calibrateFilenameXML<<endl;
		cout<<"[3] Bilder auswaehlen"<<endl;
		cout<<"[4] Block Matching"<<endl;
		cout<<"[6] Pointcloud erstellen"<<endl;
		cout<<"[7] Abstand messen"<<endl;
		Webcam* test;
		cin>>input;
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(),'\n');
		switch (input)
		{
			case 100: {Callibration::uiChangeSettings();continue;}
			case -2: {test=new Webcam(1);test->saveVideo();continue;}
			case -1: {SinglePictureManipulation::get3dFromPicture();continue;}
			case 0: {Kalli->showMainUI();continue;}
			case 1:	{Kalli=new Callibration(); continue;}
			case 2: {Kalli=new Callibration(Settings::calibrateFilenameXML);continue;}
			case 3:
				{
					cout<<Kalli->dcSolution.KameraA.intrinsischeParameter.dims;
					if(Kalli->dcSolution.KameraA.intrinsischeParameter.dims==0)
					{
						cout<<"Kalibration fehlgeschlagen"<<endl;
						throw;
					}
					destroyAllWindows();
					vector<Mat> zweiBilder;
					zweiBilder=uiGetPictures();
					Mat imageA=zweiBilder[0];
					Mat imageB=zweiBilder[1];
					
					
					if (imageA.data && imageB.data)
					{
						Size imageSize = imageA.size();
						clock_t t;
						t = clock();
						tmp =TwoPictureManipulation::rectify(imageA,imageB,
							Kalli->dcSolution.KameraA.intrinsischeParameter,
							Kalli->dcSolution.KameraA.verzerrungsParameter,
							Kalli->dcSolution.KameraB.intrinsischeParameter,
							Kalli->dcSolution.KameraB.verzerrungsParameter,
							imageSize,
							Kalli->dcSolution.Rotation,
							Kalli->dcSolution.Translation,
							Q,roi);
						t = clock() - t;
						cout << "Rektifizieren dauerte "<<((float)t)/CLOCKS_PER_SEC<<" Sekunden!"<<endl;
						int newWidth=1024;//960;
						int newHeight=640;//600;
						resize(tmp[0], tmp[0], Size(newWidth,newHeight), newWidth, newHeight);
						resize(tmp[1], tmp[1], Size(newWidth,newHeight), newWidth, newHeight);
						resize(tmp[2], tmp[2], Size(newWidth,newHeight), newWidth, newHeight);
						resize(tmp[3], tmp[3], Size(newWidth,newHeight), newWidth, newHeight);
						roi.x=roi.x*newWidth/Settings::cameraresWidth+150;
						roi.y=roi.y*newHeight/Settings::cameraresHight;
						roi.width=roi.width*newWidth/Settings::cameraresWidth-50;
						roi.height=roi.height*newHeight/Settings::cameraresHight;
						tmp[2]=tmp[0](roi);
					}
					else cout<<"keine Bilder aufgenommen"<<endl;
					continue;
				}
			case 4: 
				{
					//tmp[0]=(imread("Pictures\\IOPictures\\l.bmp", CV_LOAD_IMAGE_COLOR)) ;
					//tmp[1]=(imread("Pictures\\IOPictures\\r.bmp", CV_LOAD_IMAGE_COLOR)) ;
					dmap.clear();
					dmap=TwoPictureManipulation::DispMapSlow(tmp[0],tmp[1],roi);
					break;
				}
			case 5: 
				{
					dmap.clear();
					dmap=TwoPictureManipulation::DispMapFast(tmp[0],tmp[1],roi);
					break;
				}
			case 6: 
				{
					Punkte3D=TwoPictureManipulation::reproject2(dmap[0],Q,tmp[2]);
					break;
				}
			case 7: 
				{
					TwoPictureManipulation::Measurement(tmp[2],Punkte3D,false); 
					break;
				}

			default: continue;
		}
	}
}

int main( int argc, char** argv )
{
	UI();
  return 0;
}

