#include "stdafx.h"
#include "TwoPictureManipulation.h"
#include "Settings.h"
#include "Callibration.h"
#include "SinglePictureManipulation.h"
#include "Webcam.h"
#include "Menue.h"
#include "model3d.h"
#include <time.h>

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"

using namespace cv;
using namespace std;


StereoSGBM sgbm;
bool sgbmInit=false;
StereoBM sbm;
bool sbmInit=false;

void getTwoPoints();
double calcDistance(Point3f p1, Point3f p2);

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	vector <int> *punkte = static_cast <vector <int> *> (userdata);
	punkte->push_back(0);punkte->push_back(0);punkte->push_back(0);punkte->push_back(0);
     if  ( event == EVENT_LBUTTONDOWN )
     {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		  punkte->at(0)=x;
		  punkte->at(1)=y;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		  punkte->at(2)=x;
		  punkte->at(3)=y;
     }
}
void TwoPictureManipulation::drawEpilines(Mat img1, Mat img2, Mat fundemental, unsigned int NumberOfEpilinesPrinted=10)
{
	Mat manipulatedImg1, manipulatedImg2, manipulatedImg3, manipulatedImg4;
	img1.copyTo(manipulatedImg1);img2.copyTo(manipulatedImg2);
	img1.copyTo(manipulatedImg3);img2.copyTo(manipulatedImg4);
	vector<KeyPoint> punkte1 = SinglePictureManipulation::getFeatures(manipulatedImg1);
	vector<KeyPoint> punkte2 = SinglePictureManipulation::getFeatures(manipulatedImg2);
	cout<<"Suche Punkte"<<endl;
	manipulatedImg1 = SinglePictureManipulation::getFeaturesAsMat(manipulatedImg1);
	manipulatedImg2 = SinglePictureManipulation::getFeaturesAsMat(manipulatedImg2);
	manipulatedImg3 = SinglePictureManipulation::getFeaturesAsMat(manipulatedImg3);
	manipulatedImg4 = SinglePictureManipulation::getFeaturesAsMat(manipulatedImg4);

	std::vector<cv::Point2f> selPoints1, selPoints2;
	cv::KeyPoint::convert(punkte1,selPoints1);
	cv::KeyPoint::convert(punkte2,selPoints2);

	std::vector<cv::Vec3f> lines1;
	std::vector<cv::Vec3f> lines2;
	std::vector<cv::Vec3f> lines3;
	std::vector<cv::Vec3f> lines4;
	//Linien berechnen
	cout<<"Berechne Linien"<<endl;
	computeCorrespondEpilines( 
		Mat(selPoints1), 
		2,               //2-> im eigenen Bild 1-> im fremden Bild
		fundemental,	// F matrix
		lines1);
	computeCorrespondEpilines( 
		Mat(selPoints2), // image points 
		2,               // in image 1 (can also be 2)
		fundemental,	// F matrix
		lines3);
	computeCorrespondEpilines( 
		Mat(selPoints1), 
		1,               //2-> im eigenen Bild 1-> im fremden Bild
		fundemental,	// F matrix
		lines1);
	computeCorrespondEpilines( 
		Mat(selPoints2), // image points 
		1,               // in image 1 (can also be 2)
		fundemental,	// F matrix
		lines4);
	//Linien zeichnen
	int iterator1=lines1.size()/NumberOfEpilinesPrinted;
	int iterator2=lines2.size()/NumberOfEpilinesPrinted;
	int iterator3=lines3.size()/NumberOfEpilinesPrinted;
	int iterator4=lines4.size()/NumberOfEpilinesPrinted;
	iterator1=iterator1<1?1:iterator1;
	iterator2=iterator2<1?1:iterator2;
	iterator3=iterator3<1?1:iterator3;
	iterator4=iterator4<1?1:iterator4;
	//Im eigenen Bild
	for (unsigned int i=0;i<lines1.size();i=i+iterator1) {
		Vec3f tmpVekt=lines1[i];
		int x1=0;
		int x2=manipulatedImg1.cols;
		int y1=(-tmpVekt[2]/tmpVekt[1])-((tmpVekt[0]*x1)/tmpVekt[1]);
		int y2=-(tmpVekt[2]/tmpVekt[1])-((tmpVekt[0]*x2)/tmpVekt[1]);
		cv::line(manipulatedImg1,Point(x1,y1), Point(x2,y2), Scalar(255,0,0));
	}

	for (unsigned int i=0;i<lines2.size();i=i+iterator2) {
		Vec3f tmpVekt=lines2[i];
		int x1=0;
		int x2=manipulatedImg2.cols;
		int y1=(-tmpVekt[2]/tmpVekt[1])-((tmpVekt[0]*x1)/tmpVekt[1]);
		int y2=-(tmpVekt[2]/tmpVekt[1])-((tmpVekt[0]*x2)/tmpVekt[1]);
		cv::line(manipulatedImg2,Point(x1,y1), Point(x2,y2), Scalar(255,0,0));
	}
	//im fremden Bild
	for (unsigned int i=0;i<lines3.size();i=i+iterator3) {
		Vec3f tmpVekt=lines3[i];
		int x1=0;
		int x2=manipulatedImg1.cols;
		int y1=(-tmpVekt[2]/tmpVekt[1])-((tmpVekt[0]*x1)/tmpVekt[1]);
		int y2=-(tmpVekt[2]/tmpVekt[1])-((tmpVekt[0]*x2)/tmpVekt[1]);
		cv::line(manipulatedImg3,Point(x1,y1), Point(x2,y2), Scalar(255,0,0));
	}

	for (unsigned int i=0;i<lines4.size();i=i+iterator4) {
		Vec3f tmpVekt=lines4[i];
		int x1=0;
		int x2=manipulatedImg2.cols;
		int y1=(-tmpVekt[2]/tmpVekt[1])-((tmpVekt[0]*x1)/tmpVekt[1]);
		int y2=-(tmpVekt[2]/tmpVekt[1])-((tmpVekt[0]*x2)/tmpVekt[1]);
		cv::line(manipulatedImg4,Point(x1,y1), Point(x2,y2), Scalar(255,0,0));
	}
	imshow("Epilinien im eigenen Bild: Bild 1",manipulatedImg1);
	imshow("Epilinien im eigenen Bild: Bild 2",manipulatedImg2);
	imshow("Epilinien im fremden Bild: Bild 1",manipulatedImg3);
	imshow("Epilinien im fremden Bild: Bild 2",manipulatedImg4);
	waitKey(0);
	destroyAllWindows();
}
vector < vector<cv::Vec3f> > TwoPictureManipulation::getAllEpilineVector(Mat img1, Mat img2, Mat fundemental, int whichPicture)
{
	whichPicture=whichPicture>=2?2:1;
	Mat manipulatedImg1, manipulatedImg2;
	img1.copyTo(manipulatedImg1);img2.copyTo(manipulatedImg2);

	vector<KeyPoint> punkte1 = SinglePictureManipulation::getFeatures(manipulatedImg1);
	vector<KeyPoint> punkte2 = SinglePictureManipulation::getFeatures(manipulatedImg2);
	std::vector<cv::Point2f> selPoints1, selPoints2;
	cv::KeyPoint::convert(punkte1,selPoints1);
	cv::KeyPoint::convert(punkte2,selPoints2);

	std::vector<cv::Vec3f> lines1;
	std::vector<cv::Vec3f> lines2;

	computeCorrespondEpilines( 
		Mat(selPoints1), 
		whichPicture,               //2-> im eigenen Bild 1-> im fremden Bild
		fundemental,	// F matrix
		lines1);
	computeCorrespondEpilines( 
		Mat(selPoints2), // image points 
		whichPicture,               // in image 1 (can also be 2)
		fundemental,	// F matrix
		lines2);

	vector < vector<cv::Vec3f> > returnVec;
	returnVec.push_back(lines1);returnVec.push_back(lines2);
	return returnVec;
}
vector<Mat> TwoPictureManipulation::rectify(Mat img1, Mat img2,	Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Size imageSize, Mat R, Mat T,Mat &OutputQ,Rect &OutputROI)
{
	if( !img1.data ||  !img2.data )
		 { std::cout<< " --(!) Error reading images " << std::endl; throw; }

	Mat R1,R2,P1,P2;
	Rect validRoi[2];

	//Compute rectification transforms for each head of a calibrated stereo camera.
	stereoRectify(
		cameraMatrix1, 
		distCoeffs1, 
		cameraMatrix2, 
		distCoeffs2, 
		imageSize, 
		R, T, 
		R1, R2, P1, P2, OutputQ, 
		 CV_CALIB_ZERO_DISPARITY,
		-1,
		imageSize, 
		&validRoi[0], &validRoi[1]);
	OutputROI=validRoi[0];
	//Compute the undistortion and rectification transformation map
	Mat rmap[2][2];//CV_16SC2//CV_32FC1
	initUndistortRectifyMap(cameraMatrix1,distCoeffs1,R1,P1,imageSize,CV_32FC1 ,rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix2,distCoeffs2,R2,P2,imageSize,CV_32FC1 ,rmap[1][0], rmap[1][1]);
	
	//remap image
	Mat rimg1,rimg2;
	remap(img1,rimg1,rmap[0][0], rmap[0][1],INTER_LINEAR);
	remap(img2,rimg2,rmap[1][0], rmap[1][1],INTER_LINEAR);	

	//Linien reinmalen
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
	Mat canvas, cimg;
    double sf;
    int w, h;
	if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }
	
	Mat canvasPart1 = !isVerticalStereo ? canvas(Rect(w*0, 0, w, h)) : canvas(Rect(0, h*0, w, h));
	Mat canvasPart2 = !isVerticalStereo ? canvas(Rect(w*1, 0, w, h)) : canvas(Rect(0, h*1, w, h));

	resize(rimg1, canvasPart1, canvasPart1.size(), 0, 0, INTER_AREA);
	resize(rimg2, canvasPart2, canvasPart2.size(), 0, 0, INTER_AREA);

	Rect vroi1(cvRound(validRoi[0].x*sf), cvRound(validRoi[0].y*sf),
                          cvRound(validRoi[0].width*sf), cvRound(validRoi[0].height*sf));
	Rect vroi2(cvRound(validRoi[1].x*sf), cvRound(validRoi[1].y*sf),
                          cvRound(validRoi[1].width*sf), cvRound(validRoi[1].height*sf));

	
	
     rectangle(canvasPart1, vroi1, Scalar(0,0,200), 3, 8);
	 rectangle(canvasPart2, vroi2, Scalar(0,0,200), 3, 8);

	 if( !isVerticalStereo )
            for( int j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 100, 0), 1, 8);
        else
            for( int j = 0; j < canvas.cols; j += 16 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 100, 0), 1, 8);
	
	if(Settings::globalDebugshowPictures){
    imshow("rectified1", canvas);
	waitKey(0);}
	Mat roiImage1(rimg1,validRoi[0]);
	Mat roiImage2(rimg2,validRoi[1]);

	vector<Mat> returnMat;
	returnMat.push_back(rimg2);
	returnMat.push_back(rimg1);
	
	returnMat.push_back(roiImage2);
	returnMat.push_back(roiImage1);
	

	return returnMat;
}
vector<Mat> TwoPictureManipulation::DispMapSlow(Mat img1,Mat img2, Rect roi)
{
	vector<Mat> returnVector;
	int input;
	int value;
	clock_t t;	
	if (!::sgbmInit)
	{
		sgbm.SADWindowSize = 3;
		sgbm.numberOfDisparities = 256;
		sgbm.preFilterCap = 1;
		sgbm.minDisparity = 0;
		sgbm.uniquenessRatio = 2;
		sgbm.speckleWindowSize = 75;
		sgbm.speckleRange = 100;
		sgbm.disp12MaxDiff = 75;
		sgbm.fullDP = false;

		sgbmInit=true;
	}
	cvtColor(img1, img1, CV_BGR2GRAY);
	cvtColor(img2, img2, CV_BGR2GRAY);

	Mat dispSBM,dispSBMnorm,dispSBM8;

	while(true)
	{
		try
		{
			sgbm.P1 = 4*sgbm.SADWindowSize*sgbm.SADWindowSize;
			sgbm.P2 = 32*sgbm.SADWindowSize*sgbm.SADWindowSize;
			t = clock();
			sgbm(img1, img2, dispSBM);
			normalize(dispSBM, dispSBMnorm, 0, 255, CV_MINMAX, CV_8U);
			t = clock() - t;
			cout << "Block-Matching dauerte "<<((float)t)/CLOCKS_PER_SEC<<" Sekunden!"<<endl;
			dispSBM=dispSBM(roi);
			
			dispSBMnorm=dispSBMnorm(roi);
		
			imshow("SlowDisparityMap",dispSBMnorm);
			waitKey(30);
		}
		catch(...)
		{
			cout<<"Falschen Wert eingegeben!"<<endl;
			cout<<"Rueckgaengig machen!"<<endl<<endl;
		}
		Menue::printMenueEntryMid(-2,"refresh Image");
		Menue::printMenueEntryMid(-1,"Abbrechen");
		Menue::printMenueEntryMid(1,"SADWindowSize",sgbm.SADWindowSize);
		Menue::printMenueEntryMid(2,"preFilterCap",sgbm.preFilterCap);
		Menue::printMenueEntryMid(3,"numberOfDisparities",sgbm.numberOfDisparities);
		Menue::printMenueEntryMid(4,"minDisparity",sgbm.minDisparity);
		Menue::printMenueEntryMid(5,"uniquenessRatio",sgbm.uniquenessRatio);
		Menue::printMenueEntryMid(6,"speckleWindowSize",sgbm.speckleWindowSize);
		Menue::printMenueEntryMid(7,"speckleRange",sgbm.speckleRange);
		Menue::printMenueEntryMid(8,"disp12MaxDiff",sgbm.disp12MaxDiff);
				
		Menue::readValue(&input);
		cout<<"Neuen Wert eingeben: "<<endl;
		switch (input)
		{
			case -2:{destroyWindow("SlowDisparityMap");imshow("SlowDisparityMap",dispSBMnorm);waitKey(30);break;}
			case -1: {returnVector.push_back(dispSBM); returnVector.push_back(dispSBMnorm); return returnVector;}
			case 1:{Menue::readValue(&sgbm.SADWindowSize);break;}
			case 2:{Menue::readValue(&sgbm.preFilterCap);break;}
			case 3:{Menue::readValue(&sgbm.numberOfDisparities);break;}
			case 4:{Menue::readValue(&sgbm.minDisparity);break;}
			case 5:{Menue::readValue(&sgbm.uniquenessRatio);break;}
			case 6:{Menue::readValue(&sgbm.speckleWindowSize);break;}
			case 7:{Menue::readValue(&sgbm.speckleRange);break;}
			case 8:{Menue::readValue(&sgbm.disp12MaxDiff);break;}
			default : continue;
		}
	}
}
vector<Mat> TwoPictureManipulation::DispMapFast(Mat img1, Mat img2,Rect roi)
{
	vector<Mat> returnVector;
	if(!sbmInit)
	{
		sbm.state->SADWindowSize = 11;
		sbm.state->preFilterSize = 11;
		sbm.state->numberOfDisparities = 256;//durch 16 teilbar
		sbm.state->preFilterCap = 20;
		sbm.state->minDisparity = -176;
		sbm.state->uniquenessRatio = 8;
		sbm.state->speckleWindowSize = 0;
		sbm.state->speckleRange = 0;
		sbm.state->disp12MaxDiff = 100;
		sbm.state->textureThreshold = 500;

		sbmInit=true;
	}
	int input;
	int value;
	cvtColor(img1, img1, CV_BGR2GRAY);
	cvtColor(img2, img2, CV_BGR2GRAY);
	Mat dispSGBM,dispSGBMnorm,dispSGBM8;
	while(true)
	{
		try
		{
			sbm(img1, img2, dispSGBM,CV_32F);
			normalize(dispSGBM, dispSGBMnorm, 0, 255, CV_MINMAX, CV_8U);
			dispSGBM=dispSGBM(roi);
			dispSGBMnorm=dispSGBMnorm(roi);
			imshow("FastDisparityMap",dispSGBMnorm);
			waitKey(5);
		}
		catch(...)
		{
			cout<<"Falschen Wert eingegeben!"<<endl;
			cout<<"Rueckgaengig machen!"<<endl<<endl;
		}

		cout<<"[-1] Abbrechen"<<endl;
		cout<<"[1] SADWindowSize       "<<sbm.state->SADWindowSize<<endl;
		cout<<"[2] preFilterSize       "<<sbm.state->preFilterSize<<endl;
		//cout<<"[3] numberOfDisparities "<<sbm.state->numberOfDisparities<<endl;
		cout<<"[4] preFilterCap        "<<sbm.state->preFilterCap<<endl;
		//cout<<"[5] minDisparity        "<<sbm.state->minDisparity<<endl;
		cout<<"[6] uniquenessRatio     "<<sbm.state->uniquenessRatio<<endl;
		cout<<"[7] speckleWindowSize   "<<sbm.state->speckleWindowSize<<endl;
		cout<<"[8] speckleRange        "<<sbm.state->speckleRange<<endl;
		cout<<"[9] disp12MaxDiff       "<<sbm.state->disp12MaxDiff<<endl;
		cout<<"[10] textureThreshold   "<<sbm.state->textureThreshold<<endl;

		Menue::readValue(&input);
		cout<<"Neuen Wert eingeben"<<endl;
		switch (input)
		{
			case -1: {returnVector.push_back(dispSGBM); returnVector.push_back(dispSGBMnorm); return returnVector;}
			case 1:{Menue::readValue(&sbm.state->SADWindowSize);break;}
			case 2:{Menue::readValue(&sbm.state->preFilterSize);break;}
			//case 3:{readValue(&sbm.state->numberOfDisparities);break;}
			case 4:{Menue::readValue(&sbm.state->preFilterCap);break;}
			//case 5:{readValue(&sbm.state->minDisparity);break;}
			case 6:{Menue::readValue(&sbm.state->uniquenessRatio);break;}
			case 7:{Menue::readValue(&sbm.state->speckleWindowSize);break;}
			case 8:{Menue::readValue(&sbm.state->speckleRange);break;}
			case 9:{Menue::readValue(&sbm.state->disp12MaxDiff);break;}
			case 10:{Menue::readValue(&sbm.state->textureThreshold);break;}
			default : break;
		}
		
	}
}
Mat TwoPictureManipulation::reproject2(Mat img_disparity,Mat Q,Mat img_rgb)
{
	clock_t t;
	cout<<Q<<endl;
	double f=Q.at<double>(1,3);
	//Fix Q-Matrix
	Q.at<double>(3,2)=-1*Q.at<double>(3,2);//Baseline
	Q.at<double>(0,3)=1/2.4 *Q.at<double>(0,3);//camx
	Q.at<double>(1,3)=1/2 *Q.at<double>(1,3);//camy
	Q.at<double>(2,3)=1 *Q.at<double>(2,3);//fokus
	Q.at<double>(3,3)=(Q.at<double>(1,3)-Q.at<double>(0,3))*Q.at<double>(3,2);
	cout<<Q<<endl;
	//Berechnung der Punkte
	t = clock();
	cout << "Berechne Punkte" << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<MyPointType>::Ptr mycloud_ptr (new pcl::PointCloud<MyPointType>);
	Mat xyz;
	unsigned char pr, pg, pb;
	Size imageSize=Size(img_disparity.rows,img_disparity.cols);
	cout << ".....Berechne Punkte in OpenCV" << endl;
	//Punkte in OpenCV berechnen
	reprojectImageTo3D(img_disparity,xyz,Q);
	cout << ".....Konvertiere nach PCL" << endl;
	for(unsigned int pictureY=0;pictureY<xyz.rows-2;pictureY++)
	{
		unsigned char* rgb_ptr = img_rgb.ptr<unsigned char>(pictureY);
		for(unsigned int pictureX=0;pictureX<xyz.cols-2;pictureX++)
		{
			Point3f CVPoint = xyz.at<Point3f>(pictureY,pictureX)*16;

			//Koordianten
			pcl::PointXYZRGB point;
			MyPointType mypoint;
			point.x = CVPoint.x/1000;
			point.y = CVPoint.y/1000;
			point.z = CVPoint.z/1000;
			//Farben
			if ( !( point.x > 0 || point.x < 1 )) cout <<point.x;
			if ( !( point.y > 0 || point.y < 1 )) cout <<point.y;
			if ( !( point.z > 0 || point.z < 1 )) cout <<point.z;
			pb = rgb_ptr[3*pictureX];
			pg = rgb_ptr[3*pictureX+1];
			pr = rgb_ptr[3*pictureX+2];
			uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
				static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			//hinzufügen
			point_cloud_ptr->points.push_back(point);

			mypoint.x=point.x; mypoint.y=point.y; mypoint.z=point.z;
			mypoint.rgb=point.rgb;
			mypoint.u=pictureX; mypoint.v=pictureY;
			mypoint.vx=0; mypoint.vy=0; mypoint.vz=0;

			mycloud_ptr->points.push_back(mypoint);
		}
	}
	mycloud_ptr->width = xyz.cols-2;
	mycloud_ptr->height = xyz.rows-2;
	t = clock() - t;
	cout << "Berechnung der Punkte dauerte "<<((float)t)/CLOCKS_PER_SEC<<" Sekunden!"<<endl;
	model3d* myModel = new model3d();
	myModel->setUnfilteredCloud(mycloud_ptr);
	myModel->uiMain();
	return xyz;
}
void TwoPictureManipulation::Measurement(Mat img1,  Mat xyz, bool average=true)
{
	vector<int> bildpunkte;
	bildpunkte.push_back(0);bildpunkte.push_back(0);bildpunkte.push_back(0);bildpunkte.push_back(0);
	namedWindow("Klicken um Punkt aufzunehmen", 1);
	setMouseCallback("Klicken um Punkt aufzunehmen", CallBackFunc, &bildpunkte);
	imshow("Klicken um Punkt aufzunehmen", img1);
	waitKey(0);
	double tmp1;
	double median;
	vector <double> measuredDistances;
	cout<<"Suche Punkte"<<endl;
	if(average)
	{


		for(int j1=-5;j1<=5;++j1)
		{	for(int i1=-5;i1<=5;++i1)
			{	for(int j2=-5;j2<=5;++j2)
				{	for(int i2=-5;i2<=5;++i2)
					{
						int a1=(bildpunkte[0])+j1;
						int a2=(bildpunkte[1])+i1;
						int b1=(bildpunkte[2])+j2;
						int b2=(bildpunkte[3])+i2;
						if (a1<0)a1=0;if (a2<0)a2=0;if (b1<0)b1=0;if (b2<0)b2=0;

						Point3f Pkt1, Pkt2;
						Pkt1 =  xyz.at<Point3f>(a2,a1);
						Pkt2 =  xyz.at<Point3f>(b2,b1);
						tmp1=calcDistance(Pkt1,Pkt2);
						measuredDistances.push_back(tmp1);
		}	}	}	}

		
		size_t size = measuredDistances.size();

		sort(measuredDistances.begin(), measuredDistances.end());

		if (size  % 2 == 0)
		{
			median = (measuredDistances[size / 2 - 1] + measuredDistances[size / 2]) / 2;
		}
		else 
		{
			median = measuredDistances[size / 2];
		}
		
	}
	else
	{
		int a1=(bildpunkte[0]);
		int a2=(bildpunkte[1]);
		int b1=(bildpunkte[2]);
		int b2=(bildpunkte[3]);
		if (a1<0)a1=0;if (a2<0)a2=0;if (b1<0)b1=0;if (b2<0)b2=0;

		Point3f Pkt1, Pkt2;
		Pkt1 =  xyz.at<Point3f>(a2,a1);
		Pkt2 =  xyz.at<Point3f>(b2,b1);
		median=calcDistance(Pkt1,Pkt2);
	}
	cout<<median*16<<endl;//16 weil disparity value immer als vielfaches von 16 eingegeben wird
}
double calcDistance(Point3f p1, Point3f p2)
{
	double x,y,z;
	x=p1.x-p2.x;
	y=p1.y-p2.y;
	z=p1.z-p2.z;
	double d=sqrt(x*x+y*y+z*z);
	return d;
}

TwoPictureManipulation::TwoPictureManipulation(void)
{
}
TwoPictureManipulation::~TwoPictureManipulation(void)
{
}








//void RANSAC(pcl::PointCloud<PointType>::Ptr cloud_filtered){
//	cout<<"Objektsuche RANSAC"<<endl;
//
//	{
//		cout<<".....normale berechnen"<<endl;
//		pcl::NormalEstimation<PointType, pcl::Normal> ne;
//		pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
//		ne.setSearchMethod (tree);
//		ne.setInputCloud (cloud_filtered);
//		ne.setKSearch (50);
//		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//		ne.compute (*cloud_normals);
//
//		cout<<".....Ebene suchen"<<endl;
//		pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg; 
//		seg.setOptimizeCoefficients (true);
//		seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
//		seg.setNormalDistanceWeight (0.3);
//		seg.setMethodType (pcl::SAC_RANSAC);
//		seg.setMaxIterations (100);
//		seg.setDistanceThreshold (5);
//		seg.setInputCloud (cloud_filtered);
//		seg.setInputNormals (cloud_normals);
//		pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
//		pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
//		seg.segment (*inliers_plane, *coefficients_plane);
//		std::cerr <<"..........Plane coefficients: " << *coefficients_plane << std::endl;
//
//		cout<<".....Extrahiere Punkte"<<endl;
//		pcl::PointCloud<PointType>::Ptr cloud_filtered2 (new pcl::PointCloud<PointType>);
//		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
//		pcl::ExtractIndices<PointType> extract;
//		extract.setInputCloud (cloud_filtered);
//		extract.setIndices (inliers_plane);
//		extract.setNegative (false);
//		extract.setNegative (true);
//		extract.filter (*cloud_filtered2);
//
//		cout<<".....Extrahiere Normale"<<endl;
//		pcl::ExtractIndices<pcl::Normal> extract_normals;
//		extract_normals.setNegative (true);
//		extract_normals.setInputCloud (cloud_normals);
//		extract_normals.setIndices (inliers_plane);
//		extract_normals.filter (*cloud_normals2);
//
//		cout<<".....Zylinder suchen"<<endl;
//		seg.setOptimizeCoefficients (true);
//		seg.setModelType (pcl::SACMODEL_CYLINDER);
//		seg.setMethodType (pcl::SAC_RANSAC);
//		seg.setNormalDistanceWeight (0.1);
//		seg.setMaxIterations (10000);
//		seg.setDistanceThreshold (0.05);
//		seg.setRadiusLimits (0, 0.1);
//		seg.setInputCloud (cloud_filtered2);
//		seg.setInputNormals (cloud_normals2);
//		pcl::PointIndices::Ptr  inliers_cylinder (new pcl::PointIndices);
//		pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
//		seg.segment (*inliers_cylinder, *coefficients_cylinder);
//		std::cerr <<"..........Cylinder coefficients: " << *coefficients_cylinder << std::endl;
//
//		cout<<".....Ebene markieren"<<endl;
//		pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType> ());
//		extract.filter (*cloud_plane);
//		std::cerr <<"..........PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
//
//		cout<<".....Zylinder markieren"<<endl;
//		extract.setInputCloud (cloud_filtered2);
//		extract.setIndices (inliers_cylinder);
//		extract.setNegative (false);
//		pcl::PointCloud<PointType>::Ptr cloud_cylinder (new pcl::PointCloud<PointType> ());
//		extract.filter (*cloud_cylinder);
//		if (cloud_cylinder->points.empty ()) 
//		std::cerr <<"..........Can't find the cylindrical component." << std::endl;
//		else
//		std::cerr <<"..........PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
//	}

