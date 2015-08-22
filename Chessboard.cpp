#include "stdafx.h"
#include "Chessboard.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv\cv.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "Settings.h"
using namespace cv;
using namespace std;

#pragma region Chessboard
Chessboard::Chessboard(unsigned int NumberOfRows, unsigned int NumberOfCols, float SizeOfASquare)
{
	chessboardSize.height=NumberOfRows;
	chessboardSize.width=NumberOfCols;
	SquareSize = SizeOfASquare;

	//ObjectPoints: Objekt(Schachbrett) im KS des Objektes
	for(unsigned int j=0; j< NumberOfRows * NumberOfCols; ++j)  
	{
		Point3f ObjPT;
		ObjPT.x = j%chessboardSize.width*SquareSize;	
		ObjPT.y = j/chessboardSize.width*SquareSize;	
		ObjPT.z = 0.0f;									//Schachbrett ist Flach
		vObjectPoints.push_back(ObjPT);					//im Vektor speichern
	}
	currentPerspective= new Perspective(this);
	
}
Chessboard::~Chessboard(void)
{
	//TODO destroy all elements in vPerspectiveList
}

void Chessboard::changeActivePerspective(int i)
{
	//Eingabe prüfen
	if (i>vPerspektiven.size()	|| i < 0)
	{
		cout<<"Element nicht vorhanden"<<endl;
		return;
	}
	//alten Eintrag speichern
	saveCurrentPerspective();
	//Neuen Eintrag kopieren
	currentPerspective=new Perspective;
	*currentPerspective=vPerspektiven[i];
}
void Chessboard::saveCurrentPerspective()
{
	vPerspektiven.push_back(*currentPerspective);
	delete currentPerspective;
}
void Chessboard::addPerspective()
{
	saveCurrentPerspective();
	currentPerspective=new Perspective(this);
}
int Chessboard::getNumberOfSavedPerspectived()
{
	return vPerspektiven.size();
}
#pragma endregion
#pragma region Perspective
Chessboard::Perspective::Perspective(Chessboard* einSchachbrett)
{
	thisChessboard = einSchachbrett;
}

bool Chessboard::Perspective::addPictures(vector <Mat> vPicturesWithChessboard)
{
	for(unsigned int j=0; j< vPicturesWithChessboard.size(); ++j) 
	{
		if (!addPicture ( vPicturesWithChessboard[j] ));
	}
	return true;
}
bool Chessboard::Perspective::addPicture(Mat pictureWithChessboard)
{
	vImagePoints.clear();

	imageSize = Size(pictureWithChessboard.cols, pictureWithChessboard.rows);
	//Ecken näherungsweise finden
	bool patternfound;
	patternfound = findChessboardCorners(pictureWithChessboard, thisChessboard->chessboardSize, vChessboardPoints,
		CALIB_CB_NORMALIZE_IMAGE|CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS|CALIB_CB_FAST_CHECK);
	if (!patternfound)
	{
		cout<<"Kein Schachbrett gefunden. Bild nicht aufgenommen!"<<endl;
		return 0;
	}

	//In graues Bild umwandeln
	Mat grayImage;
	if (pictureWithChessboard.channels() != 1)
	{
		cvtColor(pictureWithChessboard , grayImage , CV_BGR2GRAY);
	}
	else
	{
		grayImage=pictureWithChessboard;
	}

	//Ecken genau finden
	
	cout << "Schachbrett gefunden. Suche Subpix" << endl;

	cornerSubPix(grayImage, vChessboardPoints, Size(11, 11), Size(-1, -1), 
			TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	
	/*Mat pictureWithChessboardDrawen;
	pictureWithChessboard.copyTo(pictureWithChessboardDrawen);
	drawChessboardCorners(pictureWithChessboardDrawen,thisChessboard->chessboardSize,vChessboardPoints,patternfound);
	
	if (Settings::globalDebugshowPictures == 1)
	{
		namedWindow( "FoundChessboard", WINDOW_AUTOSIZE );
		imshow( "FoundChessboard", pictureWithChessboardDrawen);
		waitKey(0);
	}*/

	//Ergebnisse abspeichern

	goodPictures.push_back(pictureWithChessboard);
	vvObjectPoints.push_back(thisChessboard->vObjectPoints);
	vvImagePoints.push_back(vChessboardPoints);
	vImagePoints.clear();
	vChessboardPoints.clear();
	return 1;
}
void Chessboard::Perspective::deleteLastPicture()
{
	if(goodPictures.size()>0)
	{
		goodPictures.pop_back();
		vvObjectPoints.pop_back();
		vvImagePoints.pop_back();
	}
	else
	{
		cout<<"Kein Element zum loeschen"<<endl;
	}
}

#pragma endregion