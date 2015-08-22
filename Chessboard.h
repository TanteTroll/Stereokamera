#pragma once
#include <opencv2/core/core.hpp>
#include <string>
using namespace cv;
using namespace std;

class Chessboard
{
public:
	class Perspective
	{
	public:
		Chessboard *thisChessboard;
		Size imageSize;
		vector < vector< Point2f> > vvImagePoints;
		vector < vector< Point3f> > vvObjectPoints;
	
		Perspective(){};
		Perspective(Chessboard* einSchachbrett);
		bool addPicture(Mat pictureWithChessboard);
		bool addPictures(vector <Mat> vPicturesWithChessboard);
		vector < Mat > goodPictures;
		void deleteLastPicture();
		

	private:
		vector<Point2f> vChessboardPoints;
		vector< Point2f> vImagePoints;
	};
	float SquareSize;
	Size chessboardSize;

	vector< Point3f> vObjectPoints;
	
	void changeActivePerspective(int i);
	//void deletePerspective(int i);
	void addPerspective();
	
	void saveCurrentPerspective();
	int getNumberOfSavedPerspectived();
	//Chessboard(){};
	Chessboard(unsigned int NumberOfRows, unsigned int NumberOfCols, float SizeOfASquare = 1);
	~Chessboard(void);

	Perspective* currentPerspective;


	private:
	vector < Perspective > vPerspektiven;
};