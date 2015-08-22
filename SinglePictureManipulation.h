#pragma once
#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"

using namespace cv;
using namespace std;



class SinglePictureManipulation
{
public:
	SinglePictureManipulation(void);
	~SinglePictureManipulation(void);
	//void undistort(InputArray src, OutputArray dst, InputArray cameraMatrix, InputArray distCoeffs, InputArray newCameraMatrix=noArray() );
/*src – Input (distorted) image.
dst – Output (corrected) image that has the same size and type as src .
cameraMatrix – Input camera matrix  A = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1} .
distCoeffs – Input vector of distortion coefficients  (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
newCameraMatrix – Camera matrix of the distorted image. By default, it is the same as cameraMatrix but you may additionally scale and shift the result by using a different matrix.*/

	static vector<KeyPoint> getFeatures(Mat image);
	
	static vector < vector <KeyPoint> > getFeatures (vector <Mat> images);
	static Mat getFeaturesAsMat(Mat image);
	static vector<Mat> getFeaturesAsMat(vector<Mat> vimages);
	static void get3dFromPicture();
};

