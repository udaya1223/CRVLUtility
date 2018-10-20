/********************************************************
Computer & Robot Vision Lab, KNU
Programmed by: UKW
2013.11.04.
Last Update: 2018.08.06
********************************************************/

#include "opencv2/opencv.hpp" 
#include <iostream>
#include <string>

using namespace std;

typedef class crvlUtility{

public:
	crvlUtility(void);
	~crvlUtility();

	static void combineTwoSimilarImages(cv::Mat &first, cv::Mat &second, cv::Mat &combined);
	static void viewImage(string windowName, cv::Mat &image, int flag = 0);
	static void viewAndSaveImage(string windowName, cv::Mat &image, int flag = 0, string savePath = "");
	
	// Print openCV Mat
	static void crvlPrintMatrix(const char *pMessage, cv::Mat &mat);
		
	// Load calibration file (Matlab Calibration Toolbox) and make the intrinsic matrix
	static bool crvlParseIntrinsicCalibFile(cv::String pFilePath, cv::Mat &intrinsic, cv::Mat &distCoef);

	// Load calibration file (Matlab Calibration Toolbox) and make the extrinsic matrix
	static bool crvlParseExtrinsicCalibFile(char *pFilePath, cv::Mat &rotation, cv::Vec3d &translation);

	// Generate random colors
	static void crvlGenerateRandomPseudoColors(vector<cv::Scalar> &colors, int nMaxSize);

	// Find the plane that goes through several 3D points
	static void crvlFindPlaneEquations(vector<cv::Point3f> &laserPoints3D, cv::Vec4f &planeEquation);

	// Make a chessboard pattern given the num. of rows & cols, square size in pixels, and square colors 
	static void crvlMakeChessboardPattern(cv::Mat &outChessboard, int inRows, int inCols, int inSquareSize = 100, cv::Scalar color1 = CV_RGB(0, 0, 0), cv::Scalar color2 = CV_RGB(255, 255, 255)); 

	// Sort a float array in ascending order
	static void crvlQuickSort(std::vector<float> &src, int left, int right);

	// Sort a float array in ascending order
	static float findMode(vector<float> &src);
	
	static string ZeroPadNumber(int num, int pad);
	
	// Linear interpolation
	static bool linearInterpolation(double *inVals, double *outVals, int inCount, int outCount);
	
private:

} CRVL;
