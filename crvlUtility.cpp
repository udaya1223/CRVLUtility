/********************************************************
Computer & Robot Vision Lab, KNU
Programmed by: UKW
2013.11.04.
********************************************************/

#include "crvlUtility.h"

using namespace std;
using namespace cv;


// Print openCV Mat
void crvlUtility::crvlPrintMatrix(const char *pMessage, cv::Mat &mat)
{
	printf("%s\n", pMessage);

	for (int r = 0; r < mat.rows; r++) {
		for (int c = 0; c < mat.cols; c++) {

			switch (mat.depth())
			{
			case CV_8U:
			{
				printf("%*u ", 3, mat.at<uchar>(r, c));
				break;
			}
			case CV_8S:
			{
				printf("%*hhd ", 4, mat.at<schar>(r, c));
				break;
			}
			case CV_16U:
			{
				printf("%*hu ", 5, mat.at<ushort>(r, c));
				break;
			}
			case CV_16S:
			{
				printf("%*hd ", 6, mat.at<short>(r, c));
				break;
			}
			case CV_32S:
			{
				printf("%*d ", 6, mat.at<int>(r, c));
				break;
			}
			case CV_32F:
			{
				printf("%*.4f ", 10, mat.at<float>(r, c));
				break;
			}
			case CV_64F:
			{
				printf("%*.4f ", 10, mat.at<double>(r, c));
				break;
			}
			}
		} printf("\n");
	} printf("\n");
}

// Load calibration file (Matlab Calibration Toolbox) and make the intrinsic matrix
bool crvlUtility::crvlParseIntrinsicCalibFile(cv::String pFilePath, cv::Mat &intrinsic, cv::Mat &distCoef)
{
	FILE *fp = fopen(pFilePath.c_str(), "r+");

	if (!fp) {
		printf("Failed to open calibration file.\n");
		return false;
	}

	intrinsic = cv::Mat::eye(3, 3, CV_64F);
	distCoef = cv::Mat::zeros(1, 5, CV_64F);

	char szBuffer[512];

	while (!feof(fp)) {
		fscanf(fp, "%s", szBuffer);

		if (!strcmp("fc", szBuffer)) {
			fscanf(fp, "%s", szBuffer);
			fscanf(fp, "%s", szBuffer);

			// fc_x
			fscanf(fp, "%s", szBuffer);
			intrinsic.at<double>(0, 0) = atof(szBuffer);

			// fc_y
			fscanf(fp, "%s", szBuffer);
			intrinsic.at<double>(1, 1) = atof(szBuffer);
			continue;
		}
		else if (!strcmp("cc", szBuffer)) {
			fscanf(fp, "%s", szBuffer);
			fscanf(fp, "%s", szBuffer);

			// cc_x
			fscanf(fp, "%s", szBuffer);
			intrinsic.at<double>(0, 2) = atof(szBuffer);

			// cc_y
			fscanf(fp, "%s", szBuffer);
			intrinsic.at<double>(1, 2) = atof(szBuffer);

			continue;
		}
		else if (!strcmp("kc", szBuffer)) {
			fscanf(fp, "%s", szBuffer);
			fscanf(fp, "%s", szBuffer);

			// kc_1
			fscanf(fp, "%s", szBuffer);
			distCoef.at<double>(0, 0) = atof(szBuffer);

			// kc_2
			fscanf(fp, "%s", szBuffer);
			distCoef.at<double>(0, 1) = atof(szBuffer);

			// kc_3
			fscanf(fp, "%s", szBuffer);
			distCoef.at<double>(0, 2) = atof(szBuffer);

			// kc_4
			fscanf(fp, "%s", szBuffer);
			distCoef.at<double>(0, 3) = atof(szBuffer);

			// kc_5
			fscanf(fp, "%s", szBuffer);
			distCoef.at<double>(0, 4) = atof(szBuffer);

			continue;
		}
	}

	// Print details
	printf("\n");
	printf("-------------------------------------------------------------------------------\n");
	printf("Camera Calibration Info. : \n");
	crvlPrintMatrix("Intrinsic", intrinsic);
	crvlPrintMatrix("Dist-Coef.", distCoef);
	printf("-------------------------------------------------------------------------------\n");

	return true;
}

// Load calibration file (Matlab Calibration Toolbox) and make the extrinsic matrix
bool crvlUtility::crvlParseExtrinsicCalibFile(char *pFilePath, cv::Mat &rotation, cv::Vec3d &translation){

	FILE *fp = fopen(pFilePath, "r+");

	if (!fp) {
		printf("Failed to open calibration file.\n");
		return false;
	}

	rotation = cv::Mat::eye(3, 3, CV_64F);

	char szBuffer[512];

	while (!feof(fp)) {
		fscanf(fp, "%s", szBuffer);

		if (!strcmp("Tc_ext", szBuffer)) {
			fscanf(fp, "%s", szBuffer);
			fscanf(fp, "%s", szBuffer);

			// translation_x
			fscanf(fp, "%s", szBuffer);
			translation[0] = atof(szBuffer);

			// translation_y
			fscanf(fp, "%s", szBuffer);
			translation[1] = atof(szBuffer);

			// translation_z
			fscanf(fp, "%s", szBuffer);
			translation[2] = atof(szBuffer);

			continue;
		}
		else if (!strcmp("Rc_ext", szBuffer)) {
			fscanf(fp, "%s", szBuffer);
			fscanf(fp, "%s", szBuffer);

			// R_00
			fscanf(fp, "%s", szBuffer);
			rotation.at<double>(0, 0) = atof(szBuffer);

			// R_01
			fscanf(fp, "%s", szBuffer);
			rotation.at<double>(0, 1) = atof(szBuffer);

			// R_02
			fscanf(fp, "%s", szBuffer);
			rotation.at<double>(0, 2) = atof(szBuffer);

			// R_10
			fscanf(fp, "%s", szBuffer);
			rotation.at<double>(1, 0) = atof(szBuffer);

			// R_11
			fscanf(fp, "%s", szBuffer);
			rotation.at<double>(1, 1) = atof(szBuffer);

			// R_12
			fscanf(fp, "%s", szBuffer);
			rotation.at<double>(1, 2) = atof(szBuffer);

			// R_20
			fscanf(fp, "%s", szBuffer);
			rotation.at<double>(2, 0) = atof(szBuffer);

			// R_21
			fscanf(fp, "%s", szBuffer);
			rotation.at<double>(2, 1) = atof(szBuffer);

			// R_22
			fscanf(fp, "%s", szBuffer);
			rotation.at<double>(2, 2) = atof(szBuffer);

			continue;
		}

	}

	// Print details
	printf("\n");
	printf("-------------------------------------------------------------------------------\n");
	printf("Extrinsic Camera Info. : \n");
	crvlPrintMatrix("Rotation", rotation);
	printf("Translation: [%f %f %f]\n", translation[0], translation[1], translation[2]);
	printf("-------------------------------------------------------------------------------\n");

	return true;
}

// Generate random colors
void crvlUtility::crvlGenerateRandomPseudoColors(vector<cv::Scalar> &colors, int nMaxSize)
{
	colors.clear();

	for (int i = 0; i < nMaxSize; i++) {
		cv::RNG& rng = cv::theRNG();
		colors.push_back(cv::Scalar(rng(256), rng(256), rng(256)));
	}
}

// Find the plane that goes through several 3D points
void crvlUtility::crvlFindPlaneEquations(vector<cv::Point3f> &laserPoints3D, cv::Vec4f &planeEquation){

	cv::Vec3f cameraDir = cv::Vec3f(0.f, 0.f, 1.f);
	cv::Vec3f planeNormal;

	int nNumPoints = (int)laserPoints3D.size();

	if (nNumPoints < 4) {
		printf("ERROR: At least 4 3D points are required to find plane equation using SVD. \n");
		planeEquation = cv::Vec4f(0.f, 0.f, 0.f, 0.f);
		//system("pause");
	}
	else{
		cv::Mat A(nNumPoints, 4, CV_32F, cv::Scalar(0.)), W, U, Vt;
		for (int j = 0; j < nNumPoints; j++) {
			A.at<float>(j, 0) = laserPoints3D[j].x;
			A.at<float>(j, 1) = laserPoints3D[j].y;
			A.at<float>(j, 2) = laserPoints3D[j].z;
			A.at<float>(j, 3) = 1.;
		}

		// find SVD of A
		cv::SVD::compute(A, W, U, Vt);

		// Vt는 이미 정렬이 되어 있음!
		planeNormal[0] = Vt.at<float>(3, 0);
		planeNormal[1] = Vt.at<float>(3, 1);
		planeNormal[2] = Vt.at<float>(3, 2);

		float dLen = sqrt(planeNormal.dot(planeNormal));
		planeNormal /= dLen;

		// 카메라의 방향과 정반대라면.....
		if (cameraDir.dot(planeNormal) > 0.) {
			planeEquation = cv::Vec4f(-Vt.at<float>(3, 0), -Vt.at<float>(3, 1), -Vt.at<float>(3, 2), -Vt.at<float>(3, 3));
		}
		else {
			planeEquation = cv::Vec4f(Vt.at<float>(3, 0), Vt.at<float>(3, 1), Vt.at<float>(3, 2), Vt.at<float>(3, 3));
		}
	}

}

// Make a chessboard pattern given the num. of rows & cols, square size in pixels, and square colors 
void crvlUtility::crvlMakeChessboardPattern(cv::Mat &outChessboard, int inRows, int inCols, int inSquareSize, cv::Scalar color1, cv::Scalar color2){

	CV_Assert(inRows > 1 && inCols > 1);
	CV_Assert(inSquareSize > 0);

	int chessboardImgCols = inCols*inSquareSize;
	int chessboardImgRows = inRows*inSquareSize;

	outChessboard = cv::Mat(chessboardImgRows, chessboardImgCols, CV_8UC3, cv::Scalar(0, 0, 0));

	for (int rows = 0, rowNum = 0; rows < chessboardImgRows; rows += inSquareSize, rowNum++)
	{
		for (int cols = 0, colNum = 0; cols < chessboardImgCols; cols += inSquareSize, colNum++)
		{
			cv::Rect rec(cols, rows, inSquareSize, inSquareSize);

			if ((rowNum + colNum) % 2 == 0)
				rectangle(outChessboard, rec, color1, -1, 8);
			else
				rectangle(outChessboard, rec, color2, -1, 8);
		}
	}
}

// Sort a float array in ascending order  
void crvlUtility::crvlQuickSort(std::vector<float> &src, int left, int right) {

	if (src.size() > 1){

		int i = left, j = right;
		float tmp;
		double pivot = src[(left + right) / 2];

		/* partition */
		while (i <= j) {
			while (src[i] < pivot)
				i++;
			while (src[j] > pivot)
				j--;
			if (i <= j) {
				tmp = src[i];
				src[i] = src[j];
				src[j] = tmp;
				i++;
				j--;
			}
		};

		/* recursion */
		if (left < j)
			crvlQuickSort(src, left, j);
		if (i < right)
			crvlQuickSort(src, i, right);
	}
}

// Find the mode of a float array  
float crvlUtility::findMode(vector<float> &src){
	float number = src[src.size() - 1];
	float mode = number;
	int count = 1;
	int countMode = 1;

	for (int i = src.size() - 2; i >= 0; i--)
	{
		if (src[i] == number)
		{
			count++;
		}
		else
		{
			if (count > countMode)
			{
				countMode = count;
				mode = number;
			}
			count = 1;
			number = src[i];
		}
	}
	//printf("Mode : %f\n", mode);
	return mode;
}

// Zero Padding. Add zeros in front of a number  
string crvlUtility::ZeroPadNumber(int num, int pad)
	{
		stringstream ss;

		// the number is converted to string with the help of stringstream
		ss << num;
		string ret;
		ss >> ret;

		if (pad < 1){
			return ret;
		}
		else{
			// Append zero chars
			int str_length = ret.length();
			for (int i = 0; i < pad - str_length; i++)
				ret = "0" + ret;
			return ret;
		}
	}

// Linear interpolation
bool crvlUtility::linearInterpolation(double *inVals, double *outVals, int inCount, int outCount)
{
	if (inCount > outCount)
		return false;

	double mult = inCount / (double)outCount;
	int j = 0;
	double i = 0;
	for (j = 0, i = 0.; i <= inCount; i += mult, j++)
	{
		int k = (int)floor(i);
		double a = i - k;
		double y = a * inVals[k + 1] + (-inVals[k] * a + inVals[k]);
		outVals[j] = y;
		//printf("%f %f\n", i, y);
	}
	return true;
}
