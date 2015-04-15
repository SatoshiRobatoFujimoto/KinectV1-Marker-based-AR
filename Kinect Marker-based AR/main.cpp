#include <Windows.h>
#include <NuiApi.h>
#include <opencv2/opencv.hpp>
#include <opencv_lib.hpp>
#include <sys/timeb.h>
#include <iostream>
#include <fstream>
#include <direct.h>
#include <string.h>


#pragma comment(lib, "Kinect10.lib")

using namespace cv;
using namespace std;


int mat2id(const cv::Mat &bits)
{
	int val = 0;
	for (int y = 0; y<5; y++)
	{
		val <<= 1;
		if (bits.at<uchar>(y, 1)) val |= 1;
		val <<= 1;
		if (bits.at<uchar>(y, 3)) val |= 1;
	}
	return val;
}

cv::Mat rotate(cv::Mat in)
{
	cv::Mat out;
	in.copyTo(out);
	for (int i = 0; i<in.rows; i++)
	{
		for (int j = 0; j<in.cols; j++)
		{
			out.at<uchar>(i, j) = in.at<uchar>(in.cols - j - 1, i);
		}
	}
	return out;
}


int hammDistMarker(cv::Mat bits)
{
	int ids[4][5] =
	{
		{ 1, 0, 0, 0, 0 },
		{ 1, 0, 1, 1, 1 },
		{ 0, 1, 0, 0, 1 },
		{ 0, 1, 1, 1, 0 }
	};

	int dist = 0;

	for (int y = 0; y<5; y++)
	{
		int minSum = 1e5; //hamming distance to each possible word

		for (int p = 0; p<4; p++)
		{
			int sum = 0;
			//now, count
			for (int x = 0; x<5; x++)
			{
				sum += bits.at<uchar>(y, x) == ids[p][x] ? 0 : 1;
			}

			if (minSum>sum)
				minSum = sum;
		}

		//do the and
		dist += minSum;
	}

	return dist;
}

float perimeter(const std::vector<cv::Point2f> &a)
{
	float sum = 0, dx, dy;

	for (size_t i = 0; i<a.size(); i++)
	{
		size_t i2 = (i + 1) % a.size();

		dx = a[i].x - a[i2].x;
		dy = a[i].y - a[i2].y;

		sum += sqrt(dx*dx + dy*dy);
	}

	return sum;
}

int main()
{

	//ファイル入出力
	ofstream ofs_rgb, ofs_depth;
	int fileNum = 0;
	int count = 0;

	cout << "フォルダの削除" << endl;
	system("rmdir /s /q ..\\data");
	cout << "新しいフォルダを作成中" << endl;
	waitKey(10);
	_mkdir("..\\data");
	_mkdir("..\\data\\rgb");
	_mkdir("..\\data\\depth");
	cout << "完了" << endl;
	waitKey(10);

	cout << "qキーかESCで終了" << endl;
	//cout << "sキーで保存" << endl;

	ofs_depth.open("..\\data\\depth.txt");
	ofs_rgb.open("..\\data\\rgb.txt");
	if (!ofs_depth || !ofs_rgb){
		cout << "file open error\n";
		exit(1);
	}

	ofs_depth << "# depth maps\n# file: 'data'\n# timestamp filename" << endl;
	ofs_rgb << "# rgb maps  \n# file: 'data'\n# timestamp filename" << endl;

	ofstream ofs_dist;
	ofs_dist.open("..\\data\\dist.txt");
	if (!ofs_dist){
		cout << "file open error\n";
		exit(1);
	}


	cv::setUseOptimized(true);

	// Kinectのインスタンス生成、初期化
	INuiSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = NuiCreateSensorByIndex(0, &pSensor);
	if (FAILED(hResult)){
		std::cerr << "Error : NuiCreateSensorByIndex" << std::endl;
		return -1;
	}

	hResult = pSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX);
	if (FAILED(hResult)){
		std::cerr << "Error : NuiInitialize" << std::endl;
		return -1;
	}

	// Colorストリーム
	HANDLE hColorEvent = INVALID_HANDLE_VALUE;
	HANDLE hColorHandle = INVALID_HANDLE_VALUE;
	hColorEvent = CreateEvent(nullptr, true, false, nullptr);
	hResult = pSensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, hColorEvent, &hColorHandle);
	if (FAILED(hResult)){
		std::cerr << "Error : NuiImageStreamOpen( COLOR )" << std::endl;
		return -1;
	}

	// Depth&Playerストリーム
	HANDLE hDepthPlayerEvent = INVALID_HANDLE_VALUE;
	HANDLE hDepthPlayerHandle = INVALID_HANDLE_VALUE;
	hDepthPlayerEvent = CreateEvent(nullptr, true, false, nullptr);
	hResult = pSensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_640x480, 0, 2, hDepthPlayerEvent, &hDepthPlayerHandle);
	if (FAILED(hResult)){
		std::cerr << "Error : NuiImageStreamOpen( DEPTH&PLAYER )" << std::endl;
		return -1;
	}

	/*
	// Near Modeの設定
	hResult = pSensor->NuiImageStreamSetImageFrameFlags( hDepthPlayerHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE );
	if( FAILED( hResult ) ){
	std::cerr << "Error : NuiImageStreamSetImageFrameFlags" << std::endl;
	return -1;
	}
	*/

	// 解像度の取得
	unsigned long refWidth = 0;
	unsigned long refHeight = 0;
	NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_640x480, refWidth, refHeight);
	int width = static_cast<int>(refWidth);
	int height = static_cast<int>(refHeight);

	// 位置合わせの設定
	INuiCoordinateMapper* pCordinateMapper;
	hResult = pSensor->NuiGetCoordinateMapper(&pCordinateMapper);
	if (FAILED(hResult)){
		std::cerr << "Error : NuiGetCoordinateMapper" << std::endl;
		return -1;
	}
	std::vector<NUI_COLOR_IMAGE_POINT> pColorPoint(width * height);

	HANDLE hEvents[2] = { hColorEvent, hDepthPlayerEvent };

	// カラーテーブル
	cv::Vec3b color[7];
	color[0] = cv::Vec3b(0, 0, 0);
	color[1] = cv::Vec3b(255, 0, 0);
	color[2] = cv::Vec3b(0, 255, 0);
	color[3] = cv::Vec3b(0, 0, 255);
	color[4] = cv::Vec3b(255, 255, 0);
	color[5] = cv::Vec3b(255, 0, 255);
	color[6] = cv::Vec3b(0, 255, 255);

	cv::namedWindow("Color");
	cv::namedWindow("Depth");
	//cv::namedWindow("Player");

	LONG target_angle = 0;
	LONG angle = 0;
	//Tiltの角度読み込み
	pSensor->NuiCameraElevationGetAngle(&angle);
	pSensor->NuiCameraElevationSetAngle(target_angle);

	while (1){
		// フレームの更新待ち
		ResetEvent(hColorEvent);
		ResetEvent(hDepthPlayerEvent);
		WaitForMultipleObjects(ARRAYSIZE(hEvents), hEvents, true, INFINITE);

		// Colorカメラからフレームを取得
		NUI_IMAGE_FRAME colorImageFrame = { 0 };
		hResult = pSensor->NuiImageStreamGetNextFrame(hColorHandle, 0, &colorImageFrame);
		if (FAILED(hResult)){
			std::cerr << "Error : NuiImageStreamGetNextFrame( COLOR )" << std::endl;
			return -1;
		}

		// Color画像データの取得
		INuiFrameTexture* pColorFrameTexture = colorImageFrame.pFrameTexture;
		NUI_LOCKED_RECT colorLockedRect;
		pColorFrameTexture->LockRect(0, &colorLockedRect, nullptr, 0);

		// Depthセンサーからフレームを取得
		NUI_IMAGE_FRAME depthPlayerImageFrame = { 0 };
		hResult = pSensor->NuiImageStreamGetNextFrame(hDepthPlayerHandle, 0, &depthPlayerImageFrame);
		if (FAILED(hResult)){
			std::cerr << "Error : NuiImageStreamGetNextFrame( DEPTH&PLAYER )" << std::endl;
			return -1;
		}

		// Depth&Playerデータの取得
		BOOL nearMode = false;
		INuiFrameTexture* pDepthPlayerFrameTexture = nullptr;
		pSensor->NuiImageFrameGetDepthImagePixelFrameTexture(hDepthPlayerHandle, &depthPlayerImageFrame, &nearMode, &pDepthPlayerFrameTexture);
		NUI_LOCKED_RECT depthPlayerLockedRect;
		pDepthPlayerFrameTexture->LockRect(0, &depthPlayerLockedRect, nullptr, 0);

		// 表示
		cv::Mat colorMat(height, width, CV_8UC4, reinterpret_cast<unsigned char*>(colorLockedRect.pBits));

		cv::Mat bufferMat = cv::Mat::zeros(height, width, CV_16UC1);
		//cv::Mat playerMat = cv::Mat::zeros(height, width, CV_8UC3);
		NUI_DEPTH_IMAGE_PIXEL* pDepthPlayerPixel = reinterpret_cast<NUI_DEPTH_IMAGE_PIXEL*>(depthPlayerLockedRect.pBits);
		pCordinateMapper->MapDepthFrameToColorFrame(NUI_IMAGE_RESOLUTION_640x480, width * height, pDepthPlayerPixel, NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, width * height, &pColorPoint[0]);
		for (int y = 0; y < height; y++){
			for (int x = 0; x < width; x++){
				unsigned int index = y * width + x;
				bufferMat.at<unsigned short>(pColorPoint[index].y, pColorPoint[index].x) = pDepthPlayerPixel[index].depth;
				//playerMat.at<cv::Vec3b>(pColorPoint[index].y, pColorPoint[index].x) = color[pDepthPlayerPixel[index].playerIndex];
			}
		}

		cv::Mat depthMat(height, width, CV_8UC1);
		bufferMat.convertTo(depthMat, CV_8U, -255.0f / 10000.0f, 255.0f);

		//追加
		cv::Mat depthMatC4(height, width, CV_8UC4);
		cv::cvtColor(depthMat, depthMatC4, CV_GRAY2BGRA);
		cv::Mat rgbdMat(height, width, CV_8UC4);
		rgbdMat = colorMat * 0.5 + depthMatC4 * 0.5;

		// 1. grayscale
		cv::Mat binMat, grayMat;
		cv::cvtColor(colorMat, grayMat, CV_BGRA2GRAY);
		cv::imshow("Gray", grayMat);

		// 2. binaryzation
		cv::threshold(grayMat, binMat, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

		cv::Mat binMat_;//表示用
		cv::cvtColor(binMat, binMat_, CV_GRAY2BGR);

		// 3. find contours
		std::vector<std::vector<cv::Point>> allContours;
		std::vector<Vec4i> hierarchy;
		cv::findContours(binMat, allContours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

		std::vector<std::vector<cv::Point>> contours;
		contours.clear();
		int minContourPointsAllowed = 10;
		for (size_t i = 0; i < allContours.size(); i++){
			int contourSize = allContours[i].size();
			if (contourSize > minContourPointsAllowed){
				contours.push_back(allContours[i]);
			}
		}

		// 4. detect possible marker
		std::vector<std::vector<cv::Point2f>> possibleMarkersPoints;
		std::vector<cv::Point> approxCurve;

		// For each contour, analyze if it is a parallelepiped likely to be the marker
		for (size_t i = 0; i < contours.size(); i++){
			// Approximate to a polygon
			double eps = contours[i].size() * 0.05;
			cv::approxPolyDP(contours[i], approxCurve, eps, true);

			// We interested only in polygons that contains only four points
			if (approxCurve.size() != 4)
				continue;

			// And they have to be convex
			if (!cv::isContourConvex(approxCurve))
				continue;

			// Ensure that the distance between consecutive points is large enough
			float minDist = std::numeric_limits<float>::max();

			for (int i = 0; i < 4; i++){
				cv::Point side = approxCurve[i] - approxCurve[(i + 1) % 4];
				float squaredSideLength = side.dot(side);
				minDist = std::min(minDist, squaredSideLength);
			}

			// Check that distance is not very small
			int m_minContourLengthAllowed = 100;
			if (minDist < m_minContourLengthAllowed)
				continue;

			// All tests are passed. Save marker candidate:
			std::vector<cv::Point2f> mpoints;
			for (int i = 0; i < 4; i++){
				mpoints.push_back(cv::Point2f(approxCurve[i].x, approxCurve[i].y));
			}

			// Sort the points in anti-clockwise order 
			// Trace a line between the first and second point.
			// If the third point is at the right side, then the points are anti-clockwise
			cv::Point v1 = mpoints[1] - mpoints[0];
			cv::Point v2 = mpoints[2] - mpoints[0];
			double o = (v1.x * v2.y) - (v1.y * v2.x);

			if (o < 0.0) // if the third point is in the left side, then
				std::swap(mpoints[1], mpoints[3]);

			possibleMarkersPoints.push_back(mpoints);
		}

		// Remove these elements which corners are too close to each other.
		// First detect candidates for removal
		std::vector<std::pair<int, int>> tooNearCandidates;
		for (size_t i = 0; i < possibleMarkersPoints.size(); i++){
			const std::vector<cv::Point2f> mpoints1 = possibleMarkersPoints[i];

			//calculate the average distance of each corner to the nearest corner of the other marker candidate
			for (size_t j = i + 1; j < possibleMarkersPoints.size(); j++){
				const std::vector<cv::Point2f> mpoints2 = possibleMarkersPoints[j];

				float distSquared = 0;
				for (int c = 0; c < 4; c++){
					cv::Point v = mpoints1[c] - mpoints2[c];
					distSquared += v.dot(v);
				}

				distSquared /= 4;

				if (distSquared < 100){
					tooNearCandidates.push_back(std::pair<int, int>(i, j));
				}
			}
		}

		// Mark for removal the element of the pair with smaller perimeter
		std::vector<bool> removalMask(possibleMarkersPoints.size(), false);

		for (size_t i = 0; i < tooNearCandidates.size(); i++){
			float p1 = perimeter(possibleMarkersPoints[tooNearCandidates[i].first]);
			float p2 = perimeter(possibleMarkersPoints[tooNearCandidates[i].second]);

			size_t removalIndex;
			if (p1 > p2)
				removalIndex = tooNearCandidates[i].second;
			else
				removalIndex = tooNearCandidates[i].first;

			removalMask[removalIndex] = true;
		}

		// Return candidates
		std::vector<std::vector<cv::Point2f>> detectedMarkersPoints;
		detectedMarkersPoints.clear();
		for (size_t i = 0; i < possibleMarkersPoints.size(); i++){
			if (!removalMask[i]){
				detectedMarkersPoints.push_back(possibleMarkersPoints[i]);
				std::vector<cv::Point2f> mpoints = possibleMarkersPoints[i];
			}
		}

		// 5. decode marker and recognize
		std::vector<std::vector<cv::Point2f>> goodMarkersPoints;
		goodMarkersPoints.clear();
		//Find the perspective transformation that brings current marker to rectangular form
		std::vector<cv::Point2f> markerCorners2d;
		cv::Size markerSize = cv::Size(100, 100);

		markerCorners2d.push_back(cv::Point2f(0, 0));
		markerCorners2d.push_back(cv::Point2f(markerSize.width - 1, 0));
		markerCorners2d.push_back(cv::Point2f(markerSize.width - 1, markerSize.height - 1));
		markerCorners2d.push_back(cv::Point2f(0, markerSize.height - 1));

		for (size_t i = 0; i < detectedMarkersPoints.size(); i++){
			cv::Mat canonicalMarkerMat;
			std::vector<cv::Point2f> markersPoints = detectedMarkersPoints[i];

			cv::Mat M = cv::getPerspectiveTransform(markersPoints, markerCorners2d);

			// Transform image to get a canonical marker image
			cv::warpPerspective(grayMat, canonicalMarkerMat, M, markerSize);

			cv::threshold(canonicalMarkerMat, canonicalMarkerMat, 125, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
			cv::imshow("canonicalMarker", canonicalMarkerMat);
			//cv::waitKey(0);

			//Markers  are divided in 7x7 regions, of which the inner 5x5 belongs to marker info
			//the external border should be entirely black

			int cellSize = canonicalMarkerMat.rows / 7;

			for (int y = 0; y<7; y++)
			{
				int inc = 6;

				if (y == 0 || y == 6) inc = 1; //for first and last row, check the whole border

				for (int x = 0; x<7; x += inc)
				{
					int cellX = x * cellSize;
					int cellY = y * cellSize;
					cv::Mat cell = canonicalMarkerMat(cv::Rect(cellX, cellY, cellSize, cellSize));

					int nZ = cv::countNonZero(cell);

					if (nZ >(cellSize*cellSize) / 2)
					{
						continue;//can not be a marker because the border element is not black!
					}
				}
			}

			cv::Mat bitMatrix = cv::Mat::zeros(5, 5, CV_8UC1);

			//get information(for each inner square, determine if it is  black or white)  
			for (int y = 0; y<5; y++)
			{
				for (int x = 0; x<5; x++)
				{
					int cellX = (x + 1)*cellSize;
					int cellY = (y + 1)*cellSize;
					cv::Mat cell = canonicalMarkerMat(cv::Rect(cellX, cellY, cellSize, cellSize));

					int nZ = cv::countNonZero(cell);
					if (nZ>(cellSize*cellSize) / 2)
						bitMatrix.at<uchar>(y, x) = 1;
				}
			}

			//check all possible rotations
			cv::Mat rotations[4];
			int distances[4];

			rotations[0] = bitMatrix;
			distances[0] = hammDistMarker(rotations[0]);

			std::pair<int, int> minDist(distances[0], 0);

			for (int i = 1; i<4; i++)
			{
				//get the hamming distance to the nearest possible word
				rotations[i] = rotate(rotations[i - 1]);
				distances[i] = hammDistMarker(rotations[i]);

				if (distances[i] < minDist.first)
				{
					minDist.first = distances[i];
					minDist.second = i;
				}
			}
			int nRotations = minDist.second;
			int id;
			if (minDist.first == 0)
			{
				id = mat2id(rotations[minDist.second]);
			}
			else{
				id = -1;
			}

			if (id != -1)
			{
				//sort the points so that they are always in the same order 
				// no matter the camera orientation
				std::rotate(markersPoints.begin(), markersPoints.begin() + 4 - nRotations, markersPoints.end());

				goodMarkersPoints.push_back(markersPoints);
			}
		}
		if (goodMarkersPoints.size() > 0)
		{
			std::vector<cv::Point2f> preciseCorners(4 * goodMarkersPoints.size());

			for (size_t i = 0; i < goodMarkersPoints.size(); i++){
				std::vector<cv::Point2f> markersPoints = goodMarkersPoints[i];
				for (int c = 0; c < 4; c++){
					preciseCorners[i * 4 + c] = markersPoints[c];
				}
			}
			cv::cornerSubPix(grayMat, preciseCorners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER, 30, 0.1));

			//copy back
			for (size_t i = 0; i < goodMarkersPoints.size(); i++){
				std::vector<cv::Point2f> markersPoints = goodMarkersPoints[i];

				for (int c = 0; c < 4; c++){
					markersPoints[c] = preciseCorners[i * 4 + c];
				}

				int thickness = 2;
				cv::line(binMat_, markersPoints[0], markersPoints[1], cv::Scalar(255, 0, 0), thickness, CV_AA);
				cv::line(binMat_, markersPoints[1], markersPoints[2], cv::Scalar(255, 0, 0), thickness, CV_AA);
				cv::line(binMat_, markersPoints[2], markersPoints[3], cv::Scalar(255, 0, 0), thickness, CV_AA);
				cv::line(binMat_, markersPoints[3], markersPoints[0], cv::Scalar(255, 0, 0), thickness, CV_AA);
			}

			// 6. Estimate marker 3D pose
			cv::Mat camMatrix(3, 3, CV_32F);
			cv::Mat distCoeff(4, 1, CV_32F);

			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					camMatrix.at<float>(i, j) = 0;

			camMatrix.at<float>(0, 0) = 525.0; // fx
			camMatrix.at<float>(1, 1) = 525.0; // fy
			camMatrix.at<float>(0, 2) = 319.5; // cx
			camMatrix.at<float>(1, 2) = 239.5; // cy
			camMatrix.at<float>(2, 2) = 1.0;

			for (int i = 0; i < 4; i++)
				distCoeff.at<float>(i, 0) = 0;

			std::vector<cv::Point3f> markerCorners3d;
			bool centerOrigin = true;
			if (centerOrigin)
			{
				markerCorners3d.push_back(cv::Point3f(-0.45f, -0.45f, 0));
				markerCorners3d.push_back(cv::Point3f(+0.45f, -0.45f, 0));
				markerCorners3d.push_back(cv::Point3f(+0.45f, +0.45f, 0));
				markerCorners3d.push_back(cv::Point3f(-0.45f, +0.45f, 0));
			}
			else
			{
				markerCorners3d.push_back(cv::Point3f(0, 0, 0));
				markerCorners3d.push_back(cv::Point3f(1, 0, 0));
				markerCorners3d.push_back(cv::Point3f(1, 1, 0));
				markerCorners3d.push_back(cv::Point3f(0, 1, 0));
			}

			for (size_t i = 0; i < goodMarkersPoints.size(); i++)
			{
				std::vector<cv::Point2f> mpoints = goodMarkersPoints[i];

				cv::Mat Rvec;
				cv::Mat_<float> Tvec;
				cv::Mat raux, taux;
				cv::solvePnP(markerCorners3d, mpoints, camMatrix, distCoeff, raux, taux);
				raux.convertTo(Rvec, CV_32F);
				taux.convertTo(Tvec, CV_32F);

				cv::Mat_<float> rotMat(3, 3);
				cv::Rodrigues(Rvec, rotMat);

				// Copy to transformation matrix
				//for (int col = 0; col<3; col++)
				//{
				//	for (int row = 0; row<3; row++)
				//	{
				//		m.transformation.r().mat[row][col] = rotMat(row, col); // Copy rotation component
				//	}
				//	m.transformation.t().data[col] = Tvec(col); // Copy translation component
				//}

				// drgb
				char str[100];
				sprintf(str, "%.3lf mm", Tvec(2) * 100);//10cm → 1
				cv::putText(binMat_, str, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(200, 0, 0), 2, CV_AA);

				cv::Mat transformMat(3, 4, CV_32F);
				transformMat.at<float>(0, 0) = rotMat(0, 0);
				transformMat.at<float>(0, 1) = rotMat(0, 1);
				transformMat.at<float>(0, 2) = rotMat(0, 2);
				transformMat.at<float>(1, 0) = rotMat(1, 0);
				transformMat.at<float>(1, 1) = rotMat(1, 1);
				transformMat.at<float>(1, 2) = rotMat(1, 2);
				transformMat.at<float>(2, 0) = rotMat(2, 0);
				transformMat.at<float>(2, 1) = rotMat(2, 1);
				transformMat.at<float>(2, 2) = rotMat(2, 2);
				transformMat.at<float>(0, 3) = Tvec(0);
				transformMat.at<float>(1, 3) = Tvec(1);
				transformMat.at<float>(2, 3) = Tvec(2);
				std::cout << transformMat << std::endl;

				cv::Mat centerMarkerObject = cv::Mat::zeros(4, 1, CV_32F);
				centerMarkerObject.at<float>(3, 0) = 1.0;
				//std::cout << centerMarkerObject << std::endl;
				//std::cout << camMatrix << std::endl;

				cv::Mat centerMarker(3, 1, CV_32F);
				centerMarker = camMatrix*transformMat*centerMarkerObject;
				float centerX, centerY;
				centerX = centerMarker.at<float>(0, 0) / centerMarker.at<float>(2, 0);
				centerY = centerMarker.at<float>(1, 0) / centerMarker.at<float>(2, 0);
				//std::cout << centerX << " " << centerY << std::endl;

				// dd
				unsigned int index = ((unsigned int)centerY * (unsigned int)width + (unsigned int)centerX);
				unsigned short Z = bufferMat.at<unsigned short>(pColorPoint[index].y, pColorPoint[index].x);

				ofs_dist << Tvec(2) * 100 << " " << Z << std::endl;

				cv::line(binMat_, cv::Point2f(centerX - 10, centerY), cv::Point2f(centerX + 10, centerY), cv::Scalar(0, 0, 200));
				cv::line(binMat_, cv::Point2f(centerX, centerY - 10), cv::Point2f(centerX, centerY + 10), cv::Scalar(0, 0, 200));

				sprintf(str, "%u mm", Z);
				cv::putText(binMat_, str, cv::Point(350, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 200), 2, CV_AA);

				// Since solvePnP finds camera location, w.r.t to marker pose, to get marker pose w.r.t to the camera we invert it.
				//m.transformation = m.transformation.getInverted();
			}
		}

		cv::imshow("Color", colorMat);
		cv::imshow("Depth", depthMat);
		//cv::imshow("Player", playerMat);
		cv::imshow("RGBD", rgbdMat);
		cv::imshow("Color Bin", binMat_);

		// フレームの解放
		pColorFrameTexture->UnlockRect(0);
		pDepthPlayerFrameTexture->UnlockRect(0);
		pSensor->NuiImageStreamReleaseFrame(hColorHandle, &colorImageFrame);
		pSensor->NuiImageStreamReleaseFrame(hDepthPlayerHandle, &depthPlayerImageFrame);

		// ループの終了判定(Escキー)
		int key = cv::waitKey(30);
		if (key == VK_ESCAPE || key == 'q'){
			system("explorer ..\\data");
			break;
		}
		else if (key == 's'){
			//Kinectデータ保存
			//ここのタイムスタンプを変更する必要がある。
			double epoch_time;
			struct timeb tv_;
			ftime(&tv_);
			epoch_time = (double)tv_.time;
			epoch_time += (double)tv_.millitm / 1000.0;
			char fileName[100];
			sprintf_s(fileName, "..\\data\\rgb\\%016.6lf.png", (double)epoch_time);
			cv::Mat colorMatC3(height, width, CV_8UC3);
			cv::cvtColor(colorMat, colorMatC3, CV_RGBA2RGB);
			imwrite(fileName, colorMatC3);
			sprintf_s(fileName, "%016.6lf", (double)epoch_time);
			ofs_rgb << fileName << " rgb/" << fileName << ".png" << endl;

			sprintf_s(fileName, "..\\data\\depth\\%016.6lf.png", (double)epoch_time);
			imwrite(fileName, bufferMat);
			sprintf_s(fileName, "%016.6lf", (double)epoch_time);
			ofs_depth << fileName << " depth/" << fileName << ".png" << endl;
			cout << fileName << endl;
			fileNum++;
			colorMatC3.release();
		}
		// 上キー
		else if ((key >> 16) == VK_UP) {
			target_angle = std::min(target_angle + 1, (LONG)NUI_CAMERA_ELEVATION_MAXIMUM);
			pSensor->NuiCameraElevationSetAngle(target_angle);
		}
		// 下キー
		else if ((key >> 16) == VK_DOWN) {
			target_angle = std::max(target_angle - 1, (LONG)NUI_CAMERA_ELEVATION_MINIMUM);
			pSensor->NuiCameraElevationSetAngle(target_angle);
		}
		pSensor->NuiCameraElevationGetAngle(&angle);
		cout << "tilt : " << target_angle << " " << angle << "[deg]" << endl;

		count++;

		colorMat.release();
		depthMat.release();
		depthMatC4.release();
		rgbdMat.release();
	}

	// Kinectの終了処理
	pSensor->NuiShutdown();
	pCordinateMapper->Release();
	CloseHandle(hColorEvent);
	CloseHandle(hDepthPlayerEvent);

	cv::destroyAllWindows();

	return 0;
}
