#include "Utility.h"
#include "Constant.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <queue>

using namespace std;
using namespace cv;
using namespace FM;

static const int deltaAngle[8][2] = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 } };	// 顺时针

Mat fillHoles(Mat& imInput){
	Mat imShow = Mat::zeros(imInput.size(), CV_8UC3);    // for show result
	vector<PointSet > contours;
	vector<Vec4i> hierarchy;
	findContours(imInput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	for (int idx = 0; idx < contours.size(); idx++){
		drawContours(imShow, contours, idx, Scalar::all(255), CV_FILLED, 8);
	}
	Mat imFilledHoles;
	cvtColor(imShow, imFilledHoles, CV_BGR2GRAY);
	imFilledHoles = imFilledHoles > 0;
	imShow.release();
	return imFilledHoles;
}

vector< vector<bool> > getCorners(const Mat& mat, PointSet& outputCorners){
	// todo: 如果有时间，改进效果
	vector< vector<bool> > cornerFlag(mat.size().height);
	for (int i = 0; i < cornerFlag.size(); i++){
		cornerFlag[i] = vector<bool>(mat.size().width);
	}
	for (int i = 0; i<mat.size().height; i++){
		for (int j = 0; j<mat.size().width; j++){
			cornerFlag[i][j] = false;
		}
	}
	goodFeaturesToTrack(mat, outputCorners, 20, .05, 20, Mat(), 5, true, .1);
	cout << outputCorners.size() << " corner points." << endl;
	for (int i = 0; i < outputCorners.size(); i++){
		cornerFlag[outputCorners[i].y][outputCorners[i].x] = true;
		//circle(mat, outputCorners[i], 5, Scalar(255, 0, 0));
	}
	//imshow("corners", mat); waitKey();
	return cornerFlag;
}

Point findStartPoint(const Mat& mat){
	for (int i = 0; i < mat.cols; i++){
		for (int j = 0; j < mat.rows; j++){
			if (mat.at<uchar>(j, i) > 128){
				int firstStepCnt = 0, maxSecondStepCnt = 0;
				for (int d = 0; d < 8; d++){
					int tempX = i + deltaAngle[d][0];
					int tempY = j + deltaAngle[d][1];
					if (mat.at<uchar>(tempY, tempX) > 128){
						firstStepCnt++;
						int secondStepCnt = 0;
						for (int dd = 0; dd<8; dd++){
							int tempXX = tempX + deltaAngle[dd][0];
							int tempYY = tempY + deltaAngle[dd][1];
							if (mat.at<uchar>(tempYY, tempXX) > 128){
								secondStepCnt++;
							}
						}
						if (secondStepCnt > maxSecondStepCnt)	maxSecondStepCnt = secondStepCnt;
					}
				}
				if (firstStepCnt >= 1 && maxSecondStepCnt >= 1){
					return Point(i, j);
				}
			}
		}
	}
	return Point(-1, -1);
}

vector< vector<int> > getPolygon(Mat mat, Point start, PointSet& polygon){
	// todo: 这样找到的多边形有些角点并不在上面！
	vector< vector<int> > orderAtThisPoint(mat.size().height);	// starts from 0.
	for (int i = 0; i < orderAtThisPoint.size(); i++){
		orderAtThisPoint[i] = vector<int>(mat.size().width);
	}
	for (int i = 0; i < mat.size().height; i++){
		for (int j = 0; j < mat.size().width; j++){
			orderAtThisPoint[i][j] = -1;
		}
	}
	int currentX = start.x;
	int currentY = start.y;
	while (mat.at<uchar>(currentY, currentX) > 128){
		orderAtThisPoint[currentY][currentX] = polygon.size();
		polygon.push_back(Point(currentX, currentY));
		mat.at<uchar>(currentY, currentX) = 0;	// 标记此点已经用过，避免找回到此点
		int firstStepCnt = 0;
		int secondStepCnt[8] = { 0 };	// 标记选择一个点后，第二次能找到几个点
		int firstStepDirection[8] = { 0 };
		for (int i = 0; i<8; i++){
			int tempX = currentX + deltaAngle[i][0];
			int tempY = currentY + deltaAngle[i][1];
			if (mat.at<uchar>(tempY, tempX) > 128){
				firstStepCnt++;
				firstStepDirection[firstStepCnt - 1] = i;
				for (int j = 0; j<8; j++){
					int tempXX = tempX + deltaAngle[j][0];
					int tempYY = tempY + deltaAngle[j][1];
					if (mat.at<uchar>(tempYY, tempXX) > 128){
						secondStepCnt[firstStepCnt - 1]++;
					}
				}
			}
		}
		if (firstStepCnt == 1){	// 只找到了一个后续点
			currentX += deltaAngle[firstStepDirection[0]][0];
			currentY += deltaAngle[firstStepDirection[0]][1];
		}
		else if (firstStepCnt > 1){	// 多个可选的后续点
			int minCnt = infinity, direct = -1;
			for (int i = 0; i<firstStepCnt; i++){
				if (secondStepCnt[i] >= 1 && secondStepCnt[i] < minCnt){	// 找后续点最少的后续点？
					minCnt = secondStepCnt[i];
					direct = firstStepDirection[i];
				}
			}
			currentX += deltaAngle[direct][0];
			currentY += deltaAngle[direct][1];
		}
		else{	// 没找到后续点，多边形的构建结束
			cout << "find no next points" << endl;
		}
	}
	return orderAtThisPoint;
}

bool checkTriangle(const Point& a, const Point& b, const Point& c)
{
	float va2vb = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + .0);
	float vb2vc = sqrt((b.x - c.x) * (b.x - c.x) + (b.y - c.y) * (b.y - c.y) + .0);
	float va2vc = sqrt((a.x - c.x) * (a.x - c.x) + (a.y - c.y) * (a.y - c.y) + .0);
	float minEdge = min(min(va2vb, vb2vc), va2vc);
	float maxEdge = max(max(va2vb, vb2vc), va2vc);
	if (maxEdge / minEdge >= 5){	// 过于瘦高
		return false;
	}
	float twoEdges = va2vb + vb2vc + va2vc - maxEdge;
	if (maxEdge / twoEdges >= 0.9){	// 过于矮胖
		return false;
	}
	return true;
}

TriMesh getConnectTri(const PointSet& pointSet, const vector<int>& strokeEndAtVertex)
{
	int numStroke = strokeEndAtVertex.size();
	TriMesh connectTri;

	// stub
	/*connectTri.push_back(Triangle(35, 36, 137));
	connectTri.push_back(Triangle(171, 173, 75));*/

	for (int i = 0; i < numStroke - 1; i++){
		int startPos = i == 0 ? 0 : strokeEndAtVertex[i - 1] + 1;
		int endPos = strokeEndAtVertex[i];
		int numPoints = endPos - startPos + 1;
		Point center(0, 0);	// 当前笔画的重心
		for (int j = startPos; j <= endPos; j++){
			center += pointSet[j];
		}
		center.x /= numPoints;
		center.y /= numPoints;

		// 找出下个笔画上离当前笔画重心最近的点。
		int minIdx = -1;
		int minDist = infinity;
		int startPosNext = strokeEndAtVertex[i] + 1;
		int endPosNext = strokeEndAtVertex[i + 1];
		int numPointsNext = endPosNext - startPosNext + 1;
		for (int j = startPosNext; j <= endPosNext; j++){
			Point diff = center - pointSet[j];
			int dist = sqrt(diff.x * diff.x + diff.y * diff.y + .0);
			if (dist < minDist){
				minDist = dist;
				minIdx = j;
			}
		}
		int interval = 2;	// 在附近取两个点。
		int va = (minIdx - interval - startPosNext + numPointsNext) % numPointsNext + startPosNext;
		int vb = (minIdx + interval - startPosNext + numPointsNext) % numPointsNext + startPosNext;
		int vc;

		// 然后再在当前笔画附近找几个点，最后判断三角形。
		int *pointDist = new int[numPoints];
		for (int j = 0; j < numPoints; j++)	pointDist[j] = infinity;
		auto comparePoints = [=](int a, int b){ return pointDist[a - startPos] > pointDist[b - startPos] ; };
		priority_queue<int, vector<int>, decltype(comparePoints)> candidatePoints(comparePoints);
		for (int j = startPos; j <= endPos; j++){
			Point diff = center - pointSet[j];
			pointDist[j - startPos] = sqrt(diff.x * diff.x + diff.y * diff.y + .0);
			candidatePoints.push(j);
		}
		delete[] pointDist;
 		while (!candidatePoints.empty()){
			vc = candidatePoints.top();
			candidatePoints.pop();
			bool ok = checkTriangle(pointSet[va], pointSet[vb], pointSet[vc]);
			if (ok){
				connectTri.push_back(Triangle(va, vb, vc));
				break;
			}
		}
	}
	return connectTri;
}