#include "Utility.h"
#include "Constant.h"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;
using namespace FM;

static const int deltaAngle[8][2] = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 } };	// ˳ʱ��

Mat fillHoles(Mat imInput){
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

vector< vector<bool> > getCorners(const Mat& mat){
	vector< vector<bool> > cornerFlag(mat.size().height);
	for (int i = 0; i < cornerFlag.size(); i++){
		cornerFlag[i] = vector<bool>(mat.size().width);
	}
	for (int i = 0; i<mat.size().height; i++){
		for (int j = 0; j<mat.size().width; j++){
			cornerFlag[i][j] = false;
		}
	}
	PointSet corners;
	goodFeaturesToTrack(mat, corners, 20, .05, 20, Mat(), 5, true, .1);
	cout << corners.size() << " corner points." << endl;
	for (int i = 0; i<corners.size(); i++){
		cornerFlag[corners[i].y][corners[i].x] = true;
		//circle(mat, corners[i], 5, Scalar(255, 0, 0));
	}
	//imshow("corners", mat); waitKey();
	return cornerFlag;
}

Point findStartPoint(const Mat& mat){
	for (int i = 0; i < mat.cols; i++){
		for (int j = 0; j < mat.rows; j++){
			if (mat.at<uchar>(j, i) > 128){
				int firstStepCnt = 0, maxSecondStepCnt = 0;
				for (int i = 0; i < 8; i++){	// todo: ע�⣡����ͺ���ʹ��i��j�Ǵ���ġ�����ʹ��������Ϊ���������λ�û��ʵ���Զ���á�
					int tempX = i + deltaAngle[i][0];
					int tempY = j + deltaAngle[i][1];
					if (mat.at<uchar>(tempY, tempX) > 128){
						firstStepCnt++;
						int secondStepCnt = 0;
						for (int j = 0; j<8; j++){
							int tempXX = tempX + deltaAngle[j][0];
							int tempYY = tempY + deltaAngle[j][1];
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
	// todo: �����ҵ��Ķ������Щ�ǵ㲢�������棡
	vector< vector<int> > orderAtThisPoint(mat.size().height);
	for (int i = 0; i < orderAtThisPoint.size(); i++){
		orderAtThisPoint[i] = vector<int>(mat.size().width);
	}
	for (int i = 0; i < mat.size().height; i++){
		for (int j = 0; j < mat.size().width; j++){
			orderAtThisPoint[i][j] = 0;
		}
	}
	int currentX = start.x;
	int currentY = start.y;
	while (mat.at<uchar>(currentY, currentX) > 128){
		polygon.push_back(Point(currentX, currentY));
		orderAtThisPoint[currentY][currentX] = polygon.size();
		mat.at<uchar>(currentY, currentX) = 0;	// ��Ǵ˵��Ѿ��ù��������һص��˵�
		int firstStepCnt = 0;
		int secondStepCnt[8] = { 0 };	// ���ѡ��һ����󣬵ڶ������ҵ�������
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
		if (firstStepCnt == 1){	// ֻ�ҵ���һ��������
			currentX += deltaAngle[firstStepDirection[0]][0];
			currentY += deltaAngle[firstStepDirection[0]][1];
		}
		else if (firstStepCnt > 1){	// �����ѡ�ĺ�����
			int minCnt = infinity, direct = -1;
			for (int i = 0; i<firstStepCnt; i++){
				if (secondStepCnt[i] >= 1 && secondStepCnt[i] < minCnt){	// �Һ��������ٵĺ����㣿
					minCnt = secondStepCnt[i];
					direct = firstStepDirection[i];
				}
			}
			currentX += deltaAngle[direct][0];
			currentY += deltaAngle[direct][1];
		}
		else{	// û�ҵ������㣬����εĹ�������
			cout << "find no next points" << endl;
		}
	}
	return orderAtThisPoint;
}