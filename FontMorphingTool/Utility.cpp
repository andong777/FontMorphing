#include "Utility.h"
#include "Constant.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <queue>

using namespace std;
using namespace cv;
using namespace FM;

static const int deltaAngle[8][2] = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 } };	// ˳ʱ��

Mat fillHoles(Mat& imInput){
	Mat imShow = Mat::zeros(imInput.size(), CV_8UC3);    // for show result
	vector<PointSet > contours;
	vector<Vec4i> hierarchy;
	findContours(imInput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	for (int idx = 0; idx < contours.size(); idx++){
		drawContours(imShow, contours, idx, Scalar::all(255), CV_FILLED);
	}
	Mat imFilledHoles;
	cvtColor(imShow, imFilledHoles, CV_BGR2GRAY);
	//imshow("", imFilledHoles); waitKey();
	//imFilledHoles = imFilledHoles > 0;
	imShow.release();
	return imFilledHoles;
}

Mat getEdge(Mat& mat)
{
	Mat filled = fillHoles(mat);
	//GaussianBlur(filled, filled, Size(0, 0), 2);
	Mat edge = Mat::zeros(filled.size(), CV_8UC3);    // for show result
	vector<PointSet> contours;
	vector<Vec4i> hierarchy;
	findContours(filled, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	vector<PointSet> contoursSmoothed(contours.size());
	for (int i = 0; i < contours.size(); i++){
		approxPolyDP(contours[i], contoursSmoothed[i], 3, true);
		drawContours(edge, contoursSmoothed, i, Scalar::all(255), 1);
	}
	cvtColor(edge, edge, CV_BGR2GRAY);
	//imwrite("edge.jpg", edge);
	return edge;
}

vector< vector<bool> > getCorners(const Mat& mat, PointSet& outputCorners){
	// todo: �����ʱ�䣬�Ľ�Ч��
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
	// todo: �����ҵ��Ķ������Щ�ǵ㲢�������棡
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

bool checkTriangle(const Point& a, const Point& b, const Point& c)
{
	float va2vb = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + .0);
	float vb2vc = sqrt((b.x - c.x) * (b.x - c.x) + (b.y - c.y) * (b.y - c.y) + .0);
	float va2vc = sqrt((a.x - c.x) * (a.x - c.x) + (a.y - c.y) * (a.y - c.y) + .0);
	float minEdge = min(min(va2vb, vb2vc), va2vc);
	float maxEdge = max(max(va2vb, vb2vc), va2vc);
	if (maxEdge / minEdge >= 10){	// �����ݸ�
		cout << "too thin" << endl;
		return false;
	}
	float twoEdges = va2vb + vb2vc + va2vc - maxEdge;
	//if (maxEdge / twoEdges >= 0.95){	// ���ڰ���	// �Ƿ����������������Ϊ���������ǰ��ñȽϽ��ġ� 4.15
	//	cout << "too short" << endl;
	//	return false;
	//}
	return true;
}

TriMesh getConnectTri(const PointSet& pointSet, const vector<int>& strokeEndAtVertex)
{
	TriMesh connectTri;
	int numStroke = strokeEndAtVertex.size();
	PointSet strokeCenter(numStroke);
	bool **connected = new bool*[numStroke];
	for (int i = 0; i < numStroke; i++){
		connected[i] = new bool[numStroke];
	}
	for (int i = 0; i < numStroke; i++){
		for (int j = 0; j < numStroke; j++){
			connected[i][j] = false;
		}
	}
	for (int i = 0; i < numStroke; i++){
		int startPos = (i == 0 ? 0 : strokeEndAtVertex[i - 1] + 1);
		int endPos = strokeEndAtVertex[i];
		int numPoints = endPos - startPos + 1;
		Point center(0, 0);
		for (int j = startPos; j <= endPos; j++){
			center += pointSet[j];
		}
		center.x /= numPoints;
		center.y /= numPoints;
		strokeCenter[i] = center;
	}

	for (int i = 0; i < numStroke; i++){
		int startPos = (i == 0 ? 0 : strokeEndAtVertex[i - 1] + 1);
		int endPos = strokeEndAtVertex[i];
		int numPoints = endPos - startPos + 1;
		Point& center = strokeCenter[i];

		int minStrokeIdx = -1;
		float minStrokeDist = infinity;
		for (int j = 0; j < numStroke; j++){
			if (j != i && !connected[i][j]){
				Point diff = center - strokeCenter[j];
				float dist = sqrt(diff.x * diff.x + diff.y * diff.y + .0);
				if (dist < minStrokeDist){
					minStrokeDist = dist;
					minStrokeIdx = j;
				}
			}
		}
		connected[i][minStrokeIdx] = true;
		connected[minStrokeIdx][i] = true;

		// �ҳ�����ʻ����뵱ǰ�ʻ���������ĵ㡣
		int startPosThis = (minStrokeIdx == 0 ? 0 : strokeEndAtVertex[minStrokeIdx - 1] + 1);
		int endPosThis = strokeEndAtVertex[minStrokeIdx];
		int numPointsThis = endPosThis - startPosThis + 1;
		int minPointIdx = -1;
		float minPointDist = infinity;
		for (int j = startPosThis; j <= endPosThis; j++){
			Point diff = center - pointSet[j];
			float dist = sqrt(diff.x * diff.x + diff.y * diff.y + .0);
			if (dist < minPointDist){
				minPointDist = dist;
				minPointIdx = j;
			}
		}
		int interval = 2;	// �ڸ���ȡ�����㡣
		int va = (minPointIdx - interval - startPosThis + numPointsThis) % numPointsThis + startPosThis;
		int vb = (minPointIdx + interval - startPosThis + numPointsThis) % numPointsThis + startPosThis;
		int vc;

		// Ȼ�����ڵ�ǰ�ʻ������Ҽ����㣬����ж������Ρ�
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
	for (int i = 0; i < numStroke; i++){
		delete[] connected[i];
	}
	delete[] connected;
	return connectTri;
}

PointSet getSamplePoints(const PointSet& polygon, vector< vector<bool> >& cornerFlag, double keyPointsInterval, int numSample)
{
	PointSet samplePoints;
	int numKeyPoints = floor(polygon.size() / keyPointsInterval);	// ���ݹؼ���֮����룬����ؼ���������������εĸ���
	if (numKeyPoints == 0)	numKeyPoints = 1;	// �Բ���һ������ĵ���һ�����䴦�� 4.16
	double extraInterval = polygon.size() - keyPointsInterval * numKeyPoints;	// ���ֺ����ĳ��ȣ������ֵ�ÿ����������
	keyPointsInterval += extraInterval / numKeyPoints;
	double sampleStep = keyPointsInterval / numSample;	// ÿ���������У�������ͨ��֮��ľ���
	for (int i = 0; i < numKeyPoints; i++){
		double basePosition = keyPointsInterval * i;
		for (int j = 0; j < numSample; j++){
			int pointIndex = floor(basePosition + sampleStep * j);
			samplePoints.push_back(polygon[pointIndex]);
			// replace regular sample points by the corner points
			for (int p = 1; p <= sampleStep - 1; p++){
				int index = floor(fmod(basePosition + sampleStep * (j - .5) + p + polygon.size(), polygon.size()));
				Point pnt = polygon[index];
				if (cornerFlag[pnt.y][pnt.x]){
					cout << "replace corner (" << pnt.x << ", " << pnt.y << ")" << endl;
					//cornerFlag[pnt.y][pnt.x] = false;
					samplePoints.pop_back();
					samplePoints.push_back(pnt);
					break;
				}
			}
		}
	}
	return samplePoints;
}

PointSet getSamplePoints(const PointSet& polygon, vector< vector<bool> >& cornerFlag, PointSet& keyPoints, vector< vector<int> >& orderAtThisPoint, int numSample)
{
	PointSet samplePoints;
	// �޸ĺ�Ĳ�������Ҫ�󽫹ؼ����С�����ź���
	std::sort(keyPoints.begin(), keyPoints.end(), [=](const Point & a, const Point & b) -> bool { return orderAtThisPoint[a.y][a.x] < orderAtThisPoint[b.y][b.x]; });

	for (int i = 0; i < keyPoints.size(); i++){
		int i2 = (i + 1) % keyPoints.size();	// ��һ���ؼ���
		samplePoints.push_back(keyPoints[i]);
		int kp1Order = orderAtThisPoint[keyPoints[i].y][keyPoints[i].x];
		int kp2Order = orderAtThisPoint[keyPoints[i2].y][keyPoints[i2].x];
		assert(kp1Order >= 0 && kp2Order >= 0);
		int dist = (kp2Order + polygon.size() - kp1Order) % polygon.size();
		if (dist == 0)	dist = polygon.size();	// ��ֻ��һ���ؼ����������⴦�� 4.16
		double numIntervalPoints = double(dist / numSample);
		for (int j = 1; j < numSample; j++){
			int index = floor(fmod(kp1Order + numIntervalPoints * j + polygon.size(), polygon.size()));
			samplePoints.push_back(polygon[(index + polygon.size()) % polygon.size()]);
			// replace regular sample points by the corner points
			for (int p = 1; p <= floor(abs(numIntervalPoints)) - 1; p++){
				index = floor(fmod(kp1Order + numIntervalPoints*(j - .5) + p + polygon.size(), polygon.size()));
				Point pnt = polygon[(index + polygon.size()) % polygon.size()];
				if (cornerFlag[pnt.y][pnt.x]){
					cout << "replace corner (" << pnt.x << ", " << pnt.y << ")" << endl;
					//cornerFlag[pnt.y][pnt.x] = false;
					samplePoints.pop_back();
					samplePoints.push_back(pnt);
					break;
				}
			}
		}
	}
	return samplePoints;
}