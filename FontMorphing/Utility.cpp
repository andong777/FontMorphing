#include "Utility.h"
#include "Constant.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <queue>

using namespace std;
using namespace cv;
using namespace FM;

static const int deltaAngle[8][2] = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 } };	// 顺时针

struct Pair
{
	int a, b;
	Pair() : a(-1), b(-1) {};
	Pair(int _a, int _b) : a(_a), b(_b) {};
};

class UnionFindSet
{
public:
	UnionFindSet(int initial = 100);
	~UnionFindSet();
	int findParent(int x);
	void makeUnion(int x, int y);
	int getParents(vector<int>& parents);
	int getChildren(int parent, vector<int>& children);

private:
	int capacity;
	int *parent;
	int *rank;
};

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

Mat getEdge(Mat& mat, PointSet& contour)
{
	Mat filled = fillHoles(mat);
	//GaussianBlur(filled, filled, Size(0, 0), 2);
	Mat edge = Mat::zeros(filled.size(), CV_8UC3);
	vector<PointSet> contours;
	vector<Vec4i> hierarchy;
	findContours(filled, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	vector<PointSet> contoursSmoothed(contours.size());
	for (int i = 0; i < contours.size(); i++){
		approxPolyDP(contours[i], contoursSmoothed[i], 3, true);
		drawContours(edge, contoursSmoothed, i, Scalar::all(255), 1);
	}
	cvtColor(edge, edge, CV_BGR2GRAY);
	//imshow("edge", edge); waitKey();
	int idx = 0, maxCount = -1;
	for (int i = 0; i < contours.size(); i++){	// 处理孤立点的情况。（见楷体“掣”字） 4.27
		if ((int)contours[i].size() > maxCount){
			maxCount = contours[i].size();
			idx = i;
		}
	}
	contour.insert(contour.end(), contours[idx].begin(), contours[idx].end());
#ifdef VERBOSE
	cout << "There are " << contour.size() << " points on the polygon" << endl;
	cout << "start point (" << contour[0].x << ", " << contour[0].y << ")" << endl;
#endif
	return edge;
}

vector< vector<bool> > getCornerPoints(const Mat& mat, const PointSet& polygon, PointSet& outputCorners){
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
	Mat mat2draw = mat.clone();
	for (int i = 0; i < outputCorners.size(); i++){
		Point& corner = outputCorners[i];
		auto idx = std::find(polygon.begin(), polygon.end(), corner);
		if (idx == polygon.end()){	// 角点不在轮廓上
			int minIdx = -1, minDist = infinity;
			for (int j = 0; j < polygon.size(); j++){
				Point diff = polygon[j] - corner;
				int dist = diff.x * diff.x + diff.y * diff.y;
				if (dist < minDist){
					minDist = dist;
					minIdx = j;
				}
			}
			assert(minIdx >= 0);
#ifdef VERBOSE
			cout << "replace corner (" << corner.x << ", " << corner.y << ") with polygon point " << minIdx << " (" << polygon[minIdx].x << ", " << polygon[minIdx].y << ") dist: " << minDist << endl;
#endif
			outputCorners[i] = polygon[minIdx];
		}
		cornerFlag[corner.y][corner.x] = true;	// 引用的地址是不变的，如果改变了地址存储的内容，则引用也发生改变。
		circle(mat2draw, corner, 2, Scalar(255, 0, 0), CV_FILLED);
	}
#ifdef VERBOSE
	cout << outputCorners.size() << " corner points." << endl;
#endif
	//imwrite("corners.jpg", mat2draw);
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

// deprecated. use getEdge and get the output contour instead.
vector< vector<int> > getPolygon(Mat mat, Point start, PointSet& polygon, const PointSet& corners){
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

float evaluateNegative(const Point& a, const Point& b, const Point& c)
{
	float va2vb = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + .0);
	float vb2vc = sqrt((b.x - c.x) * (b.x - c.x) + (b.y - c.y) * (b.y - c.y) + .0);
	float va2vc = sqrt((a.x - c.x) * (a.x - c.x) + (a.y - c.y) * (a.y - c.y) + .0);
	float minEdge = min(min(va2vb, vb2vc), va2vc);
	float maxEdge = max(max(va2vb, vb2vc), va2vc);
	float twoEdges = va2vb + vb2vc + va2vc - maxEdge;
	int maxLength = 100;
	int minLength = 5;
	if (va2vb + va2vc <= vb2vc || va2vb + vb2vc <= va2vc || va2vc + vb2vc <= va2vb){	// 先检查是否构成三角形。 4.25
		return -100;
	}
	if (maxEdge / minEdge >= 10){	// 过于瘦高
		//cout << "too thin" << endl;
		return -50;
	}
	if ((twoEdges - maxEdge) / maxEdge <= 0.05){		// 过于矮胖	// 是否会出现这种情况？因为有两个点是挨得比较近的。 4.15	// 确实会出现。 4.25
		//cout << "too short" << endl;
		return -50;
	}
	if (va2vb < minLength && va2vc < minLength && vb2vc < minLength){	// 三条边都小于，认为三角形太小。允许存在一条边小于。 4.26
		//cout << "too small" << endl;
		return -10;
	}
	if (va2vb > maxLength || va2vc > maxLength || vb2vc > maxLength){	// 三条边有一条大于，认为三角形太大。 4.27
		// cout << "too big" << endl;
		return -10;
	}
	return 0;	// 表示符合基本要求
}

float evaluatePositive(const Point& a, const Point& b, const Point& c)
{
	float va2vb = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + .0);
	float vb2vc = sqrt((b.x - c.x) * (b.x - c.x) + (b.y - c.y) * (b.y - c.y) + .0);
	float va2vc = sqrt((a.x - c.x) * (a.x - c.x) + (a.y - c.y) * (a.y - c.y) + .0);
	float minEdge = min(min(va2vb, vb2vc), va2vc);
	float maxEdge = max(max(va2vb, vb2vc), va2vc);
	int maxLength = 100;
	int minLength = 5;
	int score = 0;
	//score += min(va2vb, va2vc) / max(va2vb, va2vc) * 10 + min(va2vb, vb2vc) / max(va2vb, vb2vc) * 10 + min(vb2vc, va2vc) / max(vb2vc, va2vc) * 10;
	score += minEdge / maxLength * 50;	// 三角形越大影响越大。 4.29
	return score;
}

float evaluateTriangle(const Point& a, const Point& b, const Point& c)
{
	int score = evaluateNegative(a, b, c);
	if (score >= 0){
		score = evaluatePositive(a, b, c);
	}
	return score;
	
}

TriMesh getConnectTri(const PointSet& sourcePoints, const PointSet& targetPoints, const vector<int>& strokeEndAtVertex)
{
	TriMesh connectTri;
	int numStroke = strokeEndAtVertex.size();
	bool **connected = new bool*[numStroke];
	bool **tried = new bool*[numStroke];
	for (int i = 0; i < numStroke; i++){
		connected[i] = new bool[numStroke];
		tried[i] = new bool[numStroke];
	}
	for (int i = 0; i < numStroke; i++){
		for (int j = 0; j < numStroke; j++){
			connected[i][j] = false;
			tried[i][j] = false;
		}
		connected[i][i] = true;	// 自身不能连接。
		tried[i][i] = true;
	}

	int **strokeDistance = new int*[numStroke];
	Pair **strokeNearPoint = new Pair*[numStroke];
	for (int i = 0; i < numStroke; i++){
		strokeDistance[i] = new int[numStroke];
		strokeNearPoint[i] = new Pair[numStroke];
	}
	int averageStrokeDistance = 0;
	int averageCnt = 0;
	const int minThreshold = 1500;	// 避免两个笔画连在一起导致找到的两点距离太近或重合。 4.27
	for (int i = 0; i < numStroke; i++){
		for (int j = 0; j < numStroke; j++){
			strokeDistance[i][j] = infinity;
			if (j <= i)	continue;
			int minIdxA = -1, minIdxB = -1, minDist = infinity;
			for (int m = (i == 0) ? 0 : strokeEndAtVertex[i - 1] + 1; m <= strokeEndAtVertex[i]; m++){
				for (int n = (j == 0) ? 0 : strokeEndAtVertex[j - 1] + 1; n <= strokeEndAtVertex[j]; n++){
					Point diff = sourcePoints[m] - sourcePoints[n];
					int dist = diff.x * diff.x + diff.y * diff.y;
					if (dist < minDist && dist > minThreshold){
						if (dist < minThreshold)
						{
							continue;
						}	// 距离过近的不要
						minDist = dist;
						minIdxA = m;
						minIdxB = n;
					}
				}
			}
			strokeDistance[i][j] = strokeDistance[j][i] = minDist;
			strokeNearPoint[i][j] = Pair(minIdxA, minIdxB);
			strokeNearPoint[j][i] = Pair(minIdxB, minIdxB);
			averageStrokeDistance += minDist;
			averageCnt++;
#ifdef VERBOSE
			cout << "stroke " << i << " to stroke " << j << " min dist: " << minDist << endl;
#endif
		}
	}
	averageStrokeDistance /= averageCnt;	// 平均距离

	UnionFindSet *ufs = new UnionFindSet(numStroke);
	vector<int> parts;
	int max_loop_times = 100;	// 最多支持100个笔画，足够了。
	int loop_times_cnt = 0;
	int numPart = ufs->getParents(parts);
	while (numPart > 1){
		loop_times_cnt++;
		if (loop_times_cnt > max_loop_times){
			cout << "quit because of exceeding max loop times" << endl;
			break;	// 避免死循环。 4.26
		}
		int minStrokeIdxA = -1, minStrokeIdxB = -1, minStrokeDist = infinity;
		for (int p = 0; p < numPart; p++){
			vector<int> strokesA, strokesB;
			ufs->getChildren(parts[p], strokesA);
			for (int i = 0; i < numPart; i++){
				if (parts[i] == parts[p] || ufs->findParent(parts[i]) == ufs->findParent(parts[p]))	continue;
				ufs->getChildren(parts[i], strokesB);
				for (int j = 0; j < strokesA.size(); j++){
					for (int k = 0; k < strokesB.size(); k++){
						if (strokesA[j] == strokesB[k] || tried[strokesA[j]][strokesB[k]] || connected[strokesA[j]][strokesB[k]]){
#ifdef VERBOSE
							//cout << "have tried" << endl;
#endif
							continue;
						}
						int strokeA = strokesA[j];
						int strokeB = strokesB[k];
						if (strokeDistance[strokeA][strokeB] < minStrokeDist){
							minStrokeDist = strokeDistance[strokeA][strokeB];
							minStrokeIdxA = strokeA;
							minStrokeIdxB = strokeB;
						}
					}
				}
			}
		}
		if (minStrokeIdxA < 0 || minStrokeIdxB < 0)	continue;
		tried[minStrokeIdxA][minStrokeIdxB] = true;	// 尝试过的要标记，避免反复尝试同一个组合。
		tried[minStrokeIdxB][minStrokeIdxA] = true;

		int startPosA = (minStrokeIdxA == 0 ? 0 : strokeEndAtVertex[minStrokeIdxA - 1] + 1);
		int endPosA = strokeEndAtVertex[minStrokeIdxA];
		int numPointsA = endPosA - startPosA + 1;
		int startPosB = (minStrokeIdxB == 0 ? 0 : strokeEndAtVertex[minStrokeIdxB - 1] + 1);
		int endPosB = strokeEndAtVertex[minStrokeIdxB];
		int numPointsB = endPosB - startPosB + 1;
		Pair minPointPair = strokeNearPoint[minStrokeIdxA][minStrokeIdxB];
		int va = minPointPair.a;
		int vb = minPointPair.b;
		int vc;

		vector<int> candidatePoints;
		for (int i = startPosA; i <= endPosA; i++){
			candidatePoints.push_back(i);
		}
		for (int i = startPosB; i <= endPosB; i++){
			candidatePoints.push_back(i);
		}
		int bestIdx = -1;
		float bestScore = -1 * infinity;
		for (int i = 0; i < candidatePoints.size(); i++){
			int vtemp = candidatePoints[i];
			int score = evaluateTriangle(sourcePoints[va], sourcePoints[vb], sourcePoints[vtemp]) + evaluateNegative(targetPoints[va], targetPoints[vb], targetPoints[vtemp]) / 2;
			if (score > bestScore){
				bestScore = score;
				bestIdx = candidatePoints[i];
			}
		}
		vc = bestIdx;

		connectTri.push_back(Triangle(va, vb, vc));
		connected[minStrokeIdxA][minStrokeIdxB] = true;
		connected[minStrokeIdxB][minStrokeIdxA] = true;
#ifdef VERBOSE
		cout << "connect " << minStrokeIdxA << " and " << minStrokeIdxB << endl;
#endif
		ufs->makeUnion(minStrokeIdxA, minStrokeIdxB);

		numPart = ufs->getParents(parts);
		cout << "There are " << numPart << " parts left" << endl;
	} // end while
	cout << "loop " << loop_times_cnt << " times" << endl;

	// 对距离特别近（达到某个阈值）但已经在一个集合中的笔画也建立连接三角形。 4.26
	int before = connectTri.size();
	cout << "extra step of generating connection triangle" << endl;
	int threshold = averageStrokeDistance;	// 平方距离
	cout << "threshold = " << threshold << endl;
	int max_extra = numStroke;	// 最多添加多少个
	int extra_cnt = 0;
	auto compare = [strokeDistance](Pair sp1, Pair sp2) { return strokeDistance[sp1.a][sp1.b] > strokeDistance[sp2.a][sp2.b]; };
	priority_queue<Pair, vector<Pair>, decltype(compare)> strokePairs(compare);
	for (int i = 0; i < numStroke; i++){
		for (int j = i + 1; j < numStroke; j++){
			if (connected[i][j])	continue;
			if (strokeDistance[i][j] < threshold){
				Pair pointPair = strokeNearPoint[i][j];
				Point diff = targetPoints[pointPair.a] - targetPoints[pointPair.b];
				if (diff.x * diff.x + diff.y * diff.y < 3 * threshold){	// 也判断笔画在目标字形中的距离
					strokePairs.push(Pair(i, j));
				}
			}
		}
	}
	while (!strokePairs.empty() && extra_cnt < max_extra){
		Pair& sp = strokePairs.top();
		strokePairs.pop();
		int i = sp.a, j = sp.b;
		if (connected[i][j])	continue;
		int startPosA = (i == 0 ? 0 : strokeEndAtVertex[i - 1] + 1);
		int endPosA = strokeEndAtVertex[i];
		int numPointsA = endPosA - startPosA + 1;
		int startPosB = (j == 0 ? 0 : strokeEndAtVertex[j - 1] + 1);
		int endPosB = strokeEndAtVertex[j];
		int numPointsB = endPosB - startPosB + 1;
		Pair minPointPair = strokeNearPoint[i][j];
		int va = minPointPair.a;
		int vb = minPointPair.b;
		int vc;

		vector<int> candidatePoints;
		for (int i = startPosA; i <= endPosA; i++){
			candidatePoints.push_back(i);
		}
		for (int i = startPosB; i <= endPosB; i++){
			candidatePoints.push_back(i);
		}
		int bestIdx = -1;
		float bestScore = -1 * infinity;
		for (int i = 0; i < candidatePoints.size(); i++){
			int vtemp = candidatePoints[i];
			int score = evaluateTriangle(sourcePoints[va], sourcePoints[vb], sourcePoints[vtemp]) + evaluateNegative(targetPoints[va], targetPoints[vb], targetPoints[vtemp]) / 2;
			if (score > bestScore){
				bestScore = score;
				bestIdx = candidatePoints[i];
			}
		}
		vc = bestIdx;

		connectTri.push_back(Triangle(va, vb, vc));
		extra_cnt++;
		connected[i][j] = true;
		connected[j][i] = true;
	}
	cout << (connectTri.size() - before) << " more connection triagnle(s) added" << endl;

	for (int i = 0; i < numStroke; i++){
		delete[] connected[i];
		delete[] tried[i];
		delete[] strokeDistance[i];
		delete[] strokeNearPoint[i];
	}
	delete[] connected;
	delete[] tried;
	return connectTri;
}

int APointOnWhichSideOfALineSegment(int x, int y, int xx1, int yy1, int xx2, int yy2)
{
	float judge;
	judge = (float)(y - yy1)*(float)(xx2 - xx1) - (float)(x - xx1)*(float)
		(yy2 - yy1);
	if (judge > 1.0) {
		return 1;
	}
	if (fabs(judge) <= 1.0) {
		return 0;
	}
	return -1;
}

bool GetIntersectPointOfTwoLineSegments(int x0, int y0, int x1, int y1, int xx1, int yy1, int xx2, int yy2, Point& p_point)
{
	int a0, a1, a2, a3;
	float a, b, c, d, bb;

	// boundary conditions: if the length of the line is less than 2.
	if ((abs(xx1 - xx2) < 2) && (abs(yy1 - yy2) < 2)) {
		return false;
	}
	if ((abs(x0 - x1) < 2) && (abs(y0 - y1) < 2)) {
		return false;
	}

	// judge relationship among points and line segments.
	a0 = APointOnWhichSideOfALineSegment(x0, y0, xx1, yy1, xx2, yy2);
	a1 = APointOnWhichSideOfALineSegment(x1, y1, xx1, yy1, xx2, yy2);
	a2 = APointOnWhichSideOfALineSegment(xx1, yy1, x0, y0, x1, y1);
	a3 = APointOnWhichSideOfALineSegment(xx2, yy2, x0, y0, x1, y1);

	// if the two line segments are intersectant.
	//(a0 != 0) && (a1 != 0) && (a2 != 0) && (a3 != 0) && 
	if ((a0 != a1) && (a2
		!= a3)) {
		// calculate point of intersection.
		a = x1 - x0;
		b = y1 - y0;
		c = xx2 - xx1;
		d = yy2 - yy1;
		bb = b *(float)c - a *(float)d;
		p_point.x = (int)(0.5 + (a *c *(float)(yy1 - y0) + b *c *(float)x0
			- a *d *(float)xx1) / bb);
		p_point.y = (int)(0.5 + (d *b *(float)(x0 - xx1) + b *c *(float)
			yy1 - d *a *(float)y0) / bb);

		if ((p_point.x == x0) && (p_point.y == y0)) {
			return true;
		}
		if ((p_point.x == x1) && (p_point.y == y1)) {
			return true;
		}
		return true;
	}
	else {
		return false;
	}
}

PointSet findKeyPoints(const PointSet& templatePointSet, const PointSet& dataPointSet)
{
	const PointSet& X = dataPointSet, Y = templatePointSet;
	int **distX2Y = new int*[X.size()];
	for (int i = 0; i < X.size(); i++){
		distX2Y[i] = new int[Y.size()];
	}
	for (int i = 0; i < X.size(); i++){
		for (int j = 0; j < Y.size(); j++){
			Point diff = X[i] - Y[j];
			distX2Y[i][j] = diff.x * diff.x + diff.y * diff.y;
			if (X[i] == Y[j])	continue;
			for (int k = 0; k < Y.size(); k++){
				if (k == j || (k + 1) % Y.size() == j)	continue;
				Point l0 = Y[k], l1 = Y[(k + 1) % Y.size()];
				Point crossPnt;
				bool intersect = GetIntersectPointOfTwoLineSegments(X[i].x, X[i].y, Y[j].x, Y[j].y, l0.x, l0.y, l1.x, l1.y, crossPnt);
				if (intersect){
					distX2Y[i][j] = infinity;
				}
				/*cout << "" << i << "-> " << j << ": intersect: " << intersect << endl;
				Mat paint(500, 500, CV_8UC3);
				circle(paint, X[i], 3, Scalar(255, 0, 0));
				circle(paint, Y[j], 3, Scalar(0, 255, 0));
				line(paint, X[i], Y[j], Scalar(0, 255, 0));
				line(paint, l0, l1, Scalar(0, 0, 255));
				imshow("", paint); waitKey();*/
			}
		}
	}
	PointSet keyPoints;
	int *vertexBelongToTemplate = new int[X.size()];
	for (int i = 0; i < X.size(); i++)	vertexBelongToTemplate[i] = -1;
	for (int i = 0; i < Y.size(); i++){
		int minDist = infinity, index = -1;
		for (int j = index + 1; j < X.size(); j++){	// 保证顺序。 5.6
			if (vertexBelongToTemplate[j] < 0/* && vertexBelongToTemplate[(j - 1 + XSize) % XSize] < 0 && vertexBelongToTemplate[( j + 1) % XSize] < 0*/){	// 尚未确定该点的对应关系
				int dist = distX2Y[j][i]/*(X[j].x - Y[i].x)*(X[j].x - Y[i].x) + (X[j].y - Y[i].y)*(X[j].y - Y[i].y)*/;
				if (dist < minDist){
					minDist = dist;
					index = j;
				}
			}
		}
		vertexBelongToTemplate[index] = i;
		keyPoints.push_back(X[index]);
	}
	delete[] vertexBelongToTemplate;
	for (int i = 0; i < X.size(); i++){
		delete[] distX2Y[i];
	}
	delete[] distX2Y;
	cout << keyPoints.size() << " key points." << endl;
	return keyPoints;
}

PointSet getSamplePoints(const PointSet& polygon, int targetPolygonSize, double keyPointsInterval, int numSample)
{
	PointSet samplePoints;
#ifdef VERBOSE
	cout << "source: " << polygon.size() << ", target: " << targetPolygonSize << endl;
#endif
	int numKeyPoints = floor(polygon.size() / keyPointsInterval);	// 根据关键点之间距离，求出关键点个数，即采样段的个数
	if (targetPolygonSize > 0){	// 关键点的个数考虑目标笔画的大小，避免点太密。 4.16
		int numKeyPointsTarget = floor(targetPolygonSize / keyPointsInterval);
		numKeyPoints = min(numKeyPoints, numKeyPointsTarget);
	}
	if (numKeyPoints == 0)	numKeyPoints = 1;	// 对不足一个区间的当作一个区间处理 4.16
#ifdef VERBOSE
	cout << "numKeyPoints: " << numKeyPoints << endl;
#endif
	double extraInterval = polygon.size() - keyPointsInterval * numKeyPoints;	// 划分后多余的长度，被均分到每个采样段中
	keyPointsInterval += extraInterval / numKeyPoints;
	double sampleStep = keyPointsInterval / numSample;	// 每个采样段中，两个普通点之间的距离
	for (int i = 0; i < numKeyPoints; i++){
		double basePosition = keyPointsInterval * i;
		for (int j = 0; j < numSample; j++){
			int pointIndex = floor(basePosition + sampleStep * j);
			samplePoints.push_back(polygon[pointIndex]);
		}
	}
	return samplePoints;
}

PointSet getSamplePoints(const PointSet& polygon, PointSet& keyPoints, int numSample)
{
	PointSet samplePoints;
	for (int i = 0; i < keyPoints.size(); i++){
		int i2 = (i + 1) % keyPoints.size();	// 下一个关键点
		samplePoints.push_back(keyPoints[i]);
		int kp1Order = std::find(polygon.begin(), polygon.end(), keyPoints[i]) - polygon.begin();
		int kp2Order = std::find(polygon.begin(), polygon.end(), keyPoints[i2]) - polygon.begin();
		assert(kp1Order >= 0 && kp2Order >= 0);
		//cout << "kp1: " << kp1Order << " kp2: " << kp2Order << endl;
		int dist = (kp2Order + polygon.size() - kp1Order) % polygon.size();	// 考虑跨越起点的情况
		if (dist == 0)	dist = polygon.size();	// 对只有一个关键点的情况特殊处理 4.16
		double numIntervalPoints = double(dist / numSample);
		for (int j = 1; j < numSample; j++){
			int index = floor(fmod(kp1Order + numIntervalPoints * j + polygon.size(), polygon.size()));
			samplePoints.push_back(polygon[index]);
		}
	}
	return samplePoints;
}

void useCornerPoints(const PointSet& polygon, const PointSet& corners, PointSet& points)
{
	int *pointOrder = new int[points.size()];
	bool *used = new bool[points.size()];
	for (int i = 0; i < points.size() ;i++){
		pointOrder[i] = std::find(polygon.begin(), polygon.end(), points[i]) - polygon.begin();
		used[i] = false;
	}
	for (auto it = corners.begin(); it != corners.end(); it++){
		int cornerOrder = std::find(polygon.begin(), polygon.end(), *it) - polygon.begin();
		assert(cornerOrder != polygon.size());
		int minIdx = -1, minOrderDist = infinity;
		for (int j = 0; j < points.size(); j++){
			assert(pointOrder[j] != polygon.size());
			int temp = abs(cornerOrder - pointOrder[j]);
			int dist = min(temp, (int)polygon.size() - temp);	// 考虑大圈和小圈	4.20
			if (dist < minOrderDist && !used[j]){
				minOrderDist = dist;
				minIdx = j;
			}
		}
		points[minIdx] = (*it);
		used[minIdx] = true;
#ifdef VERBOSE
		cout << "replace corner (" << (*it).x << ", " << (*it).y << ")" << endl;
#endif
	}
	delete[] used;
	delete[] pointOrder;
}

bool moveSamplePoints(const PointSet& polygon, PointSet& points)
{
	const float sampleGap = kKeyPointsInterval / kNumSample;
	const float minSampleGap = sampleGap * 2 / 3;
	int numPoints = points.size();
	int polygonLength = polygon.size();
	int *order = new int[numPoints];
	for (int i = 0; i < numPoints; i++){
		order[i] = std::find(polygon.begin(), polygon.end(), points[i]) - polygon.begin();
		assert(order[i] != polygonLength);
	}
	int reach = 0;
	bool moved = false;
	int startPos = 0, startOrder, endPos, endOrder;
	while(startPos < numPoints && startPos > reach){	// 保证只遍历一次
		cout << startPos << endl;
		endPos = (startPos + 1) % points.size();
		startOrder = order[startPos];
		endOrder = order[endPos];
		int dist = (endOrder - startOrder + polygonLength) % polygonLength;
		if (dist > minSampleGap){
			startPos++;
			continue;
		}
		bool rightFound = false;
		while (!rightFound && endPos < numPoints){
			int tempOrder = endOrder;
			endPos = (endPos + 1 + numPoints) % numPoints;
			endOrder = order[endPos];
			if (endOrder < tempOrder)	return false;	// 确实会出现这种情况！老子不伺候了！
			dist = (endOrder - tempOrder + polygonLength) % polygonLength;
			if (dist >= sampleGap)	rightFound = true;
			if (startPos == endPos)	break;
		}
		if (!rightFound || startPos == endPos)	break;
		int intervalCount = (endPos - startPos + numPoints) % numPoints;
		float intervalLength = (endOrder - startOrder + polygonLength) % polygonLength;
		float avgDist = intervalLength / intervalCount;
		for (int j = 1; j <= intervalCount; j++){
			//cout << "move (" << points[(startPos + j) % numPoints].x << ", " << points[(startPos + j) % numPoints].y << ") to (" << polygon[(int)(startOrder + avgDist * j) % polygonLength].x << ", " << polygon[(int)(startOrder + avgDist * j) % polygonLength].y << ")" << endl;
			points[(startPos + j) % numPoints] = polygon[(int)(startOrder + avgDist * j) % polygonLength];
			moved = true;
		}
		reach = startPos;
		startPos = endPos;
	}
	delete[] order;
	return moved;
}


/* Customized Union Find Set */

UnionFindSet::UnionFindSet(int initial)
{
	capacity = initial;
	parent = new int[capacity];
	rank = new int[capacity];
	for (int i = 0; i < capacity; i++){
		parent[i] = i;
		rank[i] = 1;
	}
}

UnionFindSet::~UnionFindSet()
{
	delete[] parent;
	delete[] rank;
}

int UnionFindSet::findParent(int x)
{
	int px = x;
	while (px != parent[px]){
		px = parent[px];
	}
	int temp;
	while (x != px){
		temp = parent[x];
		parent[x] = px;
		x = temp;
	}
	return px;
}

void UnionFindSet::makeUnion(int x, int y)
{
	int px = findParent(x);
	int py = findParent(y);
	if (px != py){
		if (rank[px] >= rank[py]){
			parent[py] = px;
			rank[px] += rank[py];
		}
		else{
			parent[px] = py;
			rank[py] += rank[px];
		}
	}
}

int UnionFindSet::getParents(vector<int>& parents)
{
	parents.clear();
	for (int i = 0; i < capacity; i++){
		if (parent[i] == i){
			parents.push_back(i);
		}
	}
	return parents.size();
}

int UnionFindSet::getChildren(int parent, vector<int>& children)
{
	children.clear();
	for (int i = 0; i < capacity; i++){
		if (this->parent[i] == parent){
			children.push_back(i);
		}
	}
	return children.size();
}