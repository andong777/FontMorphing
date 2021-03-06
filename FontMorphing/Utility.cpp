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

void deleteDuplicatedPointsOnContour(PointSet& contour) {
	bool processOnceAgain = true;
	int loopTimes = 0;
	while (processOnceAgain & loopTimes < 10) {
		for (int i = 0; i < contour.size(); i++) {
			bool toBreak = false;
			for (int j = i + 1; j < contour.size(); j++) {
				Point& pi = contour[i];
				Point& pj = contour[j];
				int pix = pi.x; int pjx = pj.x;
				if (i != j && pi == pj) {
					int ps = 1;
					bool allEqual = true;
					while (i + ps < j && j - ps > i && ps < (j - i) / 2) {	// 导致把点全部删除的严重bug，需要如此判断一下。	16.1.15
						if (contour[i + ps] != contour[j - ps]) {
							allEqual = false;
							break;
						}
						ps++;
					}
					if (allEqual) {
						// 一条线的轮廓
						contour.erase(contour.begin() + i, contour.begin() + j);	// 保留一个点pj，保证轮廓还是连续的
						toBreak = true;
						break;
					}
				}
			}
			if (toBreak) {
				processOnceAgain = true;
				loopTimes++;
				break;
			}
		}
		processOnceAgain = false;
	}
}

/**
* Perform one thinning iteration.
* Normally you wouldn't call this function directly from your code.
*
* @param  im    Binary image with range = 0-1
* @param  iter  0=even, 1=odd
*/
void thinningIteration(cv::Mat& im, int iter)
{
	cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);

	for (int i = 1; i < im.rows - 1; i++)
	{
		for (int j = 1; j < im.cols - 1; j++)
		{
			uchar p2 = im.at<uchar>(i - 1, j);
			uchar p3 = im.at<uchar>(i - 1, j + 1);
			uchar p4 = im.at<uchar>(i, j + 1);
			uchar p5 = im.at<uchar>(i + 1, j + 1);
			uchar p6 = im.at<uchar>(i + 1, j);
			uchar p7 = im.at<uchar>(i + 1, j - 1);
			uchar p8 = im.at<uchar>(i, j - 1);
			uchar p9 = im.at<uchar>(i - 1, j - 1);

			int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
				(p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
				(p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
				(p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
			int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
			int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
			int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

			if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
				marker.at<uchar>(i, j) = 1;
		}
	}

	im &= ~marker;
}

/**
* Function for thinning the given binary image
*
* @param  im  Binary image with range = 0-255
*/
void thinning(cv::Mat& im)
{
	im /= 255;

	cv::Mat prev = cv::Mat::zeros(im.size(), CV_8UC1);
	cv::Mat diff;

	do {
		thinningIteration(im, 0);
		thinningIteration(im, 1);
		cv::absdiff(im, prev, diff);
		im.copyTo(prev);
	} while (cv::countNonZero(diff) > 0);

	im *= 255;
}

Mat getEdge(Mat& mat, PointSet& contour)
{
	Mat filled = fillHoles(mat);
	//GaussianBlur(filled, filled, Size(0, 0), 2);
	Mat edge = Mat::zeros(filled.size(), CV_8UC3);
	vector<PointSet> contours;
	vector<Vec4i> hierarchy;
	findContours(filled, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	//vector<PointSet> contoursSmoothed(contours.size());
	for (int i = 0; i < contours.size(); i++){
		//approxPolyDP(contours[i], contoursSmoothed[i], 3, true);
		drawContours(edge, contours, i, Scalar::all(255), 1);
		//drawContours(edge, contoursSmoothed, i, Scalar::all(255), 1);
	}
	cvtColor(edge, edge, CV_BGR2GRAY);
#ifdef VERBOSE
	imwrite("edge.jpg", edge);
#endif
	int idx = 0, maxCount = -1;
	for (int i = 0; i < contours.size(); i++){	// 处理孤立点的情况。（见楷体“掣”字） 4.27
		if ((int)contours[i].size() > maxCount){
			maxCount = contours[i].size();
			idx = i;
		}
	}
	contour.insert(contour.end(), contours[idx].begin(), contours[idx].end());
	//contour.insert(contour.end(), contoursSmoothed[idx].begin(), contoursSmoothed[idx].end());

	// 处理一条线的轮廓
	deleteDuplicatedPointsOnContour(contour);

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
	goodFeaturesToTrack(mat, outputCorners, 15, .05, 10, Mat(), 5, true, .1);	// 注意是对笔画检测角点，参考仿宋复杂笔画角点很多
	//goodFeaturesToTrack(mat, outputCorners, 20, .05, 20, Mat(), 5, true, .1);
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
		return -500;
	}
	if (maxEdge / minEdge >= 10){	// 过于瘦高
		//cout << "too thin" << endl;
		return -100;
	}
	if ((twoEdges - maxEdge) / maxEdge <= 0.05){		// 过于矮胖	// 是否会出现这种情况？因为有两个点是挨得比较近的。 4.15	// 确实会出现。 4.25
		//cout << "too short" << endl;
		return -100;
	}
	if (va2vb < minLength && va2vc < minLength && vb2vc < minLength){	// 三条边都小于，认为三角形太小。允许存在一条边小于。 4.26
		//cout << "too small" << endl;
		return -50;
	}
	if (va2vb > maxLength || va2vc > maxLength || vb2vc > maxLength){	// 三条边有一条大于，认为三角形太大。 4.27
		// cout << "too big" << endl;
		return -50;
	}
	return 0;	// 表示符合基本要求
}

bool sameDirection(const Point& u, const Point& v)
{
	if (u.x * v.x > 0 && u.y * v.y > 0)	return true;
	return false;
}

float cosineSimilarity(const Point& ua, const Point& ub, const Point& va, const Point& vb)
{
	// cosine similarity = cos(u, v) = u * v / (|u| |v|)
	Point u = ub - ua;
	Point v = vb - va;
	auto dot = [] (const Point& u, const Point& v) -> float { return u.x * v.x + u.y * v.y; };	// lambda表达式好棒！
	return dot(u, v) / (sqrt(dot(u, u)) * sqrt(dot(v, v)));
}

float evaluateTriangle(const Point& a, const Point& b, const Point& c, const Point& ta, const Point& tb, const Point& tc)
{
	int score = evaluateNegative(a, b, c);
	if (score >= 0){
		// 判断向量方向
		Point vab = b - a, tvab = tb - ta;
		Point vbc = c - b, tvbc = tc - tb;
		Point vca = a - c, tvca = ta - tc;
		//int directThreshold = 10;
		int vectorScore = 50;
		if (sameDirection(vab, tvab)){ score += vectorScore; }
		if (sameDirection(vbc, tvbc)){ score += vectorScore; }
		if (sameDirection(vca, tvca)){ score += vectorScore; }

		//score += min(va2vb, va2vc) / max(va2vb, va2vc) * 10 + min(va2vb, vb2vc) / max(va2vb, vb2vc) * 10 + min(vb2vc, va2vc) / max(vb2vc, va2vc) * 10;	// 这种评判没有道理
		
		// 边向量的方向进行评判
		score += (cosineSimilarity(a, b, ta, tb) + cosineSimilarity(b, c, tb, tc) + cosineSimilarity(c, a, tc, ta)) / 3 * 100;

		// 评判三角形大小
		float va2vb = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + .0);
		float vb2vc = sqrt((b.x - c.x) * (b.x - c.x) + (b.y - c.y) * (b.y - c.y) + .0);
		float va2vc = sqrt((a.x - c.x) * (a.x - c.x) + (a.y - c.y) * (a.y - c.y) + .0);
		float minEdge = min(min(va2vb, vb2vc), va2vc);
		int maxLength = 100;
		score += minEdge / maxLength * 50;	// 三角形越大影响越大。 4.29
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
					Point tdiff = targetPoints[m] - targetPoints[n];
					int dist = diff.x * diff.x + diff.y * diff.y;
					if (!sameDirection(diff, tdiff)){	// 尽量不选择源字形和目标字形形成的向量方向不同的两个点
						dist *= 10;
						if (dist < 0)	dist = infinity;	// 防止溢出，考虑的好周全
					}
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
	if (averageCnt > 0){	// 一个笔画的情况特殊处理
		averageStrokeDistance /= averageCnt;	// 平均距离
	}

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
			int score = evaluateTriangle(sourcePoints[va], sourcePoints[vb], sourcePoints[vtemp], targetPoints[va], targetPoints[vb], targetPoints[vtemp])
				+ evaluateNegative(targetPoints[va], targetPoints[vb], targetPoints[vtemp]) / 2;
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
			int score = evaluateTriangle(sourcePoints[va], sourcePoints[vb], sourcePoints[vtemp], targetPoints[va], targetPoints[vb], targetPoints[vtemp])
				+ evaluateNegative(targetPoints[va], targetPoints[vb], targetPoints[vtemp]) / 2;
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

bool getIntersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r)
{
	Point2f x = o2 - o1;
	Point2f d1 = p1 - o1;
	Point2f d2 = p2 - o2;

	float cross = d1.x*d2.y - d1.y*d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}

int APointOnWhichSideOfALineSegment(double x, double y, double xx1, double yy1, double xx2, double yy2)
{
	double judge;
	judge = (double)(y - yy1)*(double)(xx2 - xx1) - (double)(x - xx1)*(double)(yy2 - yy1);
	if (judge > 1.0) {
		return 1;
	}
	if (fabs(judge) <= 1.0) {
		return 0;
	}
	return -1;
}

bool getIntersectPointOfTwoLineSegments(double x0, double y0, double x1, double y1, double xx1, double yy1, double xx2, double yy2, Point2f& p_point)
{
	int a0, a1, a2, a3;
	float a, b, c, d, bb;

	// boundary conditions: if the length of the line is less than 2.
	/*if ((abs(xx1 - xx2) < 2) && (abs(yy1 - yy2) < 2)) {
		return false;
	}
	if ((abs(x0 - x1) < 2) && (abs(y0 - y1) < 2)) {
		return false;
	}*/

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
		bb = b *(double)c - a *(double)d;
		p_point.x = (int)(0.5 + (a *c *(double)(yy1 - y0) + b *c *(double)x0
			- a *d *(double)xx1) / bb);
		p_point.y = (int)(0.5 + (d *b *(double)(x0 - xx1) + b *c *(double)
			yy1 - d *a *(double)y0) / bb);

		if (abs(p_point.x - x0) < 1e-8 && abs(p_point.y - y0) < 1e-8) {
			return true;
		}
		if (abs(p_point.x - x1) < 1e-8 && abs(p_point.y - y1) < 1e-8) {
			return true;
		}
		return true;
	}
	else {
		return false;
	}
}

// templateSkeleton用于约束关键点匹配时不穿越模板点集注册后对应笔画的骨架线。	16.1.14
PointSet findKeyPoints(const PointSet& templatePointSet, const PointSet& dataPointSet, const PointPairSet& templateSkeleton)
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

			/*Mat mat(250, 250, CV_8UC1, Scalar::all(255));
			line(mat, X[i], Y[j], Scalar(0, 0, 0));*/
			for (int k = 0; k < templateSkeleton.size(); k++){
				auto seg = templateSkeleton[k];
				Point l0 = seg.first, l1 = seg.second;
				/*line(mat, l0, l1, Scalar(0, 0, 0));
				imshow("", mat); waitKey();*/
				Point2f crossPnt;
				bool intersect = getIntersectPointOfTwoLineSegments(X[i].x, X[i].y, Y[j].x, Y[j].y, l0.x, l0.y, l1.x, l1.y, crossPnt);
				//bool intersect = getIntersection(X[i], Y[j], l0, l1, crossPnt);
				if (intersect){
					//distX2Y[i][j] = infinity;
					distX2Y[i][j] += 10000;	// 有时候因为骨架线不太好，确实可能出现全都穿越骨架线的，总得让它选一个点吧。	16.1.18
					break;
				}
			}
		}
	}
	PointSet keyPoints;
	int *vertexBelongToTemplate = new int[X.size()];
	for (int i = 0; i < X.size(); i++)	vertexBelongToTemplate[i] = -1;
	//int i = 0, oriI = 0, extraI = 0;
	int i = 0;
	while (i < Y.size()) {	// +=kNumSample表示只匹配关键点，+=1表示匹配所有点
		int minDist = infinity, index = -1;
		for (int j = 0; j < X.size(); j++){
			if (vertexBelongToTemplate[j] < 0){	// 尚未确定该点的对应关系
				int dist = distX2Y[j][i];
				if (dist < minDist){
					minDist = dist;
					index = j;
				}
			}
		}
		if (index == -1) {
			assert(false);	// 不应该出现在这里
			//oriI = i;
			//i += 1;
			//extraI = i - oriI;
			//if (extraI > 3) {
			//	i = oriI + kNumSample;
			//}
			//continue;
		}
		vertexBelongToTemplate[index] = i;
		keyPoints.push_back(X[index]);
		i += kNumSample;
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
#ifdef VERBOSE
			Point p = polygon[pointIndex];
			cout << pointIndex << ": " << p.x << ", " << p.y << endl;
#endif
		}
	}
	return samplePoints;
}

// key points -> sample points.
PointSet getSamplePoints(const PointSet& polygon, PointSet& keyPoints, int numSample)
{
	PointSet samplePoints;
	// 首先将keyPoints排序。因为可能出现本来逆时针，CPD匹配过来后变成顺时针，导致采样全乱的情况。 5.13
	int kpHeadPos = std::find(polygon.begin(), polygon.end(), keyPoints[0]) - polygon.begin();;
	std::sort(keyPoints.begin(), keyPoints.end(), [polygon, kpHeadPos](Point& p1, Point& p2) -> int {
		// 注意这里并不能按绝对的顺序排序，而应该是按相对第一个关键点的顺序，这样才能保证第一个点还是排序后的第一个点。 5.13
		int p1OrderToKpHead = std::find(polygon.begin(), polygon.end(), p1) - polygon.begin() - kpHeadPos;
		int p2OrderToKpHead = std::find(polygon.begin(), polygon.end(), p2) - polygon.begin() - kpHeadPos;
		if (p1OrderToKpHead < 0)	p1OrderToKpHead += polygon.size();
		if (p2OrderToKpHead < 0)	p2OrderToKpHead += polygon.size();
		return p1OrderToKpHead < p2OrderToKpHead;
	});
	for (int i = 0; i < keyPoints.size(); i++){
		int i2 = (i + 1) % keyPoints.size();	// 下一个关键点
		int kp1Order = std::find(polygon.begin(), polygon.end(), keyPoints[i]) - polygon.begin();
		int kp2Order = std::find(polygon.begin(), polygon.end(), keyPoints[i2]) - polygon.begin();
		assert(kp1Order >= 0 && kp2Order >= 0);
#ifdef VERBOSE
		cout << "kp1: " << kp1Order << " kp2: " << kp2Order << endl;
#endif
		int dist = (kp2Order + polygon.size() - kp1Order) % polygon.size();	// 考虑跨越起点的情况
		if (dist == 0)	dist = polygon.size();	// 对只有一个关键点的情况特殊处理 4.16
		double numIntervalPoints = double(dist) / numSample;	// 原来的写法是错的！double(dist / numSample) 5.12
		for (int j = 0; j < numSample; j++){
			int index = floor(fmod(kp1Order + numIntervalPoints * j + polygon.size(), polygon.size()));
			//cout << index << ", ";
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
		Point bak = points[minIdx];
		points[minIdx] = (*it);
		used[minIdx] = true;
		
#ifdef VERBOSE
		cout << "replace corner (" << (*it).x << ", " << (*it).y << ")" << endl;
#endif
	}
	// 删除包含角点可能造成的重复线段，它会导致生成三角网格失败而退出。	16.1.18
	deleteDuplicatedPointsOnContour(points);

	delete[] used;
	delete[] pointOrder;
}

// deprecated. no significant effects.
bool moveSamplePoints(const PointSet& polygon, PointSet& points)
{
	const float sampleGap = kKeyPointsInterval / kNumSample;
	const float minSampleGap = sampleGap * 2 / 3;
	const float endSearchThreshold = sampleGap * 3 / 2;
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
			startPos = endPos;
			continue;
		}
		bool rightFound = false;
		int cend = -1; int maxDist = 0;
		while (!rightFound && endPos < numPoints){
			int tempOrder = endOrder;
			endPos = (endPos + 1 + numPoints) % numPoints;
			endOrder = order[endPos];
			if (endOrder < tempOrder)	return false;	// 确实会出现这种情况！老子不伺候了！
			dist = (endOrder - tempOrder + polygonLength) % polygonLength;
			if (dist >= endSearchThreshold)	rightFound = true;
			if (dist > maxDist){
				maxDist = dist;
				cend = endPos;
			}
			if (startPos == endPos)	break;
		}
		if (!rightFound && startPos == endPos)	break;
		if (!rightFound)	endPos = cend;	// 没找到那么长的，找个差不多的凑合一下吧
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

// mat需要是灰度图，并且骨架为白色
PointPairSet getSkeleton(const Mat& mat) {
	PointSet points;
	for (int i = 0; i < mat.cols; i++) {
		for (int j = 0; j < mat.rows; j++) {
			if (mat.at<uchar>(j, i) > 128) {
				points.push_back(Point(i, j));
			}
		}
	}
	int n = points.size();
	if (n == 0) {
		return PointPairSet();
	}
	// 使用最小生成树得到骨架的段
	vector< pair<Point, Point> > skeleton;
	PointSet processed;
	processed.push_back(points[0]);
	points.erase(points.begin());

	for (int k = 0; k < n - 1; k++) {
		int processedIdx, pointsIdx;
		int minDist = infinity;
		for (int i = 0; i < processed.size(); i++) {
			bool toBreak = false;
			for (int j = 0; j < points.size(); j++) {
				Point& pi = processed[i];
				Point& pj = points[j];
				int dist = abs(pi.x - pj.x) + abs(pi.y - pj.y);	// 直接使用L1距离就够了
				if (dist < minDist) {
					minDist = dist;
					processedIdx = i;
					pointsIdx = j;
					if (dist == 1) {	// 已经是最小距离，不用继续循环了
						toBreak = true;
						break;
					}
				}
			}
			if (toBreak) { break; }
		}
		skeleton.push_back(make_pair(processed[processedIdx], points[pointsIdx]));
		processed.push_back(points[pointsIdx]);
		points.erase(points.begin() + pointsIdx);
	}
	return skeleton;
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

bool has_suffix(const std::string &str, const std::string &suffix)
{
	return str.size() >= suffix.size() &&
		str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}