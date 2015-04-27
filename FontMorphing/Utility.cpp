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

float evaluateTriangle(const Point& a, const Point& b, const Point& c)
{
	float va2vb = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + .0);
	float vb2vc = sqrt((b.x - c.x) * (b.x - c.x) + (b.y - c.y) * (b.y - c.y) + .0);
	float va2vc = sqrt((a.x - c.x) * (a.x - c.x) + (a.y - c.y) * (a.y - c.y) + .0);
	
	// negative score
	if (va2vb + va2vc <= vb2vc || va2vb + vb2vc <= va2vc || va2vc + vb2vc <= va2vb){	// 先检查是否构成三角形。 4.25
		return -10;
	}
	float minEdge = min(min(va2vb, vb2vc), va2vc);
	float maxEdge = max(max(va2vb, vb2vc), va2vc);
	if (maxEdge / minEdge >= 10){	// 过于瘦高
		//cout << "too thin" << endl;
		return -5;
	}
	float twoEdges = va2vb + vb2vc + va2vc - maxEdge;
	if ((twoEdges - maxEdge) / maxEdge <= 0.05){		// 过于矮胖	// 是否会出现这种情况？因为有两个点是挨得比较近的。 4.15	// 确实会出现。 4.25
		//cout << "too short" << endl;
		return -5;
	}
	int minLength = 5;
	if (va2vb < minLength && va2vc < minLength && vb2vc < minLength){	// 三条边都小于，认为三角形太小。允许存在一条边小于。 4.26
		//cout << "too small" << endl;
		return -1;
	}
	int maxLength = 100;
	if (va2vb > maxLength || va2vc > maxLength || vb2vc > maxLength){	// 三条边有一条大于，认为三角形太大。 4.27
		// cout << "too big" << endl;
		return -1;
	}
	
	// positive score
	int score = min(va2vb, va2vc) / max(va2vb, va2vc) * 10 + min(va2vb, vb2vc) / max(va2vb, vb2vc) * 10 + min(vb2vc, va2vc) / max(vb2vc, va2vc) * 10;
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
	const int minThreshold = 500;	// 避免两个笔画连在一起导致找到的两点距离太近或重合。 4.27
	for (int i = 0; i < numStroke; i++){
		for (int j = 0; j < numStroke; j++){
			strokeDistance[i][j] = infinity;
			if (i == j)	continue;
			int minIdxA = -1, minIdxB = -1, minDist = infinity;
			for (int m = (i == 0) ? 0 : strokeEndAtVertex[i - 1] + 1; m <= strokeEndAtVertex[i]; m++){
				for (int n = (j == 0) ? 0 : strokeEndAtVertex[j - 1] + 1; n <= strokeEndAtVertex[j]; n++){
					Point diff = sourcePoints[m] - sourcePoints[n];
					int dist = diff.x * diff.x + diff.y * diff.y;
					if (dist < minDist && dist > minThreshold){	// 距离过近的不要
						minDist = dist;
						minIdxA = m;
						minIdxB = n;
					}
				}
			}
			strokeDistance[i][j] = minDist;
			strokeNearPoint[i][j] = Pair(minIdxA, minIdxB);
#ifdef VERBOSE
			cout << "stroke " << i << " to stroke " << j << " min dist: " << minDist << endl;
#endif
		}
	}

	int interval = 5;	// 两个点间的间隔。
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
			int score = evaluateTriangle(sourcePoints[va], sourcePoints[vb], sourcePoints[vtemp]) + evaluateTriangle(targetPoints[va], targetPoints[vb], targetPoints[vtemp]);
			if (score > bestScore){
				bestScore = score;
				bestIdx = candidatePoints[i];
			}
		}
		vc = bestIdx;

		connectTri.push_back(Triangle(va, vb, vc));
		connected[minStrokeIdxA][minStrokeIdxB] = true;
		connected[minStrokeIdxB][minStrokeIdxA] = true;
		ufs->makeUnion(minStrokeIdxA, minStrokeIdxB);

		numPart = ufs->getParents(parts);
		cout << "There are " << numPart << " parts left" << endl;
	} // end while
	cout << "loop " << loop_times_cnt << " times" << endl;

	// 对距离特别近（达到某个阈值）但已经在一个集合中的笔画也建立连接三角形。 4.26
	int before = connectTri.size();
	cout << "extra step of generating connection triangle" << endl;
	int threshold = 5000;	// 平方距离
	int max_extra = numStroke;	// 最多添加多少个
	int extra_cnt = 0;
	auto compare = [strokeDistance](Pair sp1, Pair sp2) { return strokeDistance[sp1.a][sp1.b] > strokeDistance[sp2.a][sp2.b]; };
	priority_queue<Pair, vector<Pair>, decltype(compare)> strokePairs(compare);
	for (int i = 0; i < numStroke; i++){
		for (int j = 0; j < numStroke; j++){
			if (i == j || connected[i][j])	continue;
			if (strokeDistance[i][j] < threshold){
				strokePairs.push(Pair(i, j));
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
			int score = evaluateTriangle(sourcePoints[va], sourcePoints[vb], sourcePoints[vtemp]) + evaluateTriangle(targetPoints[va], targetPoints[vb], targetPoints[vtemp]);
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
//			// replace regular sample points by the corner points
//			for (int p = 1; p <= sampleStep * 1.2 - 1; p++){
//				int index = floor(fmod(basePosition + sampleStep * (j - .6) + p + polygon.size(), polygon.size()));
//				Point pnt = polygon[index];
//				if (cornerFlag[pnt.y][pnt.x]){
//#ifdef VERBOSE
//					cout << "replace corner (" << pnt.x << ", " << pnt.y << ")" << endl;
//#endif
//					cornerFlag[pnt.y][pnt.x] = false;
//					samplePoints.pop_back();
//					samplePoints.push_back(pnt);
//					break;
//				}
//			}
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