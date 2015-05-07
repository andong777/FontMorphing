#pragma once

#include "Constant.h"
#include "DataStructure.h"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;
using namespace FM;

Mat getEdge(Mat& mat, PointSet& contour = PointSet());
vector< vector<bool> > getCornerPoints(const Mat& mat, const PointSet& polygon, PointSet& outputCorners = PointSet());
Point findStartPoint(const Mat& mat);
vector< vector<int> > getPolygon(Mat mat, Point start, PointSet& polygon, const PointSet& corners = PointSet());
PointSet getSamplePoints(const PointSet& polygon, int targetPolygonSize = -1, double keyPointsInterval = kKeyPointsInterval, int numSample = kNumSample);
PointSet getSamplePoints(const PointSet& polygon, PointSet& keyPoints, int numSample = kNumSample);
void useCornerPoints(const PointSet& polygon, const PointSet& corners, PointSet& points);
TriMesh getConnectTri(const PointSet& sourcePoints, const PointSet& targetPoints, const vector<int>& strokeEndAtVertex);
PointSet findKeyPoints(const PointSet& templatePointSet, const PointSet& dataPointSet);
bool moveSamplePoints(const PointSet& polygon, PointSet& points);