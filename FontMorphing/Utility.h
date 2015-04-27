#pragma once

#include "DataStructure.h"
#include "opencv2/highgui/highgui.hpp"

#define NUM_SAMPLE 6
#define KEY_POINTS_INTERVAL 50

using namespace std;
using namespace cv;
using namespace FM;

Mat getEdge(Mat& mat, PointSet& contour = PointSet());
vector< vector<bool> > getCornerPoints(const Mat& mat, const PointSet& polygon, PointSet& outputCorners = PointSet());
Point findStartPoint(const Mat& mat);
vector< vector<int> > getPolygon(Mat mat, Point start, PointSet& polygon, const PointSet& corners = PointSet());
PointSet getSamplePoints(const PointSet& polygon, int targetPolygonSize = -1, double keyPointsInterval = KEY_POINTS_INTERVAL, int numSample = NUM_SAMPLE);
PointSet getSamplePoints(const PointSet& polygon, PointSet& keyPoints, int numSample = NUM_SAMPLE);
void useCornerPoints(const PointSet& polygon, const PointSet& corners, PointSet& points);
TriMesh getConnectTri(const PointSet& sourcePoints, const PointSet& targetPoints, const vector<int>& strokeEndAtVertex);