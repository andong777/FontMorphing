#pragma once

#include "DataStructure.h"
#include "opencv2/highgui/highgui.hpp"

#define NUM_SAMPLE 8

using namespace std;
using namespace cv;
using namespace FM;

Mat getEdge(Mat& mat);
vector< vector<bool> > getCorners(const Mat& mat, PointSet& outputCorners = PointSet());
Point findStartPoint(const Mat& mat);
vector< vector<int> > getPolygon(Mat mat, Point start, PointSet& polygon);
PointSet getSamplePoints(const PointSet& polygon, const vector< vector<bool> >& cornerFlag, double keyPointsInterval = 100., int numSample = NUM_SAMPLE);
PointSet getSamplePoints(const PointSet& polygon, const vector< vector<bool> >& cornerFlag, const PointSet& keyPoints, const vector< vector<int> >& orderAtThisPoint, int numSample = NUM_SAMPLE);
TriMesh getConnectTri(const PointSet& pointSet, const vector<int>& strokeEndAtVertex);