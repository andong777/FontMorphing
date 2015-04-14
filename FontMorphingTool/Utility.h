#pragma once

#include "DataStructure.h"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;
using namespace FM;

Mat fillHoles(Mat& imInput);
vector< vector<bool> > getCorners(const Mat& mat, PointSet& outputCorners = PointSet());
Point findStartPoint(const Mat& mat);
vector< vector<int> > getPolygon(Mat mat, Point start, PointSet& polygon);
TriMesh getConnectTri(const PointSet& pointSet, const vector<int>& strokeEndAtVertex);