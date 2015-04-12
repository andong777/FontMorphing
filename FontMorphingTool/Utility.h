#pragma once

#include "DataStructure.h"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

Mat fillHoles(Mat imInput);
vector< vector<bool> > getCorners(const Mat& mat);
Point findStartPoint(const Mat& mat);
vector< vector<int> > getPolygon(Mat mat, Point start, FM::PointSet& polygon);

