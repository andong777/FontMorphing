#include "PointsDisplay.h"

PointsDisplay::PointsDisplay(PointSet points, int pointSize, bool filled)
{
	this->points = points;
	this->pointSize = pointSize;
	this->filled = filled;
}

PointsDisplay::~PointsDisplay()
{
}

void PointsDisplay::doDisplay()
{
	for (int i = 0; i < points.size(); i++){
		circle(canvas, points[i], pointSize, color, filled ? CV_FILLED : 1);
	}
	if (toScreen){
		imshow(title, canvas); waitKey();
	}
}