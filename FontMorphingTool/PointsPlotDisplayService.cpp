#include "PointsPlotDisplayService.h"

PointsPlotDisplayService::PointsPlotDisplayService(PointSet points, int pointSize, bool filled)
{
	this->points = points;
	this->pointSize = pointSize;
	this->filled = filled;
}

PointsPlotDisplayService::~PointsPlotDisplayService()
{
}

void PointsPlotDisplayService::doDisplay()
{
	cout << points.size() << " points in " << title << "." << endl;
	for (int i = 0; i < points.size(); i++){
		circle(canvas, points[i], pointSize, color, filled ? CV_FILLED : 1);
	}
	imshow(title, canvas);
	waitKey();
}
