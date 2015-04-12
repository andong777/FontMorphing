#include "PointsPlotDisplayService.h"

PointsPlotDisplayService::PointsPlotDisplayService(PointSet points)
{
	this->points = points;
}


PointsPlotDisplayService::~PointsPlotDisplayService()
{
}

void PointsPlotDisplayService::doDisplay()
{
	cout << points.size() << " points in " << title << "." << endl;
	for (int i = 0; i < points.size(); i++){
		circle(canvas, points[i], 2, Scalar(0, 0, 0));
	}
	imshow(title, canvas);
	waitKey();
}
