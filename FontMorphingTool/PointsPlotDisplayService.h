#pragma once
#include "DisplayService.h"
class PointsPlotDisplayService : public DisplayService
{
private:
	PointSet points;
	int pointSize;
	bool filled;

public:
	PointsPlotDisplayService(PointSet points, int pointSize = 2, bool filled = false);
	~PointsPlotDisplayService();
	void doDisplay();
};

