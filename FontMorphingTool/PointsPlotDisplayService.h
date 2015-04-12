#pragma once
#include "DisplayService.h"
class PointsPlotDisplayService : public DisplayService
{
private:
	PointSet points;

public:
	PointsPlotDisplayService(PointSet points);
	~PointsPlotDisplayService();
	void doDisplay();
};

