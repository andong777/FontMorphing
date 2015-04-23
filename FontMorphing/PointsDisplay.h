#pragma once
#include "DisplayService.h"
class PointsDisplay : public DisplayService
{
private:
	PointSet points;
	int pointSize;
	bool filled;

public:
	PointsDisplay(PointSet points, int pointSize = 2, bool filled = false);
	~PointsDisplay();
	void doDisplay();
};