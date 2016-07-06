#pragma once
#include "DisplayService.h"
class PointsDisplay : public DisplayService
{
private:
	PointSet points;
	int pointSize;
	bool filled;
	bool oneByOne;

public:
	PointsDisplay(PointSet points, int pointSize = 2, bool filled = false, bool oneByOne = false);
	~PointsDisplay();
	void doDisplay();
};