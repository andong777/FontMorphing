#pragma once

#include "MorphingService.h"
#include "DisplayService.h"

using namespace FM;

class ARAPMorphing : public MorphingService, public DisplayService
{
private:
	std::vector<PointSet> pointSets;
	PointSet source;
	PointSet target;
	TriMesh mesh;

public:
	ARAPMorphing();
	~ARAPMorphing();
	void doMorphing(PointSet source, PointSet target, TriMesh mesh, int numStep);
	void doDisplay();
	PointSet getPointSetByStepIndex(int stepIndex);
};

