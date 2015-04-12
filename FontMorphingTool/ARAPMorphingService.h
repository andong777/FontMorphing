#pragma once

#include "MorphingService.h"
#include "DisplayService.h"

using namespace FM;

class ARAPMorphingService : public MorphingService, public DisplayService
{
private:
	std::vector<PointSet> pointSets;
	PointSet source;
	PointSet target;
	TriMesh mesh;

public:
	ARAPMorphingService();
	~ARAPMorphingService();
	void doMorphing(PointSet source, PointSet target, TriMesh mesh, int numStep);
	void doDisplay();
	PointSet getPointSetByStepIndex(int stepIndex);
};

