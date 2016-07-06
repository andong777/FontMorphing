#pragma once

#include "MorphingService.h"
#include "DisplayService.h"

using namespace FM;

class ARAPMorphing : public MorphingService, public DisplayService
{
private:
	std::vector<PointSet> pointSets;
	string charName;

public:
	ARAPMorphing(string charName = "");
	~ARAPMorphing();
	void setMorphing(PointSet& source, PointSet& target, TriMesh& mesh, TriMesh& connectTri);
	void doMorphing(float outputRatio = .5f, int numStep = 10);
	void doDisplay();
	PointSet getPointSetByStepIndex(int stepIndex);
};