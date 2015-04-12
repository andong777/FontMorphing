#pragma once

#include "DataStructure.h"
#include "opencv2/highgui/highgui.hpp"

using namespace FM;

class MorphingService
{
public:
	virtual ~MorphingService() {}
	virtual void doMorphing(PointSet source, PointSet target, TriMesh mesh, int numStep) = 0;
	virtual PointSet getPointSetByStepIndex(int stepIndex) = 0;
};

