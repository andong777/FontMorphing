#pragma once

#include "DataStructure.h"
#include "opencv2/highgui/highgui.hpp"

using namespace FM;

class MorphingService
{
protected:
	PointSet source;
	PointSet target;
	TriMesh mesh;
	TriMesh connectTri;

public:
	virtual ~MorphingService() {}
	virtual void setMorphing(PointSet& source, PointSet& target, TriMesh& mesh, TriMesh& connectTri)
	{
		this->source = source;
		this->target = target;
		this->mesh = mesh;
		this->connectTri = connectTri;
	}
	virtual void doMorphing(float outputRatio, int numStep) = 0;
	virtual PointSet getPointSetByStepIndex(int stepIndex) = 0;
};