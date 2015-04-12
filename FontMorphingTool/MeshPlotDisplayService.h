#pragma once
#include "DisplayService.h"
#include "DataStructure.h"
#define SHOW_DETAILS
using namespace FM;

class MeshPlotDisplayService : public DisplayService
{
private:
	TriMesh mesh;
	PointSet points;

public:
	MeshPlotDisplayService(TriMesh mesh, PointSet points);
	~MeshPlotDisplayService();
	void doDisplay();
};

