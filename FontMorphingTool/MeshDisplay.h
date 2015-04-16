#pragma once
#include "DisplayService.h"
#include "DataStructure.h"
#define SHOW_DETAILS
using namespace FM;

class MeshDisplay : public DisplayService
{
private:
	TriMesh mesh;
	PointSet points;

public:
	MeshDisplay(TriMesh mesh, PointSet points);
	~MeshDisplay();
	void doDisplay();
};

