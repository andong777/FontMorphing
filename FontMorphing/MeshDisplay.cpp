#include "MeshDisplay.h"


MeshDisplay::MeshDisplay(TriMesh mesh, PointSet points, bool filled)
{
	this->mesh = mesh;
	this->points = points;
	this->filled = filled;
}


MeshDisplay::~MeshDisplay()
{
}

void MeshDisplay::doDisplay()
{
	for (int j = 0; j < mesh.size(); j++){
		if (filled){
			Point gon[1][3];
			gon[0][0] = points[mesh[j].va];
			gon[0][1] = points[mesh[j].vb];
			gon[0][2] = points[mesh[j].vc];
			const Point* ppt[1] = { gon[0] }; int npt[] = { 3 };
			fillPoly(canvas, ppt, npt, 1, color);
		}
		else{
			int a = mesh[j].va;
			int b = mesh[j].vb;
			int c = mesh[j].vc;
			line(canvas, points[a], points[b], color);
			line(canvas, points[b], points[c], color);
			line(canvas, points[c], points[a], color);
		}
	}
	if (toScreen){
		imshow(title, canvas); waitKey();
	}
}
