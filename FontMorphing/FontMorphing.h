#pragma once

#include "Constant.h"
#include "DataStructure.h"
#include "CharacterImage.h"
#include "opencv2/highgui/highgui.hpp"

using namespace FM;
using namespace cv;
using namespace std;
using namespace SGCPD;
using namespace GEOM_FADE2D;
using namespace Eigen;

class FontMorphing
{
private:
	string charName;
	CharacterImage sourceChar;
	CharacterImage targetChar;
	TriMesh triMesh;
	TriMesh connectTri;
	vector<int> strokeEndAtVertex;
	PointSet sourceCharVert;
	PointSet targetCharVert;

	bool toScreen;
	Mat sourceCanvas;
	Mat targetCanvas;

	void readCharactersData();
	void getTemplateFromSourceCharacter();
	void registerPointSetToTargetCharacter();

public:
	FontMorphing(string charName);
	~FontMorphing();
	bool work(bool toScreen = false);

};