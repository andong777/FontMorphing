#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "DataStructure.h"

using namespace cv;
using namespace FM;

class CharacterImage
{
private:
	Rect charBox;
	vector<Rect> strokeBox;
	int numStroke;

public:
	Mat charImage;
	std::vector<Mat> strokeImage;
	std::vector<PointPairSet> strokeSkeleton;

	CharacterImage();
	~CharacterImage();

	void setChar(Mat& image, Rect box);
	void addStroke(Mat& image, Rect box);
	void addStrokeSkeleton(PointPairSet& skeleton);	// optional

	Size getCharSize();
	Size getStrokeSize(int no);	// starts from 0, take care.
	int getStrokeLength();
	Point getStrokeOffset(int no);
};