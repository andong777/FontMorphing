#pragma once
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

class CharacterImage
{
private:
	Rect charBox;
	vector<Rect> strokeBox;
	int numStroke;

public:
	Mat charImage;
	std::vector<Mat> strokeImage;

	CharacterImage();
	~CharacterImage();

	void setChar(Mat& image, Rect box);
	void addStroke(Mat& image, Rect box);

	Size getCharSize();
	Size getStrokeSize(int no);	// starts from 0, take care.
	int getStrokeLength();
	Point getStrokeOffset(int no);
};