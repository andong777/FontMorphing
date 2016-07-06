#pragma once

#include "DataStructure.h"
#include "opencv2/highgui/highgui.hpp"

using namespace FM;
using namespace cv;

class DisplayService
{
protected:
	bool toScreen;	// ÊÇ·ñÊä³öµ½ÆÁÄ»
	string title;
	Size size;
	Mat canvas;
	Scalar color;

public:
	virtual ~DisplayService() {}

	virtual void setDisplay(bool toScreen, string title = "", Size size = Size(), Mat canvas = Mat(500, 500, CV_8UC3, Scalar(255, 255, 255)), Scalar color = Scalar(0, 0, 0)){
		this->toScreen = toScreen;
		this->title = title;
		this->size = size;
		if (size.height > 0 && size.width > 0){
			this->canvas = Mat(size, CV_8UC3, Scalar(255, 255, 255));
		}
		else{
			this->canvas = canvas;
		}
		this->color = color;
	}

	virtual void doDisplay() = 0;
};

