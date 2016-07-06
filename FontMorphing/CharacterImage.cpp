#include "CharacterImage.h"
#include <iostream>
#include <algorithm>

CharacterImage::CharacterImage()
{
	numStroke = 0;
}

CharacterImage::~CharacterImage()
{
}

void CharacterImage::setChar(Mat& image, Rect box)
{
	charImage = image;
	charBox = box;
}

void CharacterImage::addStroke(Mat& image, Rect box)
{
	strokeImage.push_back(image);
	strokeBox.push_back(box);
	numStroke++;
}

void CharacterImage::addStrokeSkeleton(PointPairSet& skeleton)
{
	strokeSkeleton.push_back(skeleton);
}

Size CharacterImage::getCharSize()
{
	return charBox.size();
}

Size CharacterImage::getStrokeSize(int no)
{
	return strokeBox.at(no).size();
}

Point CharacterImage::getStrokeOffset(int no)
{
	return strokeBox.at(no).tl() - charBox.tl();
}

int CharacterImage::getStrokeLength()
{
	return numStroke;
}