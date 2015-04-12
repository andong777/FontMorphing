#include "CharacterImage.h"
#include <iostream>
#include <algorithm>

CharacterImage::CharacterImage()
{
	strokeBox = NULL;
}

CharacterImage::CharacterImage(int numStroke)
{
	setNumOfStroke(numStroke);
}

CharacterImage::~CharacterImage()
{
	delete strokeBox;
}

void CharacterImage::setNumOfStroke(int numStroke)
{
	delete strokeBox;
	this->numStroke = numStroke;
	strokeBox = new int[numStroke][4];
}

void CharacterImage::setCharBox(int box[4])
{
	std::copy(box, box + 4, charBox);
}

void CharacterImage::setStrokeBox(int no, int box[4])
{
	std::copy(box, box + 4, strokeBox[no]);
}

int CharacterImage::getCharHeight()
{
	return charBox[3] - charBox[1];
}

int CharacterImage::getCharWidth()
{
	return charBox[2] - charBox[0];
}

int CharacterImage::getStrokeHeight(int no)
{
	return strokeBox[no][3] - strokeBox[no][1];
}

int CharacterImage::getStrokeWidth(int no)
{
	return strokeBox[no][2] - strokeBox[no][0];
}

int CharacterImage::getStrokeOffsetX(int no)
{
	return strokeBox[no][0] - charBox[0];
}

int CharacterImage::getStrokeOffsetY(int no)
{
	return strokeBox[no][1] - charBox[1];
}
