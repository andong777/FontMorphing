#pragma once
class CharacterImage
{
private:
	int numStroke;
	int charBox[4];
	int (*strokeBox)[4];

public:
	CharacterImage();
	CharacterImage(int numStroke);
	~CharacterImage();

	void setNumOfStroke(int numStroke);
	void setCharBox(int box[4]);
	void setStrokeBox(int no, int box[4]);	// starts from 0, take care.

	int getCharHeight();
	int getCharWidth();
	int getStrokeHeight(int no);
	int getStrokeWidth(int no);
	int getStrokeOffsetX(int no);
	int getStrokeOffsetY(int no);
};