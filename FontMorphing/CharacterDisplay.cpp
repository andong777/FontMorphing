#include "CharacterDisplay.h"


CharacterDisplay::CharacterDisplay(CharacterImage& image)
{
	character = image;
}


CharacterDisplay::~CharacterDisplay()
{
}


void CharacterDisplay::doDisplay()
{
	vector<Mat>& strokeImage = character.strokeImage;
	for (int no = 0; no < strokeImage.size(); no++){
		Point offset = character.getStrokeOffset(no) + Point(5, 5);
		Mat& strokeImg = character.strokeImage.at(no);
		for (int h = 0; h < strokeImg.rows; h++){
			for (int w = 0; w < strokeImg.cols; w++){
				uchar v = strokeImg.at<uchar>(h, w);
				if (v < 128){	// if black
					canvas.at<Vec3b>(h + offset.y, w + offset.x) = Vec3b(0, 0, 0);
				}
			}
		}
	}
	if (toScreen){
		imshow(title, canvas); waitKey();
	}
}