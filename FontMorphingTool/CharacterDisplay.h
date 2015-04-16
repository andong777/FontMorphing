#pragma once
#include "DisplayService.h"
#include "CharacterImage.h"

class CharacterDisplay : public DisplayService
{
private:
	CharacterImage character;

public:
	CharacterDisplay(CharacterImage& image);
	~CharacterDisplay();
	void doDisplay();
};