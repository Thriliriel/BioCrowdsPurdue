#include "stdafx.h"

Goal::Goal()
{
	isLookingFor = false;
	isTaken = false;
}

Goal::Goal(std::string newName, float newPosX, float newPosY, float newPosZ, bool isLF = false) {
	name = newName;
	posX = newPosX;
	posY = newPosY;
	posZ = newPosZ;
	isLookingFor = isLF;
	isTaken = false;
}

Goal::~Goal()
{
}
