#include "stdafx.h"
#include "Goal.h"


Goal::Goal()
{
}

Goal::Goal(std::string newName, float newPosX, float newPosY, float newPosZ) {
	name = newName;
	posX = newPosX;
	posY = newPosY;
	posZ = newPosZ;
}

Goal::~Goal()
{
}
