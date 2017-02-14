#include "stdafx.h"

Goal::Goal()
{
	isLookingFor = false;
	isTaken = false;
}

Goal::Goal(std::string newName, Vector3 newPosition, bool isLF = false) {
	name = newName;
	position = newPosition;
	isLookingFor = isLF;
	isTaken = false;
}

Goal::~Goal()
{
}
