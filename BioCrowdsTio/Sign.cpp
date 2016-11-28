#include "stdafx.h"

Sign::Sign()
{
}

Sign::Sign(float newPosX, float newPosY, float newPosZ) {
	posX = newPosX;
	posY = newPosY;
	posZ = newPosZ;
}

Sign::~Sign()
{
}

//GETs and SETs
float Sign::GetAppeal() {
	return appeal;
}
void Sign::SetAppeal(float newAppeal) {
	appeal = newAppeal;
}
Goal* Sign::GetGoal()
{
	return goal;
}
void Sign::SetGoal(Goal *newGoal)
{
	goal = newGoal;
}