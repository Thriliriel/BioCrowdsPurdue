#include "stdafx.h"

Sign::Sign()
{
}

Sign::Sign(Vector3 newPosition) {
	position = newPosition;
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