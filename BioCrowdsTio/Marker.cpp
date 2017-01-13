#include "stdafx.h"

Marker::Marker()
{
	//above agent radius, for it need to be high, since will store the min distance
	minDistance = 2;
}

//initialize with its new position
Marker::Marker(float newPosX, float newPosY, float newPosZ)
{
	posX = newPosX;
	posY = newPosY;
	posZ = newPosZ;

	//above agent radius, for it need to be high, since will store the min distance
	minDistance = 2;
}

Marker::~Marker()
{
}

//Reset auxin to his default state, for each update
void Marker::ResetAuxin() {
	SetMinDistance(2);
	taken = false;
	//std::cout << "Reseted - " + std::to_string(posX) + "\n";
}

//GET-SET
float Marker::GetMinDistance() {
	return minDistance;
}
void Marker::SetMinDistance(float minDistance) {
	minDistance = minDistance;
}
