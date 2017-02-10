#include "stdafx.h"

Hofstede::Hofstede()
{
	meanAngVar = meanCohesion = meanDist = meanSpeed = 0;
	numGroups = 1;
}

Hofstede::Hofstede(int newNumGroups)
{
	meanAngVar = meanCohesion = meanDist = meanSpeed = 0;
	numGroups = newNumGroups;
}

Hofstede::~Hofstede()
{
}

//calculate the hofstede values
void Hofstede::CalculateHofstede(int pdi, int mas, int lto, int ing) {
	meanDist = ((100 - pdi) * (1.2f) / 100) * numGroups;
	meanCohesion = (((100 - mas) * 3) / 100) * numGroups;
	meanAngVar = ((100 - lto) / 100) * numGroups;
	meanSpeed = (ing * 1.4f / 100) * numGroups;
}

//Getters and Setters
float Hofstede::GetMeanDist() {
	return meanDist;
}
void Hofstede::SetMeanDist(float value) {
	meanDist = value;
}
float Hofstede::GetMeanCohesion() {
	return meanCohesion;
}
void Hofstede::SetMeanCohesion(float value) {
	meanCohesion = value;
}
float Hofstede::GetMeanAngVar() {
	return meanAngVar;
}
void Hofstede::SetMeanAngVar(float value) {
	meanAngVar = value;
}
float Hofstede::GetMeanSpeed() {
	return meanSpeed;
}
void Hofstede::SetMeanSpeed(float value) {
	meanSpeed = value;
}