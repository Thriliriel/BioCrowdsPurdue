#include "stdafx.h"
#include "Cell.h"

Cell::Cell()
{
	//default: can pass
	canPass = 1;
}

//initialize with its new position
Cell::Cell(float newPosX, float newPosY, float newPosZ, std::string newName)
{
	//default: can pass
	canPass = 1;

	posX = newPosX;
	posY = newPosY;
	posZ = newPosZ;
	name = newName;
}


Cell::~Cell()
{
	//clear all auxins
	myAuxins.clear();
}


void Cell::StartList()
{
	//myAuxins = new std::vector<Marker>();
}

//add a new auxin on myAuxins
void Cell::AddAuxin(Marker auxin)
{
	myAuxins.push_back(auxin);
}

//return all auxins in this cell
std::vector<Marker>* Cell::GetAuxins() {
	return &myAuxins;
}
