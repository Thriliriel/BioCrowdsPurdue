#include "stdafx.h"

Vector3::Vector3()
{
	x = y = z = 0;
}

Vector3::Vector3(float newX, float newY, float newZ)
{
	x = newX;
	y = newY;
	z = newZ;
}

Vector3::~Vector3()
{
}