#include "stdafx.h"

Obstacle::Obstacle()
{
}

Obstacle::Obstacle(std::string newName) {
	name = newName;
}

Obstacle::Obstacle(std::string newName, std::vector<float> newVerticesX, std::vector<float> newVerticesY, std::vector<float> newVerticesZ, std::vector<int> newTriangles) {
	name = newName;
	verticesX = newVerticesX;
	verticesY = newVerticesY;
	verticesZ = newVerticesZ;
	triangles = newTriangles;
}

Obstacle::~Obstacle()
{
}
