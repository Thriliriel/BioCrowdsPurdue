#pragma once
class Obstacle
{
	//public attributes
	public:
		//vertices
		std::vector<float> verticesX;
		std::vector<float> verticesY;
		std::vector<float> verticesZ;
		//triangles
		std::vector<int> triangles;
		//name
		std::string name;

	//public methods
	public:
		Obstacle();
		Obstacle(std::string newName);
		Obstacle(std::string newName, std::vector<float> newVerticesX, std::vector<float> newVerticesY, std::vector<float> newVerticesZ, std::vector<int> newTriangles);
		~Obstacle();
};

