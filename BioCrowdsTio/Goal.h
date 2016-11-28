#pragma once
class Goal
{
	//public attributes
	public:
		//position
		float posX;
		float posY;
		float posZ;
		//name
		std::string name;

	//public methods
	public:
		Goal();
		Goal(std::string newName, float newPosX, float newPosY, float newPosZ);
		~Goal();
};

