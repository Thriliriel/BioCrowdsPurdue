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
		//is it looking for?
		bool isLookingFor;
		//if it is a looking for, is it taken?
		bool isTaken;

	//public methods
	public:
		Goal();
		Goal(std::string newName, float newPosX, float newPosY, float newPosZ, bool isLF);
		~Goal();
};

