#pragma once
class Goal
{
	//public attributes
	public:
		//position
		Vector3 position;
		//name
		std::string name;
		//is it looking for?
		bool isLookingFor;
		//if it is a looking for, is it taken?
		bool isTaken;

	//public methods
	public:
		Goal();
		Goal(std::string newName, Vector3 newPosition, bool isLF);
		~Goal();
};

