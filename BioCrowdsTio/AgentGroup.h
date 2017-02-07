#pragma once
class AgentGroup
{
	//public attributes
	public:
		//max group speed
		float maxSpeed;
		//goals schedule
		std::vector<Goal*> go;

	//private attributes

	//public methods
	public:
		AgentGroup();
		~AgentGroup();

	//private methods
};
