#pragma once
class AgentGroup
{
	//public attributes
	public:
		//group center position
		Vector3 position;
		//group cell
		Cell *cell;
		//agents of this group
		std::vector<Agent> agents;
		//goals schedule
		std::vector<Goal*> go;
		//goals intentions
		std::vector<float> intentions;
		//goals desire
		std::vector<float> desire;
		//agents A* path
		std::vector<Vector3> path;

	//private attributes
	private:
		//max speed
		float maxSpeed;
		//Hofstede values
		Hofstede hofstede;

	//public methods
	public:
		AgentGroup();
		AgentGroup(Vector3 newPosition, Cell *newCell, float newMaxSpeed);
		~AgentGroup();
		bool ReorderGoals();
		bool CheckSignsInView(std::vector<Sign>* allSigns);
		void ChangePath();
		float GetMaxSpeed();
		void SetMaxSpeed(float value);
		Hofstede GetHofstede();
		void SetHofstede(Hofstede value);

	//private methods
	private:
		void Interaction(Sign *sign, float distance, int index);
};
