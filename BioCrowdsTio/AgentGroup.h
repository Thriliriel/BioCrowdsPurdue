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
		std::vector<float> pathX;
		std::vector<float> pathZ;
		//max speed
		float maxSpeed;

	//public methods
	public:
		AgentGroup();
		AgentGroup(Vector3 newPosition, Cell *newCell, float newMaxSpeed);
		~AgentGroup();
		bool ReorderGoals();
		bool CheckSignsInView(std::vector<Sign>* allSigns);
		void ChangePath();

	//private methods
	private:
		float Distance(float x1, float y1, float z1, float x2, float y2, float z2);
		void Interaction(Sign *sign, float distance, int index);
};
