#pragma once
class Marker
{
	//public attributes
	public:
		//is auxin taken?
		bool taken = false;
		//position
		Vector3 position;
		//name
		std::string name;

	//private attributes
	private:
		//min distance from a taken agent
		//when a new agent find it in his personal space, test the distance with this value to see which one is smaller
		float minDistance;
		//agent who took this auxin
		//Agent *agent;

	//public methods
	public:
		Marker();
		Marker(Vector3 newPosition);
		~Marker();
		void ResetAuxin();
		float GetMinDistance();
		void SetMinDistance(float minDistance);
};

