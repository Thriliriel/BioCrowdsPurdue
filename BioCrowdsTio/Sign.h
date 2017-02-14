#pragma once
class Sign
{
	//public attributes
	public:
		//position
		Vector3 position;

	//private attributes
	private:
		//sign appeal
		float appeal;
		//goal it directs
		Goal *goal;

	//public methods
	public:
		Sign();
		Sign(Vector3 newPosition);
		~Sign();
		float GetAppeal();
		void SetAppeal(float newAppeal);
		Goal* GetGoal();
		void SetGoal(Goal *newGoal);
};

