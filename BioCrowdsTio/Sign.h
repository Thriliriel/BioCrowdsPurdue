#pragma once
class Sign
{
	//public attributes
	public:
		//position
		float posX;
		float posY;
		float posZ;

	//private attributes
	private:
		//sign appeal
		float appeal;
		//goal it directs
		Goal *goal;

	//public methods
	public:
		Sign();
		Sign(float newPosX, float newPosY, float newPosZ);
		~Sign();
		float GetAppeal();
		void SetAppeal(float newAppeal);
		Goal* GetGoal();
		void SetGoal(Goal *newGoal);
};

