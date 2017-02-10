#pragma once
class Hofstede
{
	//public attributes
	public:
		//number of groups
		int numGroups;

	//private attributes
	//Hofstede values
	private:
		float meanDist, meanCohesion, meanAngVar, meanSpeed;

	//public methods
	public:
		Hofstede();
		Hofstede(int newNumGroups);
		~Hofstede();
		void CalculateHofstede(int pdi, int mas, int lto, int ing);
		float GetMeanDist();
		void SetMeanDist(float value);
		float GetMeanCohesion();
		void SetMeanCohesion(float value);
		float GetMeanAngVar();
		void SetMeanAngVar(float value);
		float GetMeanSpeed();
		void SetMeanSpeed(float value);
};

