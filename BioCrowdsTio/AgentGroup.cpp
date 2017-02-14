#include "stdafx.h"

AgentGroup::AgentGroup()
{
}

AgentGroup::AgentGroup(Vector3 newPosition, Cell *newCell, float newMaxSpeed)
{
	position = newPosition;
	cell = newCell;
	maxSpeed = newMaxSpeed;
}

AgentGroup::~AgentGroup()
{
}

//reorder goals/intentions. Returns bool to know if first goal has changed
bool AgentGroup::ReorderGoals() {
	//store the first goal, to see if changed later
	std::string firstGoal = go[0]->name;

	for (int i = 0; i < intentions.size(); i++)
	{
		for (int j = i + 1; j < intentions.size(); j++)
		{
			//if j element is bigger, change
			if (intentions[i] < intentions[j])
			{
				//reorder intentions
				float temp = intentions[j];
				intentions[j] = intentions[i];
				intentions[i] = temp;

				//reorder desires
				float tempD = desire[j];
				desire[j] = desire[i];
				desire[i] = tempD;

				//reorder goals
				Goal *tempG = go[j];
				go[j] = go[i];
				go[i] = tempG;
			}
		}
	}

	//now, we check if the first goal changed. If so, we need to re-calculate path later
	if (go[0]->name != firstGoal) {
		return true;
	}
	else {
		return false;
	}
}

//check if there is a sign in the agent Field of View
bool AgentGroup::CheckSignsInView(std::vector<Sign>* allSigns) {
	//get all signs on scene
	//for each one of them, check the distance between it and the agent
	bool reorder = false;
	//for each agent in this group
	for (int a = 0; a < agents.size(); a++) {
		for (int s = 0; s < allSigns->size(); s++) {
			float distance = Simulation::Distance(agents[a].position, (*allSigns)[s].position);
			//if distance <= agent field of view, the sign may affect the agent
			if (distance <= agents[a].fieldOfView)
			{
				//now, lets see if this sign is from a goal that our agent has intention to go
				for (int i = 0; i < go.size(); i++)
				{
					if (go[i]->name == (*allSigns)[s].GetGoal()->name) {
						//well, lets do the interaction
						Interaction(&(*allSigns)[s], distance, i);
						reorder = true;
						break;
					}
				}
			}
		}
	}

	//reorder our goals
	if (reorder)
	{
		return ReorderGoals();
	}
	else {
		return false;
	}
}

//make the interaction between the agent and the signs he can see
void AgentGroup::Interaction(Sign *sign, float distance, int index)
{
	float deltaIntention;

	//alfa -> interaction environment
	float alfa;
	alfa = 1.0f / distance;
	if (alfa > 1)
		alfa = 1;

	// From the model in Bosse et al 2014 limited for 2 agents
	//float Sq = (intentions[index]);
	//sign intention will be always 1
	float Sq = 1;

	float gama = desire[index] * alfa * sign->GetAppeal();

	deltaIntention = gama * (Sq - intentions[index]);

	intentions[index] = intentions[index] + deltaIntention;
	//std::cout << name << " com sign " << sign->GetGoal()->name << ": " << std::to_string(deltaIntention) << "\n"; system("PAUSE");
}

//change agent path when he arrives at his next path node
void AgentGroup::ChangePath() {
	//if distance to node center is less than value
	float dist = 20;
	//check the distance of each agent
	for (int i = 0; i < agents.size(); i++) {
		float hisDist = Simulation::Distance(agents[i].goal, agents[i].position);
		if (hisDist < dist) {
			dist = hisDist;
		}
	}
	
	if (dist < 0.5f)
	{
		if (!path.empty()) {
			path.erase(path.begin());
		}

		//update
		//if it is empty, already at last node. So, set the actual goal
		Vector3 newGoal(0, 0, 0);
		if (path.empty()) {
			newGoal = go[0]->position;
		}//else, next node
		else {
			newGoal = path[0];
		}

		//update all agents of the group
		for (int i = 0; i < agents.size(); i++) {
			agents[i].goal = newGoal;
		}
	}
}

//Getters and Setters
float AgentGroup::GetMaxSpeed() {
	return maxSpeed;
}
void AgentGroup::SetMaxSpeed(float value) {
	maxSpeed = value;
}
Hofstede AgentGroup::GetHofstede() {
	return hofstede;
}
void AgentGroup::SetHofstede(Hofstede value) {
	hofstede = value;
}