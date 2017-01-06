#include "stdafx.h"

Agent::Agent()
{
	//set inicial values
	valorDenominadorW = 0;
	denominadorW = false;
	idleTimer = 0;
	maxIdleTimer = 50;
}

//initialize with its new position
Agent::Agent(float newPosX, float newPosY, float newPosZ, std::string newName) {
	posX = newPosX;
	posY = newPosY;
	posZ = newPosZ;
	name = newName;

	//set inicial values
	valorDenominadorW = 0;
	denominadorW = false;
	maxSpeed = 1.5f;
	agentRadius = 1;
	speedX = 0;
	speedY = 0;
	speedZ = 0;
	fieldOfView = 5;
	idleTimer = 0;
	maxIdleTimer = 50;
}

Agent::~Agent()
{
	//iterate all cell auxins to check distance between auxins and agent
	for (int i = 0; i < myAuxins.size(); i++)
	{
		myAuxins[i]->ResetAuxin();
	}
	myAuxins.clear();
}

void Agent::Start() {
	/*goalX = go[0]->posX;
	goalY = go[0]->posY;
	goalZ = go[0]->posZ;*/
	goalX = pathX[0];
	goalY = go[0]->posY;
	goalZ = pathZ[0];
	diffX = goalX - posX;
	diffY = goalY - posY;
	diffZ = goalZ - posZ;
	diffMod = Distance(diffX, diffY, diffZ, 0, 0, 0);
	gX = diffX / diffMod;
	gY = diffY / diffMod;
	gZ = diffZ / diffMod;
	//path = new NavMeshPath();
}

void Agent::Update(std::vector<Sign>* allSigns) {
	//clear agent큦 informations
	ClearAgent();

	//update his goal
	/*goalX = go[0]->posX;
	goalY = go[0]->posY;
	goalZ = go[0]->posZ;
	goalX = pathX[0];
	goalY = go[0]->posY;
	goalZ = pathZ[0];
	diffX = goalX - posX;
	diffY = goalY - posY;
	diffZ = goalZ - posZ;
	diffMod = Distance(diffX, diffY, diffZ, 0, 0, 0);
	gX = diffX / diffMod;
	gY = diffY / diffMod;
	gZ = diffZ / diffMod;*/

	//check interaction with possible signs
	CheckSignsInView(allSigns);	
}

//clear agent큦 informations
void Agent::ClearAgent()
{
	//re-set inicial values
	valorDenominadorW = 0;
	vetorDistRelacaoMarcacaoX.clear();
	vetorDistRelacaoMarcacaoY.clear();
	vetorDistRelacaoMarcacaoZ.clear();
	denominadorW = false;
	mX = mY = mZ = 0;
	diffX = goalX - posX;
	diffY = goalY - posY;
	diffZ = goalZ - posZ;
	diffMod = Distance(diffX, diffY, diffZ, 0, 0, 0);
	gX = diffX / diffMod;
	gY = diffY / diffMod;
	gZ = diffZ / diffMod;

	changePosition = true;
}

//check if there is a sign in the agent Field of View
void Agent::CheckSignsInView(std::vector<Sign>* allSigns) {
	//get all signs on scene
	//for each one of them, check the distance between it and the agent
	bool reorder = false;
	for(int s = 0; s < allSigns->size(); s++){
		float distance = Distance(posX, posY, posZ, (*allSigns)[s].posX, (*allSigns)[s].posY, (*allSigns)[s].posZ);
		//if distance <= agent field of view, the sign may affect the agent
		if (distance <= fieldOfView)
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

	//reorder our goals
	if (reorder)
	{
		ReorderGoals();
	}
}

//make the interaction between the agent and the signs he can see
void Agent::Interaction(Sign *sign, float distance, int index)
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

//walk
void Agent::Caminhe(float tempo)
{
	//std::cout << name << ": SpeedX - " << speedX << " -- SpeedZ - " << speedZ << "-- Tempo: " << tempo << "\n";
	posX += speedX*tempo;
	posY += speedY*tempo;
	posZ += speedZ*tempo;
	/*posX += speedX;
	posY += speedY;
	posZ += speedZ;*/
	//std::cout << name << ": PosX - " << posX << " -- PosZ - " << posZ << "-- Tempo: " << tempo << "\n";
}

//The calculation formula starts here
//the ideia is to find m=SUM[k=1 to n](Wk*Dk)
//where k iterates between 1 and n (number of auxins), Dk is the vector to the k auxin and Wk is the weight of k auxin
//the weight (Wk) is based on the degree resulting between the goal vector and the auxin vector (Dk), and the
//distance of the auxin from the agent
void Agent::CalculaDirecaoM()
{
	//if (name == "agent0") std::cout << vetorDistRelacaoMarcacaoX.size() << " -- " << vetorDistRelacaoMarcacaoZ.size() << "\n";
	//for each agent큦 auxin
	for (int k = 0; k < vetorDistRelacaoMarcacaoX.size(); k++)
	{
		//calculate W
		float valorW = CalculaW(k);
		if (valorDenominadorW < 0.0001)
			valorW = 0.0f;

		//sum the resulting vector * weight (Wk*Dk)
		mX += valorW * vetorDistRelacaoMarcacaoX[k] * maxSpeed;
		mY += valorW * vetorDistRelacaoMarcacaoY[k] * maxSpeed;
		mZ += valorW * vetorDistRelacaoMarcacaoZ[k] * maxSpeed;
	}
}

//calculate W
float Agent::CalculaW(int indiceRelacao)
{
	//if (name == "agent0") std::cout << vetorDistRelacaoMarcacaoX.size() << " -- " << vetorDistRelacaoMarcacaoX.size() << "\n";
	//calculate F (F is part of weight formula)
	float valorF = CalculaF(indiceRelacao);

	if (!denominadorW)
	{
		valorDenominadorW = 0;

		//for each agent큦 auxin
		for (int k = 0; k < vetorDistRelacaoMarcacaoX.size(); k++)
		{
			//calculate F for this k index, and sum up
			valorDenominadorW += CalculaF(k);
		}
		denominadorW = true;
	}

	float retorno = valorF / valorDenominadorW;
	return retorno;
}

//calculate F (F is part of weight formula)
float Agent::CalculaF(int indiceRelacao)
{
	//if (name == "agent0") std::cout << vetorDistRelacaoMarcacaoX.size() << " -- " << vetorDistRelacaoMarcacaoX.size() << "\n";
	//distance between auxin큦 distance and origin (dont know why origin...)
	float moduloY = Distance(vetorDistRelacaoMarcacaoX[indiceRelacao], vetorDistRelacaoMarcacaoY[indiceRelacao], vetorDistRelacaoMarcacaoZ[indiceRelacao], 
		0, 0, 0);
	//distance between goal vector and origin (dont know why origin...)
	float moduloX = Distance(gX, gY, gZ, 0, 0, 0);
	//vector * vector
	float produtoEscalar = vetorDistRelacaoMarcacaoX[indiceRelacao] * gX + vetorDistRelacaoMarcacaoY[indiceRelacao] * gY + 
		vetorDistRelacaoMarcacaoZ[indiceRelacao] * gZ;

	if (moduloY < 0.00001)
	{
		return 0.0f;
	}

	//return the formula, defined in tesis/paper
	float retorno = (float)((1.0 / (1.0 + moduloY)) * (1.0 + ((produtoEscalar) / (moduloX * moduloY))));
	return retorno;
}

//calculate speed vector    
void Agent::CalculaVelocidade()
{
	//distance between movement vector and origin
	float moduloM = Distance(mX, mY, mZ, 0, 0, 0);
	//if (name == "agent0") std::cout << mX << " -- " << mZ << "\n";
	//multiply for PI
	//float s = moduloM * 3.14f;
	float s = moduloM;

	//if it is bigger than maxSpeed, use maxSpeed instead
	if (s > maxSpeed)
		s = maxSpeed;
	
	if (moduloM > 0.0001)
	{
		//calculate speed vector
		speedX = s * (mX / moduloM);
		speedY = s * (mY / moduloM);
		speedZ = s * (mZ / moduloM);
	}
	else
	{
		//else, he is idle
		speedX = 0;
		speedY = 0;
		speedZ = 0;
	}
}

//find all auxins near him (Voronoi Diagram)
//call this method from game controller, to make it sequential for each agent
void Agent::FindNearAuxins(float cellRadius, std::vector<Cell>* allCells, std::vector<Agent>* allAgents, float worldSizeX, float worldSizeZ) {
	//clear them all, for obvious reasons
	myAuxins.clear();
	//std::cout << cell->name << "\n";
	//check all auxins on agent cell
	CheckAuxinsCell(cell, allAgents);

	//find all neighbours cells
	int startX = (int)(cell->posX - (cellRadius * 2));
	int startZ = (int)(cell->posZ - (cellRadius * 2));
	int endX = (int)(cell->posX + (cellRadius * 2));
	int endZ = (int)(cell->posZ + (cellRadius * 2));
	//distance from agent to cell, to define agent new cell
	float distanceToCell = Distance(posX, posY, posZ, cell->posX, cell->posY, cell->posZ);
	//iterate to find the cells
	//2 in 2, since the radius of each cell is 1 = diameter 2
	for (float i = startX; i <= endX; i = i + (cellRadius * 2))
	{
		for (float j = startZ; j <= endZ; j = j + (cellRadius * 2))
		{
			//if it is out of the world, continue
			if (i >= worldSizeX || j >= worldSizeZ) continue;

			//THIS WAY IS FAR SLOWER, SO USE THE CRAZY FORMULA XD
			/*float nameX = i - cellRadius;
			float nameZ = j - cellRadius;

			//find the cell
			int indCell = -1;
			//iterate through all cells to find this neighbour
			for (int c = 0; c < allCells->size(); c++) {
				if ((*allCells)[c].name == "cell" + std::to_string(nameX) + "-" + std::to_string(nameZ)) {
					indCell = c;
					//found, can break
					break;
				}
			}*/

			//crazy formula to get the right index of the neighbour cell
			int indCell = (((i - cellRadius) / (cellRadius * 2)) * (worldSizeZ / (cellRadius * 2))) + ((j - cellRadius) / (cellRadius * 2));

			//if it exists..
			//if (indCell >= 0)
			if (indCell < allCells->size())
			{
				Cell *neighbourCell = &(*allCells)[indCell];

				//check all auxins on this cell
				CheckAuxinsCell(neighbourCell, allAgents);

				//see distance to this cell
				//if it is lower, the agent is in another(neighbour) cell
				float distanceToNeighbourCell = Distance(posX, posY, posZ, neighbourCell->posX, neighbourCell->posY, neighbourCell->posZ);
				if (distanceToNeighbourCell < distanceToCell)
				{
					distanceToCell = distanceToNeighbourCell;
					SetCell(neighbourCell);
				}
			}
		}
	}
}

//check auxins on a cell
void Agent::CheckAuxinsCell(Cell *neighbourCell, std::vector<Agent>* allAgents)
{
	//if (name == "agent0") std::cout << neighbourCell->name << " -- " << allAgents->size() << "\n";
	//get all auxins on my cell
	std::vector<Marker>* cellAuxins = neighbourCell->GetAuxins();

	//iterate all cell auxins to check distance between auxins and agent
	for (int i = 0; i < cellAuxins->size(); i++)
	{
		//see if the distance between this agent and this auxin is smaller than the actual value, and inside agent radius
		float distance = Distance(posX, posY, posZ, (*cellAuxins)[i].posX, (*cellAuxins)[i].posY, (*cellAuxins)[i].posZ);
		if (distance < (*cellAuxins)[i].GetMinDistance() && distance <= agentRadius)
		{
			//take the auxin!!
			//if this auxin already was taken, need to remove it from the agent who had it
			if ((*cellAuxins)[i].taken == true)
			{
				//need to find out which agent has it
				bool exitFirstLoop = false;
				for (int a = 0; a < allAgents->size(); a++) {
					std::vector<Marker*> otherAgentMarkers = (*allAgents)[a].GetAuxins();
					for (int m = 0; m < otherAgentMarkers.size(); m++) {
						//if found, good, take it out and break
						if (otherAgentMarkers[m]->name == (*cellAuxins)[i].name) {
							//if (m == 0) std::cout << "Marker: " << otherAgentMarkers[m]->name << "\n";
							(*allAgents)[a].myAuxins.erase((*allAgents)[a].myAuxins.begin() + m);
							exitFirstLoop = true;
							break;
						}
					}
					if (exitFirstLoop) break;
				}
			}

			//auxin is taken
			(*cellAuxins)[i].taken = true;
			//auxin has agent
			//(*cellAuxins)[i].SetAgent(this.gameObject);
			//update min distance
			(*cellAuxins)[i].SetMinDistance(distance);
			//update my auxins
			AddAuxin(&(*cellAuxins)[i]);
		}
	}
	//if (name == "agent0") std::cout << myAuxins.size() << "\n";
}

//reorder goals/intentions
void Agent::ReorderGoals() {
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
}

//change agent path when he arrives at his next path node
void Agent::ChangePath() {
	//if distance to node center is less than 0.75 (max distance to the center =~ 0,7071)
	float dist = Distance(goalX, goalY, goalZ, posX, posY, posZ);
	if (dist < 0.75f)
	{
		if (!pathX.empty()) {
			pathX.erase(pathX.begin());
			pathZ.erase(pathZ.begin());
		}

		//update
		//if it is empty, already at last node. So, set the actual goal
		if (pathX.empty()) {
			goalX = go[0]->posX;
			goalY = go[0]->posY;
			goalZ = go[0]->posZ;
		}//else, next node
		else {
			goalX = pathX[0];
			goalY = go[0]->posY;
			goalZ = pathZ[0];
		}
	}
}

//GET-SET
Cell* Agent::GetCell()
{
	return cell;
}
void Agent::SetCell(Cell* newCell)
{
	cell = newCell;
}

//add a new auxin on myAuxins
void Agent::AddAuxin(Marker *newAuxin)
{
	myAuxins.push_back(newAuxin);
}
//return all auxins in this cell
std::vector<Marker*> Agent::GetAuxins()
{
	return myAuxins;
}

//add a new desire on desire
void Agent::AddDesire(float newDesire)
{
	desire.push_back(newDesire);
}
//remove a desire on desire
void Agent::RemoveDesire(int index)
{
	desire.erase(desire.begin() + index);
}
//distance between 2 points
float Agent::Distance(float x1, float y1, float z1, float x2, float y2, float z2)
{
	float result = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (z2 - z1)*(z2 - z1));

	return result;
}