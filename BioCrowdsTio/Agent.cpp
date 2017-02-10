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
	fieldOfView = 10;
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

//walk
void Agent::Caminhe(float time)
{
	//std::cout << name << ": SpeedX - " << speedX << " -- SpeedZ - " << speedZ << "-- Time: " << time << "\n";
	posX += speedX*time;
	posY += speedY*time;
	posZ += speedZ*time;
	/*posX += speedX;
	posY += speedY;
	posZ += speedZ;*/
	//std::cout << name << ": PosX - " << posX << " -- PosZ - " << posZ << "-- Time: " << time << "\n";
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
void Agent::CalculaVelocidade(Vector3 groupCenter, float cohesion, float time)
{
	//distance between movement vector and origin
	float moduloM = Distance(mX, mY, mZ, 0, 0, 0);
	//if (name == "agent0") std::cout << mX << " -- " << mZ << "\n";
	//multiply for PI
	//float s = moduloM * 3.14f;
	float s = moduloM;
	float thisMaxSpeed = maxSpeed;

	//actual distance from group center position
	float actualDistance = Distance(posX, posY, posZ, groupCenter.x, groupCenter.y, groupCenter.z);
	//movement prediction distance
	float movementPredctionDistance = Distance(posX + ((s * (mX / moduloM))*time), posY + ((s * (mY / moduloM))*time), posZ + ((s * (mZ / moduloM))*time), groupCenter.x, groupCenter.y, groupCenter.z);
	//test the movement prediction. If the agent will be too far from group center position, reduce his maxSpeed
	if (movementPredctionDistance > 3 - cohesion && movementPredctionDistance > actualDistance) {
		thisMaxSpeed /= 5;
	}

	//if it is bigger than maxSpeed, use maxSpeed instead
	if (s > thisMaxSpeed)
		s = thisMaxSpeed;
	
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
			//formula x -> y
			//int indCell = (((i - cellRadius) / (cellRadius * 2)) * (worldSizeZ / (cellRadius * 2))) + ((j - cellRadius) / (cellRadius * 2));
			//formula y -> x
			int indCell = (((j - cellRadius) / (cellRadius * 2)) * (worldSizeX / (cellRadius * 2))) + ((i - cellRadius) / (cellRadius * 2));

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
//distance between 2 points
float Agent::Distance(float x1, float y1, float z1, float x2, float y2, float z2)
{
	float result = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (z2 - z1)*(z2 - z1));

	return result;
}