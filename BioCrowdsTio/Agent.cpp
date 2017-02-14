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
Agent::Agent(Vector3 newPosition, std::string newName) {
	position = newPosition;
	name = newName;

	//set inicial values
	valorDenominadorW = 0;
	denominadorW = false;
	maxSpeed = 1.5f;
	agentRadius = 1;
	speed.x = 0;
	speed.y = 0;
	speed.z = 0;
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
	Vector3 zero(0, 0, 0);
	diff.x = goal.x - position.x;
	diff.y = goal.y - position.y;
	diff.z = goal.z - position.z;
	diffMod = Simulation::Distance(diff, zero);
	g.x = diff.x / diffMod;
	g.y = diff.y / diffMod;
	g.z = diff.z / diffMod;
}

void Agent::Update(std::vector<Sign>* allSigns) {
	//clear agent�s informations
	ClearAgent();
}

//clear agent�s informations
void Agent::ClearAgent()
{
	Vector3 zero(0, 0, 0);
	//re-set inicial values
	valorDenominadorW = 0;
	vetorDistRelacaoMarcacao.clear();
	denominadorW = false;
	m.x = m.y = m.z = 0;
	diff.x = goal.x - position.x;
	diff.y = goal.y - position.y;
	diff.z = goal.z - position.z;
	diffMod = Simulation::Distance(diff, zero);
	g.x = diff.x / diffMod;
	g.y = diff.y / diffMod;
	g.z = diff.z / diffMod;

	changePosition = true;
}

//walk
void Agent::Caminhe(float time)
{
	//std::cout << name << ": SpeedX - " << speedX << " -- SpeedZ - " << speedZ << "-- Time: " << time << "\n";
	position.x += speed.x * time;
	position.y += speed.y * time;
	position.z += speed.z * time;
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
	//for each agent�s auxin
	for (int k = 0; k < vetorDistRelacaoMarcacao.size(); k++)
	{
		//calculate W
		float valorW = CalculaW(k);
		if (valorDenominadorW < 0.0001)
			valorW = 0.0f;

		//sum the resulting vector * weight (Wk*Dk)
		m.x += valorW * vetorDistRelacaoMarcacao[k].x * maxSpeed;
		m.y += valorW * vetorDistRelacaoMarcacao[k].y * maxSpeed;
		m.z += valorW * vetorDistRelacaoMarcacao[k].z * maxSpeed;
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

		//for each agent�s auxin
		for (int k = 0; k < vetorDistRelacaoMarcacao.size(); k++)
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
	//distance between auxin�s distance and origin (dont know why origin...)
	float moduloY = Simulation::Distance(vetorDistRelacaoMarcacao[indiceRelacao], Vector3(0, 0, 0));
	//distance between goal vector and origin (dont know why origin...)
	float moduloX = Simulation::Distance(g, Vector3(0, 0, 0));
	//vector * vector
	float produtoEscalar = vetorDistRelacaoMarcacao[indiceRelacao].x * g.x + vetorDistRelacaoMarcacao[indiceRelacao].y * g.y + 
		vetorDistRelacaoMarcacao[indiceRelacao].z * g.z;

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
	Vector3 zero(0, 0, 0);
	//distance between movement vector and origin
	float moduloM = Simulation::Distance(m, zero);
	//if (name == "agent0") std::cout << mX << " -- " << mZ << "\n";
	//multiply for PI
	//float s = moduloM * 3.14f;
	float s = moduloM;
	float thisMaxSpeed = maxSpeed;

	//actual distance from group center position
	float actualDistance = Simulation::Distance(position, zero);
	//movement prediction distance
	Vector3 prediction;
	prediction.x = position.x + ((s * (m.x / moduloM))*time);
	prediction.y = position.y + ((s * (m.y / moduloM))*time);
	prediction.z = position.z + ((s * (m.z / moduloM))*time);
	float movementPredctionDistance = Simulation::Distance(prediction, groupCenter);
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
		speed.x = s * (m.x / moduloM);
		speed.y = s * (m.y / moduloM);
		speed.z = s * (m.z / moduloM);
	}
	else
	{
		//else, he is idle
		speed = zero;
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
	int startX = (int)(cell->position.x - (cellRadius * 2));
	int startZ = (int)(cell->position.z - (cellRadius * 2));
	int endX = (int)(cell->position.x + (cellRadius * 2));
	int endZ = (int)(cell->position.z + (cellRadius * 2));
	//distance from agent to cell, to define agent new cell
	float distanceToCell = Simulation::Distance(position, cell->position);
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
				float distanceToNeighbourCell = Simulation::Distance(position, neighbourCell->position);
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
		float distance = Simulation::Distance(position, (*cellAuxins)[i].position);
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