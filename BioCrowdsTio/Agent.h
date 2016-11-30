#pragma once
class Agent
{
	//public attributes
	public:
		//agent radius
		float agentRadius;
		//position
		float posX;
		float posY;
		float posZ;
		//agent speed
		float speedX;
		float speedY;
		float speedZ;
		//name
		std::string name;
		//max speed
		float maxSpeed;
		//goals schedule
		std::vector<Goal*> go;
		//goals intentions
		std::vector<float> intentions;
		//auxins distance vector from agent
		std::vector<float> vetorDistRelacaoMarcacaoX;
		std::vector<float> vetorDistRelacaoMarcacaoY;
		std::vector<float> vetorDistRelacaoMarcacaoZ;
		//field of view, to see signs
		float fieldOfView;
		//to avoid locks
		bool changePosition = true;
		//goal position
		float goalX;
		float goalY;
		float goalZ;
		//looking for position
		float lookinForX;
		float lookinForY;
		float lookinForZ;

	//private attributes
	private:
		//list with all auxins in his personal space
		std::vector<Marker*> myAuxins;
		//agent cell
		Cell *cell;
		//path
		//@TODO: como colocar o A*???
		//private NavMeshPath path;
		//goals desire
		std::vector<float> desire;
		/*
		START: Copied from Paravisi´s model
		*/
		bool denominadorW = false; // to calculate var m (blocks recalculation)
		float valorDenominadorW;	// to calculate var m (blocks recalculation)
		float mX; //orientation vector x (movement)
		float mY; //orientation vector y (movement)
		float mZ; //orientation vector z (movement)		
		float diffX; //diff x between goal and agent
		float diffY; //diff y between goal and agent
		float diffZ; //diff z between goal and agent
		float diffMod; //diff module
		float gX; //goal vector x (diff / diffMod)
		float gY; //goal vector y (diff / diffMod)
		float gZ; //goal vector z (diff / diffMod)
		/*
		FINISH: Copied from Paravisi´s model
		*/

	//public methods
	public:
		Agent();
		Agent(float newPosX, float newPosY, float newPosZ, std::string newName);
		~Agent();
		void Start();
		void Update(std::vector<Sign>* allSigns);
		void ClearAgent();
		void ReorderGoals();
		void Caminhe(float tempo);
		void CalculaDirecaoM();
		void CalculaVelocidade();
		void FindNearAuxins(float cellRadius, std::vector<Cell>* allCells, std::vector<Agent>* allAgents);
		Cell* GetCell();
		void SetCell(Cell* newCell);
		void AddAuxin(Marker *newAuxin);
		std::vector<Marker*> GetAuxins();
		void AddDesire(float newDesire);
		void RemoveDesire(int index);
	//private methods
	private:
		float Distance(float x1, float y1, float z1, float x2, float y2, float z2);
		void CheckSignsInView(std::vector<Sign>* allSigns);
		void Interaction(Sign *sign, float distance, int index);
		float CalculaW(int indiceRelacao);
		float CalculaF(int indiceRelacao);
		void CheckAuxinsCell(Cell *neighbourCell, std::vector<Agent>* allAgents);
};

