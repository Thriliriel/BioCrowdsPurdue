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
		//how many iterations is agent idle?
		unsigned int idleTimer;
		//how many iterations agent can stay idle?
		unsigned int maxIdleTimer;
		//group index
		int groupIndex;

	//private attributes
	private:
		//list with all auxins in his personal space
		std::vector<Marker*> myAuxins;
		//agent cell
		Cell *cell;
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
		void Caminhe(float tempo);
		void CalculaDirecaoM();
		void CalculaVelocidade();
		void FindNearAuxins(float cellRadius, std::vector<Cell>* allCells, std::vector<Agent>* allAgents, float worldSizeX, float worldSizeZ);
		Cell* GetCell();
		void SetCell(Cell* newCell);
		void AddAuxin(Marker *newAuxin);
		std::vector<Marker*> GetAuxins();
	//private methods
	private:
		float Distance(float x1, float y1, float z1, float x2, float y2, float z2);
		float CalculaW(int indiceRelacao);
		float CalculaF(int indiceRelacao);
		void CheckAuxinsCell(Cell *neighbourCell, std::vector<Agent>* allAgents);
};

