#pragma once
class Agent
{
	//public attributes
	public:
		//agent radius
		float agentRadius;
		//position
		Vector3 position;
		//agent speed
		Vector3 speed;
		//name
		std::string name;
		//max speed
		float maxSpeed;
		//auxins distance vector from agent
		std::vector<Vector3> vetorDistRelacaoMarcacao;
		//field of view, to see signs
		float fieldOfView;
		//to avoid locks
		bool changePosition = true;
		//goal position
		Vector3 goal;
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
		// to calculate var m (blocks recalculation)
		bool denominadorW = false; 
		// to calculate var m (blocks recalculation)
		float valorDenominadorW;
		//orientation vector (movement)	
		Vector3 m; 
		//diff between goal and agent
		Vector3 diff; 
		//diff module
		float diffMod; 
		//goal vector (diff / diffMod)
		Vector3 g; 

	//public methods
	public:
		Agent();
		Agent(Vector3 newPosition, std::string newName);
		~Agent();
		void Start();
		void Update(std::vector<Sign>* allSigns);
		void ClearAgent();
		void Caminhe(float time);
		void CalculaDirecaoM();
		void CalculaVelocidade(Vector3 groupCenter, float cohesion, float time);
		void FindNearAuxins(float cellRadius, std::vector<Cell>* allCells, std::vector<Agent>* allAgents, float worldSizeX, float worldSizeZ);
		Cell* GetCell();
		void SetCell(Cell* newCell);
		void AddAuxin(Marker *newAuxin);
		std::vector<Marker*> GetAuxins();
	//private methods
	private:
		float CalculaW(int indiceRelacao);
		float CalculaF(int indiceRelacao);
		void CheckAuxinsCell(Cell *neighbourCell, std::vector<Agent>* allAgents);
};

