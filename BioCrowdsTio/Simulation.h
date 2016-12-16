#pragma once
class Simulation
{
	//public attributes
	public:
		//scenario X
		float scenarioSizeX;
		//scenario Z
		float scenarioSizeZ;
		//agent radius
		float agentRadius;
		//cell radius
		float cellRadius;
		//qnt of agents in the scene
		int qntAgents;
		//qnt of signs in the scene (PROBALLY USELESS SO FAR)
		int qntSigns;
		//qnt of goals in the scene (PROBALLY USELESS SO FAR)
		int qntGoals;
		//radius for auxin collide
		float auxinRadius;
		//save config file?
		bool saveConfigFile;
		//load config file?
		bool loadConfigFile;
		//all simulation files directory
		std::string allSimulations;
		//config filename
		std::string configFilename;
		//obstacles filename
		std::string obstaclesFilename;
		//agents and schedule filename
		std::string scheduleFilename;
		//exit filename
		std::string exitFilename;
		//signs filename
		std::string signsFilename;
		//goals filename
		std::string goalsFilename;
		//exit agents/goal filename
		std::string agentsGoalFilename;
		//clock starter timer
		clock_t startTime;
		//simulation delta time
		double simulationTime;
		//xml for visualisation
		std::string exitXml;
		//"frame" count (Estevão said it needs to start at 24, for reasons...)
		unsigned int frameCount;

	//private attributes
	private:
		//max agent spawn x
		int spawnPositionX;
		//max agent spawn z
		int spawnPositionZ;
		//auxins density
		float PORC_QTD_Marcacoes;
		//qnt of auxins on the ground
		int qntAuxins;
		//exit file
		std::ofstream exitFile;
		//exit agents/goal file
		std::ofstream agentsGoalFile;
		//all config directories
		std::vector<std::string> allDirs;
		//simulation index
		int simulationIndex;
		//stop all sims
		bool gameOver;
		//agents array
		std::vector<Agent> agents;
		//cells array
		std::vector<Cell> cells;
		//goals array
		std::vector<Goal> goals;
		//last frame count
		int lastFrameCount;
		//intention threshold
		float intentionThreshold;
		//Simulation time step
		int fps;
		//all obstacles vertices
		//X vertices
		std::vector<float> polygonX;
		//Z vertices
		std::vector<float> polygonZ;
		//constant for InsideObstacle calculus
		std::vector<float> constant;
		//multiple for InsideObstacle calculus
		std::vector<float> multiple;
		//how much is the obstacle far away from the world origin
		float obstacleDisplacement;
		//do we plot the scene?
		bool plot;

		/*Signs Part*/
		//signs array
		std::vector<Sign> signs;
		//obstacle corners
		std::vector<float> verticesObstaclesX;
		std::vector<float> verticesObstaclesY;
		std::vector<float> verticesObstaclesZ;
		//just for 1 obstacle so far
		std::vector<int> trianglesObstacle;
		/*End Signs Part*/

	//public methods
	public:
		Simulation();
		Simulation(float mapSizeX, float mapSizeZ, float newCellRadius, int argcp, char **argv);
		~Simulation();
		void Update(double elapsed);

	//private methods
	private:
		void DefaultValues();
		void StartSimulation();
		void EndSimulation();
		void LoadChainSimulation();
		void LoadConfigFile();
		void LoadCellsAuxins();
		void DrawGoal(std::string goalName, float goalPositionX, float goalPositionY, float goalPositionZ, bool isLF);
		void DrawSign(float signPositionX, float signPositionY, float signPositionZ, Goal* signGoal, float signAppeal);
		void DrawCells();
		void PlaceAuxins();
		void PlaceAuxinsAsGrid();
		void SaveConfigFile();
		void SaveExitFile();
		void SaveAgentsGoalFile(std::string agentName, std::string goalName);
		void DrawObstacles();
		void DrawObstacle(std::vector<float> verticesX, std::vector<float> verticesY, std::vector<float> verticesZ, std::vector<int> triangles);
		void CheckGroupVertices();
		void ReadOBJFile();
		void GenerateLookingFor();
		void ChangeLookingFor(Goal* changeLF);
		float Distance(float x1, float y1, float z1, float x2, float y2, float z2);
		void Split(const std::string &s, char delim, std::vector<std::string> &elems);
		float RandomFloat(float min, float max);
		bool Contains(std::vector<float> arrayToSearch, float needle);
		bool Contains(std::vector<float> arrayToSearch, std::vector<float> arrayToSearch2, float needle, float needle2);
		void PreCalcValues();
		bool InsideObstacle(float pX, float pY, float pZ);
		void UnlockAgent(Agent* agentToUnlock);
};

