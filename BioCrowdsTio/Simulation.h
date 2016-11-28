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
		//obstacle corners
		std::vector<float> verticesObstaclesX;
		std::vector<float> verticesObstaclesY;
		std::vector<float> verticesObstaclesZ;

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
		//all obstacles
		//@TODO: see how to draw the obstacles and interact with them
		//GameObject[] allObstacles;

		/*Signs Part*/
		//signs array
		std::vector<Sign> signs;
		/*End Signs Part*/

	//public methods
	public:
		Simulation();
		Simulation(float mapSizeX, float mapSizeZ);
		~Simulation();

	//private methods
	private:
		void DefaultValues();
		void EndSimulation();
		void LoadChainSimulation();
		void LoadConfigFile();
		void LoadCellsAuxins();
		void DrawGoal(std::string goalName, float goalPositionX, float goalPositionY, float goalPositionZ);
		void DrawSign(float signPositionX, float signPositionY, float signPositionZ, Goal* signGoal, float signAppeal);
		void DrawCells();
		void PlaceAuxins();
		bool CheckObstacle(float checkPositionX, float checkPositionY, float checkPositionZ, std::string tag, float radius);
		float Distance(float x1, float y1, float z1, float x2, float y2, float z2);
		void Split(const std::string &s, char delim, std::vector<std::string> &elems);
		float RandomFloat(float min, float max);
};

