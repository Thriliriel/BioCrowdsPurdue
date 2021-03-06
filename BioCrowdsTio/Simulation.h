#pragma once
class Simulation
{
	//public attributes
	public:
		//world size
		Vector3 worldSize;
		//agent radius
		float agentRadius;
		//cell radius
		float cellRadius;
		//qnt of agents in the scene
		int qntAgents;
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

	//private attributes
	private:
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
		//now, we have groups of agents. So, need to have te groups array with agents inside it
		//std::vector<Agent> agents;
		std::vector<AgentGroup> agentsGroups;
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
		//how much is the obstacle far away from the world origin
		float obstacleDisplacement;
		//what is the obstacle scale
		float obstacleScale;
		//do we plot the scene?
		bool plot;
		//draw scale
		int drawScale;
		//default node size
		float nodeSize;
		//graph nodes for A*
		std::vector<int> graphNodes;
		//since we are using the triangles as our path, need to know their positions
		std::vector<Node> graphNodesPos;
		//signs array
		std::vector<Sign> signs;
		//quantity of groups that agents will form. If it is zero, means no groups will be used (like groups = false), therefore, each agent will be alone in a group
		int qntGroups;
		//using A*?
		bool useAStar;
		//using Hofstede?
		bool useHofstede;

		//all obstacles
		std::vector<std::vector<Vector3>> obstacles;
		std::vector<std::vector<int>> allTriangles;

	//public methods
	public:
		Simulation();
		Simulation(Vector3 newWorldSize, float newCellRadius, int argcp, char **argv);
		~Simulation();
		void Update(double elapsed);
		static float Distance(Vector3 start, Vector3 end);

	//private methods
	private:
		void DefaultValues();
		void StartSimulation(int argcp, char **argv);
		void EndSimulation();
		void LoadChainSimulation();
		void LoadConfigFile();
		void LoadCellsAuxins();
		void LoadObstacleFile();
		void DrawGoal(std::string goalName, Vector3 goalPosition, bool isLF);
		void DrawSign(Vector3 signPosition, Goal* signGoal, float signAppeal);
		void DrawCells();
		void PlaceAuxins();
		void PlaceAuxinsAsGrid();
		void SaveConfigFile();
		void SaveExitFile();
		void SaveAgentsGoalFile(std::string agentName, std::string goalName);
		void DrawObstacles();
		void DrawObstacle(std::vector<Vector3> vertices, std::vector<int> triangles);
		void CheckGroupVertices();
		void ReadOBJFile();
		void GenerateLookingFor();
		Vector3 GeneratePosition(int groupIndex, bool useCenter);
		void ChangeLookingFor(Goal* changeLF);
		void Split(const std::string &s, char delim, std::vector<std::string> &elems);
		float RandomFloat(float min, float max);
		bool Contains(std::vector<float> arrayToSearch, float needle);
		bool Contains(std::vector<float> arrayToSearch, std::vector<float> arrayToSearch2, float needle, float needle2);
		bool InsideObstacle(Vector3 p);
		void UnlockAgent(Agent* agentToUnlock);
		void AStarPath(AgentGroup* agentPath);
		void CalculateMeanPoints(std::vector<Triangle>* triangles);
};

