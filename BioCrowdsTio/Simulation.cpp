#include "stdafx.h"

Simulation::Simulation()
{
	//start with default values
	DefaultValues();
}

Simulation::~Simulation()
{
	exitFile.close();
	agentsGoalFile.close();
}

Simulation::Simulation(float mapSizeX, float mapSizeZ) {
	//start with default values
	DefaultValues();

	std::cout << "STARTING TO DEPLOY!!\n";

	scenarioSizeX = mapSizeX;
	scenarioSizeZ = mapSizeZ;

	//get all subdirectories within the defined config directory
	_finddata_t data;
	std::string path = allSimulations + "*.*";
	intptr_t ff = _findfirst(path.c_str(), &data);
	if (ff != -1)
	{
		int res = 0;
		while (res != -1)
		{
			if ((data.attrib & _A_SUBDIR) == _A_SUBDIR)
			{
				//does not include the "go back" directory
				if (data.name != "." && data.name != "..") {
					allDirs.push_back(data.name);
				}
			}
			res = _findnext(ff, &data);
		}
		_findclose(ff);
	}
	//delete the first 2 indexes, because they are the "./" and "../"
	allDirs.erase(allDirs.begin());
	allDirs.erase(allDirs.begin());
	/*for (int i = 0; i < allDirs.size(); i++) {
		std::cout << "\n" << allDirs[i];
	}*/

	//Get map size        
	spawnPositionX = (int)round(scenarioSizeX - 1);
	//spawnPositionZ = (int)round(scenarioSizeZ - 1));//anywhere
	spawnPositionZ = (int)round(scenarioSizeZ - (scenarioSizeZ - 2));//to make agents start near the bottom

	//if loadConfigFile is checked, we do not generate the initial scenario. We load it from the Config.csv file, as well from other defined files
	if (loadConfigFile)
	{
		//draw obstacles, auxins, cells and goals
		LoadCellsAuxins();

		//check group vertices
		CheckGroupVertices();

		//start from the first one
		LoadChainSimulation();
	}
	else
	{
		//open exit file to save info each frame
		exitFile.open(exitFilename);
		if (!exitFile.is_open()) {
			std::cout << "Could not open exit file: " + exitFilename + "!\n";
		}

		//open exit agents/goal file to save info
		agentsGoalFile.open(agentsGoalFilename);
		if (!agentsGoalFile.is_open()) {
			std::cout << "Could not open agents goal file: " + agentsGoalFilename + "!\n";
		}

		//draw the obstacles
		//DrawObstacles();
		ReadOBJFile();

		//check group vertices
		CheckGroupVertices();

		//precalc values to check obstacles
		PreCalcValues();
		//testing...
		/*if (InsideObstacle(10.0f, 0.0f, 10.001f)) {
			std::cout << "Inside!!\n";
		}
		else {
			std::cout << "Outside!!\n";
		}*/
		//std::cout << "Qnt Obstacles: " << obstacles.size() << "\n";
		//std::cout << "Vertice: " << obstacles[0].verticesX[0] << "\n";
		DrawCells();
		PlaceAuxins();
		std::cout << "Qnt Cells: " << cells.size() << "\n";

		//instantiante some goals
		DrawGoal("Restaurant", 3, 0, 18, false);
		DrawGoal("Theater", 27, 0, 17, false);
		DrawGoal("Stadium", 5, 0, 3, false);
		DrawGoal("AppleStore", 25, 0, 5, false);

		//generate all looking for states
		GenerateLookingFor();

		//instantiate the goal's signs
		for (int p = 0; p < goals.size(); p++) {
			if (!goals[p].isLookingFor) {
				DrawSign(goals[p].posX, goals[p].posY, goals[p].posZ, &goals[p], 1);
			}
		}

		//instantiante some signs
		DrawSign(15, 0, 10, &goals[0], 0.7);
		DrawSign(23, 0, 11, &goals[2], 0.9);

		//std::cout << goals.size() << " -- " << signs.size();

		//to avoid a freeze
		int doNotFreeze = 0;
		//instantiate qntAgents Agents
		for (int i = 0; i < qntAgents; i++)
		{
			//if we are not finding space to set the agent, lets update the maxZ position to try again
			if (doNotFreeze > qntAgents) {
				doNotFreeze = 0;
				spawnPositionZ += 2;
			}

			//sort out a cell
			int cellIndex = (int)round(RandomFloat(0, cells.size() - 1));

			//generate the agent position
			float x = RandomFloat(cells[cellIndex].posX - cellRadius, cells[cellIndex].posX + cellRadius);
			float z = RandomFloat(cells[cellIndex].posZ - cellRadius, cells[cellIndex].posZ + cellRadius);

			//see if there are agents in this radius. if not, instantiante
			bool pCollider = false;
			for (int j = 0; j < agents.size(); j++) {
				if (Distance(x, 0, z, agents[j].posX, agents[j].posY, agents[j].posZ) < 0.5f) {
					pCollider = true;
					break;
				}
			}

			//even so, if we are an obstacle, cannot instantiate either
			//just need to check for obstacle if found no player, otherwise it will not be instantiated anyway
			if (!pCollider) {
				pCollider = InsideObstacle(x, 0, z);
			}

			//if found a player in the radius, do not instantiante. try again
			if (pCollider)
			{
				//try again
				i--;
				doNotFreeze++;
				continue;
			}
			else
			{
				Agent newAgent(x, 0, z, "agent" + std::to_string(i));
				//agent cell
				newAgent.SetCell(&cells[cellIndex]);
				//agent radius
				newAgent.agentRadius = agentRadius;

				//agent goals
				for (int j = 0; j < goals.size(); j++)
				{
					//if goal is not looking for...
					if (!goals[j].isLookingFor) {
						//add a goal
						newAgent.go.push_back(&goals[j]);
						//add a random intention
						newAgent.intentions.push_back(RandomFloat(0, (intentionThreshold - 0.01)));
						//add a random desire
						newAgent.AddDesire(RandomFloat(0, 1));
					}
				}

				//get the first non taken looking for state
				for (int j = 0; j < goals.size(); j++)
				{
					//if goal is looking for and is free...
					if (goals[j].isLookingFor && !goals[j].isTaken) {
						//add a goal
						newAgent.go.push_back(&goals[j]);
						//add intention 0.8
						newAgent.intentions.push_back(0.8);
						//add desire 1
						newAgent.AddDesire(1);
						//this looking for goal is taken now
						goals[j].isTaken = true;
						//already have one, get out!
						break;
					}
				}

				//reorder following intentions
				newAgent.ReorderGoals();

				//start default values
				newAgent.Start();

				agents.push_back(newAgent);
			}
		}
	}
	for (int i = 0; i < agents.size(); i++) {
		std::cout << agents[i].name << ": PosX - " << agents[i].posX << " -- PosZ - " << agents[i].posZ << "\n";
	}
	/*for (int i = 0; i < agents[0].go.size(); i++) {
		std::cout << agents[0].go[i]->name << ": PosX - " << agents[0].go[i]->posX << " -- PosZ - " << agents[0].go[i]->posZ << " -- Intention: " << agents[0].intentions[i] << "\n";
	}*/
	system("PAUSE");

	/*for (int p = 0; p < signs.size(); p++) {
		std::cout << signs[p].GetGoal()->name << "\n";
	}*/

	//all ready to go. If saveConfigFile is checked, save this config in a csv file
	if (saveConfigFile)
	{
		std::cout << "SAVING CONFIG FILE!!\n";

		Simulation::SaveConfigFile();
	}

	std::cout << "STARTING TO RUN!!\n";

	//initiate both timers
	startTime = clock();
	simulationTime = ((double)clock() / CLOCKS_PER_SEC);
	//std::cout << (double)clock() / CLOCKS_PER_SEC << " -- " << simulationTime << "\n";

	//start
	StartSimulation();
}

void Simulation::DefaultValues() {
	//scenario X
	scenarioSizeX = 30;
	//scenario Z
	scenarioSizeZ = 20;
	//agent radius
	agentRadius = 1;
	//cell radius
	cellRadius = 1;
	//qnt of agents in the scene
	qntAgents = 10;
	//qnt of signs in the scene
	qntSigns = 0;
	//qnt of goals in the scene
	qntGoals = 1;
	//radius for auxin collide
	auxinRadius = 0.1;
	//save config file?
	saveConfigFile = false;
	//load config file?
	loadConfigFile = false;
	//all simulation files directory
	allSimulations = "Simulations/";
	//config filename
	configFilename = allSimulations + "Config.csv";
	//obstacles filename (probally useless since it is bringing the Tharindu´s file)
	obstaclesFilename = allSimulations + "Obstacles.csv";
	//agents and schedule filename
	scheduleFilename = allSimulations + "sign_0/agents.dat";
	//signs filename
	signsFilename = allSimulations + "sign_0/signs.dat";
	//goals filename
	goalsFilename = allSimulations + "goals.dat";
	//exit filename
	exitFilename = allSimulations + "sign_0/Exit.csv";
	//exit agents/goal filename
	agentsGoalFilename = allSimulations + "sign_0/AgentsGoal.csv";
	//auxins density
	PORC_QTD_Marcacoes = 0.65f;
	//simulation index
	simulationIndex = 0;
	//stop all sims
	gameOver = false;
	//last frame count
	lastFrameCount = 0;
	//intention threshold
	intentionThreshold = 0.8f;
	//simulation time step
	fps = 24;
	//how much is the obstacle far away from the world origin
	obstacleDisplacement = 500;
}

//start the simulation and control the update
void Simulation::StartSimulation() {
	//fps control
	double fpsTime = 0;

	//each time step, call again
	while (true) {
		//update simulation timer
		simulationTime += (((double)clock()) / CLOCKS_PER_SEC) - simulationTime;
		//std::cout << fpsTime << " -- " << simulationTime << "\n";
		//update fps timer
		fpsTime += (((double)clock()) / CLOCKS_PER_SEC) - simulationTime;

		//if time variation if bigger than defined FPS, we "reset" it and Update.
		if (fpsTime >= (1 / fps)) {
			fpsTime -= (1 / fps);
			Update();
		}
	}
}

//checks if simulation is over
//for that, we check if is there still an agent in the scene
void Simulation::EndSimulation() {
	if (agents.size() == 0) {
		std::cout << "Finishing Simulation " + std::to_string(simulationIndex) << "\n";

		//close exit file
		exitFile.close();
		//close exit agents/goal file
		agentsGoalFile.close();

		//update simulation index
		simulationIndex++;

		std::ofstream theReader;

		//if index is >= than allDirs, we have reached the end. So, GAME OVER!!!
		if (simulationIndex >= allDirs.size())
		{
			gameOver = true;

			//reset the iterator
			theReader.open(allSimulations + "SimulationIterator.txt");
			theReader << "0";
			theReader.close();

			//reset the frame count
			theReader.open(allSimulations + "FrameCount.txt");
			theReader << "0";
			theReader.close();

			std::cout << "Simulation Done!\n";
		}
		else
		{
			//else, we keep it going
			//update the iterator
			theReader.open(allSimulations + "SimulationIterator.txt");
			theReader << simulationIndex;
			theReader.close();

			//update last frame count
			theReader.open(allSimulations + "FrameCount.txt");
			theReader << ((float)clock() - startTime) / CLOCKS_PER_SEC; //seconds
			theReader.close();

			//reset scene
			agents.clear();
			signs.clear();
			std::cout << "Loading Simulation " + std::to_string(simulationIndex) << "\n";
			LoadChainSimulation();
		}
	}
}

//control all chained simulations
//get the new set of files to setup the new simulation and start
void Simulation::LoadChainSimulation() {
	// Create a new StreamReader, tell it which file to read and what encoding the file
	std::ifstream theReader;
	std::string line;

	theReader.open(allSimulations + "SimulationIterator.txt");
	//we get the updated simulation Index
	std::getline(theReader, line);

	simulationIndex = std::stoi(line);
	theReader.close();

	theReader.open(allSimulations + "FrameCount.txt");
	//we get the last frame count
	std::getline(theReader, line);

	lastFrameCount = std::stoi(line);
	theReader.close();

	//each directory within the defined config directory has a set of simulation files
	//get all simulation files within the defined config directory
	std::vector<std::string> allFiles;
	_finddata_t data;
	std::string path = allSimulations + allDirs[simulationIndex] + "/*.*";

	intptr_t ff = _findfirst(path.c_str(), &data);
	if (ff != -1)
	{
		int res = 0;
		while (res != -1)
		{
			if ((data.attrib & _A_SUBDIR) != _A_SUBDIR)
			{
				//does not include the "go back" directory
				if (data.name != "." && data.name != "..") {
					allFiles.push_back(allSimulations + allDirs[simulationIndex] + "/" + data.name);
				}
			}
			res = _findnext(ff, &data);
		}
		_findclose(ff);
	}
	/*for (int i = 0; i < allFiles.size(); i++) {
		std::cout << allFiles[i] << "\n";
	}*/

	std::vector<std::string> schNam;
	std::vector<std::string> agfNam;
	std::vector<std::string> extNam;
	std::vector<std::string> sigNam;
	Split(scheduleFilename, '/', schNam);
	Split(agentsGoalFilename, '/', agfNam);
	Split(exitFilename, '/', extNam);
	Split(signsFilename, '/', sigNam);
	/*std::cout << schNam[schNam.size() - 1] << " -- " << agfNam[agfNam.size() - 1] << " -- " << extNam[extNam.size() - 1]
		<< " -- " << sigNam[sigNam.size() - 1] << "\n";*/
	for (int i = 0; i < allFiles.size(); i++)
	{
		//just csv and dat files
		if (allFiles[i].substr(allFiles[i].find_last_of(".") + 1) == "csv" || allFiles[i].substr(allFiles[i].find_last_of(".") + 1) == "dat") {
			if (allFiles[i].find(schNam[schNam.size() - 1]) != std::string::npos)
			{
				scheduleFilename = allFiles[i];
			}
			else if (allFiles[i].find(agfNam[agfNam.size() - 1]) != std::string::npos)
			{
				agentsGoalFilename = allFiles[i];
			}
			else if (allFiles[i].find(extNam[extNam.size() - 1]) != std::string::npos)
			{
				exitFilename = allFiles[i];
			}
			else if (allFiles[i].find(sigNam[sigNam.size() - 1]) != std::string::npos)
			{
				signsFilename = allFiles[i];
			}
		}
	}//std::cout << scheduleFilename << " -- " << agentsGoalFilename << " -- " << exitFilename << " -- " << signsFilename << "\n";

	exitFile.open(exitFilename);
	agentsGoalFile.open(agentsGoalFilename);

	LoadConfigFile();
}

//load a csv config file
void Simulation::LoadConfigFile() {
	std::string line;

	//schedule file, with agents and their schedules
	std::ifstream theReader;
	theReader.open(scheduleFilename);

	int lineCount = 1;
	// While there's lines left in the text file, do this:
	do
	{
		std::getline(theReader, line);

		if (line != "" && !line.empty())
		{
			//in first line, it is the qnt agents
			if (lineCount == 1)
			{
				qntAgents = std::stoi(line);
			}
			else {
				//each line 1 agent, separated by " "
				std::vector<std::string> entries;
				Split(line, ' ', entries);

				float newPositionX = std::stof(entries[0]);
				float newPositionY = 0;
				float newPositionZ = std::stof(entries[1]);
				float grain = 1;
				float originalGrain = grain;

				//check if there is an obstacle in this position
				while (InsideObstacle(newPositionX, newPositionY, newPositionZ)) {
					//if there is an obstacle, test with new positions
					if (!InsideObstacle(newPositionX + grain, newPositionY, newPositionZ))
					{
						newPositionX += grain;
						break;
					}
					else if (!InsideObstacle(newPositionX, newPositionY, newPositionZ - grain))
					{
						newPositionZ -= grain;
						break;
					}
					else if (!InsideObstacle(newPositionX - grain, newPositionY, newPositionZ))
					{
						newPositionX -= grain;
						break;
					}
					else if (!InsideObstacle(newPositionX, newPositionY, newPositionZ + grain))
					{
						newPositionZ += grain;
						break;
					}
					else
					{
						//if none, update with twice the grain to try again
						grain += originalGrain;
					}
				}

				//change his name
				int agentName = lineCount - 2;

				//find the cell in x - z coords, using his name
				//use the rest of division by cellRadius*2
				int restDivisionX = (int)((int)newPositionX % (int)(cellRadius * 2));
				int restDivisionZ = (int)((int)newPositionZ % (int)(cellRadius * 2));
				int cellIndex = -1;
				int nameX = (int)(newPositionX - restDivisionX);
				int nameZ = (int)(newPositionZ - restDivisionZ);
				for (int c = 0; c < cells.size(); c++) {
					if (cells[c].name == "cell" + std::to_string((float)nameX) + "-" + std::to_string((float)nameZ)) {
						cellIndex = c;
						break;
					}
				}

				//instantiate new agent
				Agent newAgent(newPositionX, newPositionY, newPositionZ, "agent" + std::to_string(agentName));
				//agent cell
				if (cellIndex > -1) {
					newAgent.SetCell(&cells[cellIndex]);
				}
				else {
					std::cout << newAgent.name << ": " << "Celula nao encontrada! CellNameX: " + std::to_string((float)nameX) + " -- CellNameZ: "
						+ std::to_string((float)nameZ) + "\n";
				}
				//agent radius
				newAgent.agentRadius = agentRadius;

				//set his goals
				//go 2 in 2, since it is a pair between goal and intention to that goal
				for (int j = 2; j < entries.size(); j = j + 2)
				{
					//there is an empty space on the end of the line, dont know why.
					if (entries[j] == "") continue;

					//try to find this goal object
					if (std::stoi(entries[j]) < goals.size())
					{
						//add a goal
						newAgent.go.push_back(&goals[std::stoi(entries[j])]);
						//add intention
						newAgent.intentions.push_back(std::stof(entries[j + 1]));
						//add a random desire
						newAgent.AddDesire(RandomFloat(0, 1));
					}
				}

				//reorder following intentions
				newAgent.ReorderGoals();

				agents.push_back(newAgent);
			}
		}
		lineCount++;
	} while (line != "" && !line.empty());
	// Done reading, close the reader and return true to broadcast success
	theReader.close();
	/*for (int i = 0; i < agents.size(); i++) {
		std::cout << agents[i].name << "\n";
	}*/
	/*for (int i = 0; i < goals.size(); i++) {
		std::cout << goals[i].name << "\n";
	}*/


	//signs file, with signs and their appeals
	theReader.open(signsFilename);

	lineCount = 1;
	// While there's lines left in the text file, do this:
	do
	{
		std::getline(theReader, line);

		if (line != "" && !line.empty())
		{
			//in first line, it is the qnt signs
			if (lineCount == 1)
			{
				qntSigns = std::stoi(line);
			}
			else
			{
				//each line 1 agent, separated by " "
				std::vector<std::string> entries;
				Split(line, ' ', entries);

				//sign position
				float newPositionX = 0;
				float newPositionY = 0;
				float newPositionZ = 0;
				//define position based on obstacle vertices
				//@TODO: see how to draw the obstacles and interact with them
				//if (allObstacles.Length > 0) {
				if (true) {
					//if (allObstacles.Length == 1)
					if (true)
					{
						//check group vertices to find the corners
						newPositionX = verticesObstaclesX[round(RandomFloat(0, verticesObstaclesX.size()) - 1)];
						//newPositionY = verticesObstaclesY[round(RandomFloat(0, verticesObstaclesY.size()) - 1)]; //y is zero
						newPositionZ = verticesObstaclesZ[round(RandomFloat(0, verticesObstaclesZ.size()) - 1)];
						bool newPositionOK = true;

						//check every sign
						for (int p = 0; p < signs.size(); p++) {
							if (signs[p].posX == newPositionX && signs[p].posZ == newPositionZ) {
								newPositionOK = false;
								break;
							}
						}

						//while newPosition is inside the already used positions, we try again
						while (!newPositionOK)
						{
							newPositionX = verticesObstaclesX[round(RandomFloat(0, verticesObstaclesX.size()) - 1)];
							//newPositionY = verticesObstaclesY[round(RandomFloat(0, verticesObstaclesY.size()) - 1)]; //y is zero
							newPositionZ = verticesObstaclesZ[round(RandomFloat(0, verticesObstaclesZ.size()) - 1)];

							newPositionOK = true;

							//check every sign
							for (int p = 0; p < signs.size(); p++) {
								if (signs[p].posX == newPositionX && signs[p].posZ == newPositionZ) {
									newPositionOK = false;
									break;
								}
							}
						}
					}
					/*else
					{
						//check group vertices to find the corners
						newPosition = verticesObstacles[(int)Mathf.Floor(Random.Range(0, verticesObstacles.Length)) - 1];
						//Debug.Log("Position: "+newPosition+" -- "+positionsSigns.Contains(newPosition)+"--Index: "+index);
						//while newPosition is inside the already used positions, we try again
						//while (positionsSigns.Contains(newPosition))
						//{
						//index = (int)Mathf.Floor(Random.Range(0, allObstacles.Length));
						//vertices = allObstacles[index].GetComponent<MeshFilter>().mesh.vertices;
						//newPosition = vertices[(int)Mathf.Floor(Random.Range(0, vertices.Length))]; //+ new Vector3(500f, 0f, 500f)
						//Debug.Log("While...."+"Position: " + newPosition + " -- " + positionsSigns.Contains(newPosition) + "--Index: " + index); break;
						//}
					}*/
				}

				//will file bring sign goal
				std::string signGoalName = "Goal" + (std::stoi(entries[1]) - 1);
				//find its goal
				for (int g = 0; g < goals.size(); g++) {
					if (goals[g].name == signGoalName) {
						DrawSign(newPositionX, newPositionY, newPositionZ, &goals[g], std::stof(entries[2]));
					}
				}
			}
		}
		lineCount++;
	} while (line != "" && !line.empty());
	// Done reading, close the reader and return true to broadcast success
	theReader.close();

	/*for (int p = 0; p < signs.size(); p++) {
		std::cout << signs[p].GetGoal()->name << "\n";
	}*/
}

//load cells and auxins and obstacles and goals (static stuff)
void Simulation::LoadCellsAuxins() {
	//read the obstacle file
	ReadOBJFile();

	//precalc values to check obstacles
	PreCalcValues();

	// Create a new reader, tell it which file to read
	std::ifstream theReader;
	theReader.open(configFilename);
	std::string line;

	int qntCells = 0;

	int lineCount = 1;
	// While there's lines left in the text file, do this:
	do
	{
		std::getline(theReader, line);

		if (line != "" && !line.empty())
		{
			//in first line, we have the terrain size
			if (lineCount == 1)
			{
				std::vector<std::string> entries;
				std::vector<std::string> Tsize;
				Split(line, ':', entries);
				Split(entries[1], ',', Tsize);

				scenarioSizeX = std::stof(Tsize[0]);
				scenarioSizeZ = std::stof(Tsize[1]);
			}
			//in second line, we have the camera position
			else if (lineCount == 2)
			{
				//ConfigCamera();
			}
			//in the third line, we have the qntCells to instantiante
			else if (lineCount == 3)
			{
				std::vector<std::string> entries;
				Split(line, ':', entries);
				qntCells = std::stoi(entries[1]);
			}
			//else, if we are in the qntCells+4 line, we have the qntAuxins to instantiante
			else if (lineCount == qntCells + 4)
			{
				std::vector<std::string> entries;
				Split(line, ':', entries);
				qntAuxins = std::stoi(entries[1]);
			}
			else
			{
				//while we are til qntCells+3 line, we have cells. After that, we have auxins
				if (lineCount <= qntCells + 3)
				{
					std::vector<std::string> entries;
					Split(line, ';', entries);

					if (entries.size() > 0)
					{
						//new cell
						Cell newCell(std::stof(entries[1]), std::stof(entries[2]), std::stof(entries[3]), entries[0]);
						cellRadius = std::stof(entries[4]);
						cells.push_back(newCell);
					}
				}
				else if (lineCount <= qntCells + qntAuxins + 4)
				{
					std::vector<std::string> entries;
					Split(line, ';', entries);
					if (entries.size() > 0)
					{
						//find his cell
						std::string hisCell = entries[5];
						int ind = -1;
						for (int i = 0; i < cells.size(); i++) {
							if (cells[i].name == hisCell) {
								ind = i;
								break;
							}
						}

						if (ind > -1) {
							Marker newMarker(std::stof(entries[1]), std::stof(entries[2]), std::stof(entries[3]));
							newMarker.name = entries[0];
							cells[ind].AddAuxin(newMarker);
						}
						else {
							std::cout << "Celula nao encontrada!\n";
						}
						//alter auxinRadius
						auxinRadius = std::stof(entries[4]);
					}
				}
			}
		}
		lineCount++;
	} while (line != "" && !line.empty());
	// Done reading, close the reader and return true to broadcast success
	theReader.close();

	std::cout << "Qnt Cells: " << cells.size() << "\n";

	/*for (int i = 0; i < cells.size(); i++) {
		std::cout << cells[i].name << " -- Qnt Markers: " << cells[i].GetAuxins()->size() << "\n";
	}*/

	//Goals file, with goals and their positions
	theReader.open(goalsFilename);

	lineCount = 1;
	// While there's lines left in the text file, do this:
	do
	{
		std::getline(theReader, line);

		if (line != "" && !line.empty())
		{
			//in first line, it is the qnt goals
			if (lineCount == 1)
			{
				qntGoals = std::stoi(line);
			}
			else
			{
				//each line 1 agent, separated by " "
				std::vector<std::string> entries;
				Split(line, ' ', entries);

				//instantiante it
				DrawGoal(entries[0], std::stof(entries[1]), 0, std::stof(entries[2]), false);
			}
		}
		lineCount++;
	} while (line != "" && !line.empty());
	// Done reading, close the reader and return true to broadcast success
	theReader.close();

	std::cout << "Qnt Goals: " << goals.size() << "\n";

	//instantiate the goal's signs
	for (int p = 0; p < goals.size(); p++) {
		DrawSign(goals[p].posX, goals[p].posY, goals[p].posZ, &goals[p], 1);
	}

	/*for (int i = 0; i < signs.size(); i++) {
		std::cout << signs[i].posX << "\n";
	}*/
}

//draw a goal
void Simulation::DrawGoal(std::string goalName, float goalPositionX, float goalPositionY, float goalPositionZ, bool isLF)
{
	Goal newGoal(goalName, goalPositionX, goalPositionY, goalPositionZ, isLF);
	goals.push_back(newGoal);
}

//draw a sign
void Simulation::DrawSign(float signPositionX, float signPositionY, float signPositionZ, Goal* signGoal, float signAppeal) {
	Sign newSign(signPositionX, signPositionY, signPositionZ);
	newSign.SetGoal(signGoal);
	newSign.SetAppeal(signAppeal);
	signs.push_back(newSign);
	//std::cout << signs[signs.size()-1].GetGoal()->name << "\n";
}

//draw cells
void Simulation::DrawCells()
{
	//first of all, create all cells (with this scene and this agentRadius = 1 : 150 cells)
	//since radius = 1; diameter = 2. So, iterate cellRadius*2
	//if the radius varies, this 2 operations adjust the cells
	float newPositionX = cellRadius;
	float newPositionY = 0;
	float newPositionZ = cellRadius;

	for (float i = 0; i < scenarioSizeX; i = i + cellRadius * 2)
	{
		for (float j = 0; j < scenarioSizeZ; j = j + cellRadius * 2)
		{
			//verify if collides with some obstacle. We dont need cells in objects.
			//for that, we need to check all 4 sides of the cell. Otherwise, we may not have cells in some free spaces (for example, half of a cell would be covered by an obstacle, so that cell
			//would not be instantied)
			bool collideRight = InsideObstacle(newPositionX + i + cellRadius, newPositionY, newPositionZ + j + cellRadius);
			bool collideLeft = InsideObstacle(newPositionX + i - cellRadius, newPositionY, newPositionZ + j + cellRadius);
			bool collideTop = InsideObstacle(newPositionX + i + cellRadius, newPositionY, newPositionZ + j - cellRadius);
			bool collideDown = InsideObstacle(newPositionX + i - cellRadius, newPositionY, newPositionZ + j - cellRadius);

			//if did collide it all, means we have found at least 1 obstacle in each case. So, the cell is covered by an obstacle
			//otherwise, we go on
			if (!collideRight || !collideLeft || !collideTop || !collideDown)
			{
				//new cell
				Cell newCell(newPositionX + i, newPositionY, newPositionZ + j, "cell" + std::to_string(i) + "-" + std::to_string(j));
				cells.push_back(newCell);
			}
		}
	}
	std::cout << "Cells Done! \n";
	/*for (int i = 0; i < cells.size(); i++) {
		std::cout << i << "--" << cells[i].name << " -- Qnt Markers: " << cells[i].GetAuxins()->size() << "\n";
	}*/
}

//place auxins
void Simulation::PlaceAuxins() {
	//lets set the qntAuxins for each cell according the density estimation
	float densityToQnt = PORC_QTD_Marcacoes;

	densityToQnt *= (cellRadius * 2) / (2.0f * auxinRadius);
	densityToQnt *= (cellRadius * 2) / (2.0f * auxinRadius);

	qntAuxins = (int)round(densityToQnt);

	//for each cell, we generate his auxins
	for (int c = 0; c < cells.size(); c++)
	{
		//Dart throwing auxins
		//use this flag to break the loop if it is taking too long (maybe there is no more space)
		int flag = 0;
		for (int i = 0; i < qntAuxins; i++)
		{
			//k = ((float)rand()) / ((float)RAND_MAX) ;
			float x = RandomFloat(cells[c].posX - cellRadius, cells[c].posX + cellRadius);
			float z = RandomFloat(cells[c].posZ - cellRadius, cells[c].posZ + cellRadius);
			//std::cout << cells[c].name << ": PosX - " << cells[c].posX << " -- " << x << " -- " << z << "\n";

			//see if there are auxins in this radius. if not, instantiante
			std::vector<Marker>* allAuxinsInCell = cells[c].GetAuxins();
			bool canIInstantiante = true;

			for (int j = 0; j < allAuxinsInCell->size(); j++)
			{
				float distanceAA = Distance(x, 0, z, (*allAuxinsInCell)[j].posX, (*allAuxinsInCell)[j].posY, (*allAuxinsInCell)[j].posZ);

				//if it is too near, i cant instantiante. found one, so can Break
				if (distanceAA < auxinRadius)
				{
					canIInstantiante = false;
					break;
				}
			}

			//if i have found no auxin, i still need to check if is there obstacles on the way
			if (canIInstantiante)
			{
				canIInstantiante = !InsideObstacle(x, 0, z);
			}

			//canIInstantiante???
			if (canIInstantiante)
			{
				Marker newMarker(x, 0, z);
				newMarker.name = "marker" + std::to_string(x) + "-" + std::to_string(z);
				cells[c].AddAuxin(newMarker);

				//reset the flag
				flag = 0;
			}
			else
			{
				//else, try again
				flag++;
				i--;
			}

			//if flag is above qntAuxins (*2 to have some more), break;
			if (flag > qntAuxins * 2)
			{
				//reset the flag
				flag = 0;
				break;
			}
		}

		std::cout << cells[c].name << " done adding markers! Qnt markers: " << cells[c].GetAuxins()->size() << "\n";
	}

	/*for (int i = 0; i < cells.size(); i++) {
		//std::cout << cells[i].GetAuxins()->size() << "\n";
		std::cout << cells[i].GetAuxins()->at(0).name << "\n";
	}*/
}

//save a csv config file
//files saved: Config.csv, Obstacles.csv, goals.dat
void Simulation::SaveConfigFile() {
	//config file
	std::ofstream file;
	file.open(configFilename);
	//obstacles file
	std::ofstream fileObstacles;
	fileObstacles.open(obstaclesFilename);
	//goals file
	std::ofstream fileGoals;
	fileGoals.open(goalsFilename);

	//first, we save the terrain dimensions
	file << "terrainSize:" + std::to_string(scenarioSizeX) + "," + std::to_string(scenarioSizeZ) + "\n";

	//then, camera position and height
	//just need to have the line, since camera is not used here
	file << "camera\n";

	std::string allAuxins = "";
	int qntAuxins = 0;

	//get cells info
	if (cells.size() > 0)
	{
		//each line: name, positionx, positiony, positionz, cell radius
		//separated with ;

		file << "qntCells:" + std::to_string(cells.size()) + "\n";
		//for each cell auxin
		for (int i = 0; i < cells.size(); i++)
		{
			file << cells[i].name + ";" + std::to_string(cells[i].posX) + ";" + std::to_string(cells[i].posY) +
				";" + std::to_string(cells[i].posZ) + ";" + std::to_string(cellRadius) + "\n";

			//add all cell auxins to write later
			std::vector<Marker>* allCellAuxins = cells[i].GetAuxins();
			for (int j = 0; j < allCellAuxins->size(); j++)
			{
				allAuxins += (*allCellAuxins).at(j).name + ";" + std::to_string((*allCellAuxins).at(j).posX) + ";" +
					std::to_string((*allCellAuxins).at(j).posY) + ";" + std::to_string((*allCellAuxins).at(j).posZ) + ";" +
					std::to_string(auxinRadius) + ";" + cells[i].name + "\n";
				qntAuxins++;
			}
		}
	}

	//write the auxins on the file
	file << "qntAuxins:" << std::to_string(qntAuxins) << "\n";
	file << allAuxins;

	file.close();

	//get obstacles info
	//@TODO: JUST ONE OBSTACLE SO FAR. SEE IF IT WORKS FOR MORE THAN 1 (not necessary for Purdue)
	if (polygonX.size() > 0)
	{
		//separated with ;
		fileObstacles << "qntObstacles:1\n";
		//new line for the obstacle name
		fileObstacles << "\nObstacle\n";
		//new line for the qnt vertices
		fileObstacles << "qntVertices:" + std::to_string(polygonX.size()) + "\n";

		//for each vertice
		for (int i = 0; i < polygonX.size(); i++)
		{
			fileObstacles << std::to_string(polygonX[i]) + ";0;" + std::to_string(polygonZ[i]) + "\n";
		}

		//new line for the qnt triangles
		fileObstacles << "qntTriangles:" + std::to_string(trianglesObstacle.size()) + "\n";

		//for each triangle
		for (int i = 0; i < trianglesObstacle.size(); i++)
		{
			fileObstacles << std::to_string(trianglesObstacle[i]) + "\n";
		}
	}

	fileObstacles.close();

	//get goals info
	if (goals.size() > 0)
	{
		//separated with " "
		fileGoals << std::to_string(goals.size()) + "\n";
		//for each goal
		for (int i = 0; i < goals.size(); i++)
		{
			//new line for the goal name and position
			fileGoals << goals[i].name + " " + std::to_string(goals[i].posX) + " " + std::to_string(goals[i].posZ) + "\n";
		}
	}

	fileGoals.close();
}

// Update is called once per frame
void Simulation::Update() {
	//if simulation should be running yet
	if (!gameOver)
	{
		//update simulationTime
		simulationTime = clock() - simulationTime;

		//reset auxins
		//it must be here because we need to make sure they reset before calculate the new auxins
		for (int i = 0; i < cells.size(); i++)
		{
			std::vector<Marker>* allAuxins = cells[i].GetAuxins();
			for (int j = 0; j < allAuxins->size(); j++)
			{
				(*allAuxins)[j].ResetAuxin();
			}
		}
		/*
		//REMEMBER WHEN DO THIS: WHEN THE AGENT "DIES", NEED TO CLEAR THEIR AUXINS!!!
		for (int i = 0; i < qntAgents; i++) {
			//first, lets see if the agent is still in the scene
			bool destroyed = false;
			for (int j = 0; j < agentsDestroyed.Count; j++)
			{
				if (agentsDestroyed[j] == i) destroyed = true;
			}

			//if he is
			if (!destroyed)
			{
				GameObject agentI = GameObject.Find("agent" + i);
				List<AuxinController> axAge = agentI.GetComponent<AgentController>().GetAuxins();
				for (int j = 0; j < axAge.Count; j++)
				{
					axAge[j].ResetAuxin();
				}
				// Debug.Log(axAge.Count);
			}
		}
		*/

		//find nearest auxins for each agent
		for (int i = 0; i < agents.size(); i++)
		{
			//find all auxins near him (Voronoi Diagram)
			agents[i].FindNearAuxins(cellRadius, &cells, &agents);
		}
		//std::cout << agents[0].GetAuxins().size() << "\n";
		/*
		/*to find where the agent must move, we need to get the vectors from the agent to each auxin he has, and compare with
		the vector from agent to goal, generating a angle which must lie between 0 (best case) and 180 (worst case)
		The calculation formula was taken from the Bicho´s mastery tesis and from Paravisi algorithm, all included
		in AgentController.
		*/

		/*for each agent, we:
		1 - verify if he is in the scene. If he is...
		2 - find him
		3 - for each auxin near him, find the distance vector between it and the agent
		4 - calculate the movement vector (CalculaDirecaoM())
		5 - calculate speed vector (CalculaVelocidade())
		6 - walk (Caminhe())
		7 - verify if the agent has reached the goal. If so, destroy it
		*/
		for (int i = 0; i < agents.size(); i++)
		{
			//update agent
			agents[i].Update(&signs);

			//find his goal
			Goal *goal = agents[i].go[0];
			std::vector<Marker*> agentAuxins = agents[i].GetAuxins();

			//vector for each auxin
			for (int j = 0; j < agentAuxins.size(); j++)
			{
				//add the distance vector between it and the agent
				agents[i].vetorDistRelacaoMarcacaoX.push_back(agentAuxins[j]->posX - agents[i].posX);
				agents[i].vetorDistRelacaoMarcacaoY.push_back(agentAuxins[j]->posY - agents[i].posY);
				agents[i].vetorDistRelacaoMarcacaoZ.push_back(agentAuxins[j]->posZ - agents[i].posZ);
			}
			/*for (int v = 0; v < agents[i].vetorDistRelacaoMarcacaoX.size(); v++) {
				std::cout << agents[i].vetorDistRelacaoMarcacaoX[v] << "\n";
			}*/

			//calculate the movement vector
			agents[i].CalculaDirecaoM();
			//calculate speed vector
			agents[i].CalculaVelocidade();

			//now, we check if agent is stuck with another agent
			//if so, change places
			if (agents[i].speedX == 0 && agents[i].speedY == 0 && agents[i].speedZ == 0)
			{
				//check distance between this agent and every other agent
				for (int j = 0; j < agents.size(); j++) {
					//if they are too near and both with zero speed, probally stuck. Swap positions if this agent may do it
					if (Distance(agents[i].posX, agents[i].posY, agents[i].posZ, agents[j].posX, agents[j].posY, agents[j].posZ)
						< 0.1f && (agents[j].speedX == 0 && agents[j].speedY == 0 && agents[j].speedZ == 0) && agents[i].changePosition && i != j) {
						std::cout << "\n LOCKED: " + agents[i].name + " with " + agents[j].name + "\n";
						//system("PAUSE");

						float posAuxX = agents[i].posX;
						float posAuxY = agents[i].posY;
						float posAuxZ = agents[i].posZ;
						agents[i].posX = agents[j].posX;
						agents[i].posY = agents[j].posY;
						agents[i].posZ = agents[j].posZ;
						agents[j].posX = posAuxX;
						agents[j].posY = posAuxY;
						agents[j].posZ = posAuxZ;

						//the other agent doesnt change position
						agents[j].changePosition = false;

						break;
					}
				}
			}

			//walk
			agents[i].Caminhe((double)(1.0 / fps));

			//std::cout << "Segundos: " << ((float)simulationT) / CLOCKS_PER_SEC << "\n";
			//std::cout << agents[i].name << ": " << agents[i].posX << "-" << agents[i].posZ << "\n";

			//verify agent position, in relation to the goal.
			//if the distance between them is less than 1 (arbitrary, maybe authors have a better solution), he arrived. Destroy it so
			float dist = Distance(goal->posX, goal->posY, goal->posZ, agents[i].posX, agents[i].posY, agents[i].posZ);
			if (dist < agents[i].agentRadius)
			{
				//he arrived! Lets save this on file
				//open exit file to save info each frame
				//JUST NEED TO SAVE THE LAST ONE, SO BYEEEE...
				//SaveAgentsGoalFile(agentI.name, goal.name);

				//if we are already at the last agent goal, he arrived
				//if he has 2 goals yet, but the second one is the Looking For, he arrived too
				if (agents[i].go.size() == 1 || (agents[i].go.size() == 2 && agents[i].go[1]->name == "LookingFor"))
				{
					SaveAgentsGoalFile(agents[i].name, goal->name);
					agents.erase(agents.begin() + i);
					//this agent is done. Back to the for
					continue;
				}//else, he must go to the next goal. Remove this actual goal and this intention
				else
				{
					//before we remove his actual go, we check if it is the looking for state.
					//if it is, we change its position, because he doesnt know where to go yet
					if (agents[i].go[0]->name == "LookingFor") {
						ChangeLookingFor(agents[i].go[0]);
						for (int j = 0; j < agents[i].go.size(); j++) {
							std::cout << agents[i].go[j]->name << ": PosX - " << agents[i].go[j]->posX << " -- PosZ - "
								<< agents[i].go[j]->posZ << " -- Intention: " << agents[i].intentions[j] << "\n";
						}
					}//else, just remove it
					else {
						std::cout << agents[i].name << " chegou no goal " << agents[i].go[0]->name << "\n";
						agents[i].go.erase(agents[i].go.begin());
						agents[i].intentions.erase(agents[i].intentions.begin());
						agents[i].RemoveDesire(0);
					}
				}
			}
		}

		//write the exit file
		SaveExitFile();

		//End simulation?
		if (loadConfigFile)
		{
			EndSimulation();
		}
	}
}

//save a csv exit file, with positions of all agents in function of time
void Simulation::SaveExitFile() {
	//get agents info
	if (agents.size() > 0)
	{
		//each line: frame, agents name, positionx, positiony, positionz, goal object name, cell name
		//separated with ;
		//for each agent
		for (int i = 0; i < agents.size(); i++)
		{
			exitFile << std::to_string((((float)clock() - startTime) / CLOCKS_PER_SEC) - lastFrameCount) + ";" + agents[i].name + ";"
				+ std::to_string(agents[i].posX) + ";" + std::to_string(agents[i].posY) + ";" + std::to_string(agents[i].posZ) + ";" +
				agents[i].go[0]->name + ";" + agents[i].GetCell()->name + "\n";
		}
	}
}

void Simulation::SaveAgentsGoalFile(std::string agentName, std::string goalName) {
	//we save: Agent name, Goal name, Time he arrived
	agentsGoalFile << agentName + ";" + goalName + ";" + std::to_string((((float)clock() - startTime) / CLOCKS_PER_SEC) - lastFrameCount) + "\n";
}


//"draw" obstacles on the scene
void Simulation::DrawObstacles() {
	//draw rectangle
	std::vector<float> verticesX;
	std::vector<float> verticesY;
	std::vector<float> verticesZ;
	std::vector<int> triangles;

	//set vertices
	//vertice 1
	verticesX.push_back(5.0f);
	verticesY.push_back(0.0f);
	verticesZ.push_back(10.0f);
	//vertice 2
	verticesX.push_back(5.0f);
	verticesY.push_back(0.0f);
	verticesZ.push_back(13.0f);
	//vertice 3
	verticesX.push_back(15.0f);
	verticesY.push_back(0.0f);
	verticesZ.push_back(13.0f);
	//vertice 4
	verticesX.push_back(15.0f);
	verticesY.push_back(0.0f);
	verticesZ.push_back(10.0f);

	//triangles
	triangles.push_back(0);
	triangles.push_back(1);
	triangles.push_back(2);
	triangles.push_back(2);
	triangles.push_back(3);
	triangles.push_back(0);

	//"draw" it
	DrawObstacle(verticesX, verticesY, verticesZ, triangles);

	//build the navmesh at runtime
	//NavMeshBuilder.BuildNavMesh();
}

//draw each obstacle, placing its vertices on the verticesObstacles. So, signs can be instantiated on those places
void Simulation::DrawObstacle(std::vector<float> verticesX, std::vector<float> verticesY, std::vector<float> verticesZ, std::vector<int> triangles) {
	for (int i = 0; i < verticesX.size(); i++) {
		//polygon for InsideObstacle calculus
		//@TODO: CHECK IF IT WORKS FOR 2 OR MORE OBSTACLES!! (not necessary for Purdue)
		polygonX.push_back(verticesX[i]);
		polygonZ.push_back(verticesZ[i]);
	}

	//triangles
	trianglesObstacle = triangles;
}

//check group vertices to find the corners
//@TODO: JUST WORKS FOR 1 OBSTACLE! NEED TO SEE IF CAN DO FOR 2 OR MORE (not necessary for Purdue)
void Simulation::CheckGroupVertices()
{
	std::vector<int> groupTriangles;
	//start each index with 0
	for (int i = 0; i < trianglesObstacle.size(); i++) {
		groupTriangles.push_back(0);
	}

	int group = 0;

	std::cout << "Starting to check group vertices!\n";

	//go through the triangles checking if they are related each other
	while (true)
	{
		bool die = true;

		//check if we need to update group value
		for (int i = 0; i < groupTriangles.size(); i = i + 3)
		{
			//if there is at least one zero value yet, need to keep going
			if (groupTriangles[i] == 0)
			{
				die = false;

				group++;

				groupTriangles[i] = group;
				groupTriangles[i + 1] = group;
				groupTriangles[i + 2] = group;

				//ind = i;
				break;
			}
		}

		//if there are no more zero values break;
		if (die)
		{
			break;
		}

		//for each triangle, we get it...
		for (int i = 0; i < trianglesObstacle.size(); i = i + 3)
		{
			int triangleI1 = trianglesObstacle[i];
			int triangleI2 = trianglesObstacle[i + 1];
			int triangleI3 = trianglesObstacle[i + 2];

			//...to compare with other triangles
			for (int j = 0; j < trianglesObstacle.size(); j = j + 3)
			{
				if (i != j)
				{
					int triangleJ1 = trianglesObstacle[j];
					int triangleJ2 = trianglesObstacle[j + 1];
					int triangleJ3 = trianglesObstacle[j + 2];

					//super if to see if the 2 triangles have any vertice in common
					if (triangleJ1 == triangleI1 || triangleJ1 == triangleI2 || triangleJ1 == triangleI3 ||
						triangleJ2 == triangleI1 || triangleJ2 == triangleI2 || triangleJ2 == triangleI3 ||
						triangleJ3 == triangleI1 || triangleJ3 == triangleI2 || triangleJ3 == triangleI3)
					{
						//if they do, they are from the same group!
						//now, need to see which group it is
						if (groupTriangles[i] > 0 && (groupTriangles[j] == 0 && groupTriangles[j + 1] == 0 && groupTriangles[j + 2] == 0))
						{
							groupTriangles[j] = groupTriangles[i];
							groupTriangles[j + 1] = groupTriangles[i];
							groupTriangles[j + 2] = groupTriangles[i];
						}
						else if (groupTriangles[j] > 0 && (groupTriangles[i] == 0 && groupTriangles[i + 1] == 0 && groupTriangles[i + 2] == 0))
						{
							groupTriangles[i] = groupTriangles[j];
							groupTriangles[i + 1] = groupTriangles[j];
							groupTriangles[i + 2] = groupTriangles[j];
						}
					}
				}
			}
		}
	}

	std::cout << std::to_string(group) << "\n";

	//qnt groups * 4, since need to find the "bounding box" of each block
	//will not use the bouding box itself, but the idea it bears
	std::vector<float> newVerticesX;
	std::vector<float> newVerticesY;
	std::vector<float> newVerticesZ;
	int verticesIndex = 0;
	int groupIT = 1;
	while (groupIT <= group) {
		//the boundary
		float maxX = 0;
		float maxZ = 0;
		float minX = 0;
		float minZ = 0;

		bool first = true;
		//here, we get the boundary
		for (int i = 0; i < groupTriangles.size(); i++)
		{
			if (groupTriangles[i] == groupIT)
			{
				if (first)
				{
					maxX = polygonX[trianglesObstacle[i]];
					maxZ = polygonZ[trianglesObstacle[i]];
					minX = polygonX[trianglesObstacle[i]];
					minZ = polygonZ[trianglesObstacle[i]];
					first = false;
				}
				else
				{
					if (polygonX[trianglesObstacle[i]] > maxX)
					{
						maxX = polygonX[trianglesObstacle[i]];
					}
					if (polygonX[trianglesObstacle[i]] < minX)
					{
						minX = polygonX[trianglesObstacle[i]];
					}
					if (polygonZ[trianglesObstacle[i]] > maxZ)
					{
						maxZ = polygonZ[trianglesObstacle[i]];
					}
					if (polygonZ[trianglesObstacle[i]] < minZ)
					{
						minZ = polygonX[trianglesObstacle[i]];
					}
				}
			}
		}

		//lists for the vertices which belong the boundary
		std::vector<float> allMinX_X;
		std::vector<float> allMinX_Z;
		std::vector<float> allMaxX_X;
		std::vector<float> allMaxX_Z;
		std::vector<float> allMinZ_X;
		std::vector<float> allMinZ_Z;
		std::vector<float> allMaxZ_X;
		std::vector<float> allMaxZ_Z;
		for (int i = 0; i < groupTriangles.size(); i++)
		{
			if (groupTriangles[i] == groupIT)
			{
				//new idea: find all vertices with minX, maxX, etc; and use them directly, with no bounding box
				if (polygonX[trianglesObstacle[i]] == minX) {
					//if it is not in the list yet, add
					bool contains = false;
					for (int m = 0; m < allMinX_X.size(); m++) {
						if (allMinX_X[m] == polygonX[trianglesObstacle[i]] && allMinX_Z[m] == polygonZ[trianglesObstacle[i]]) {
							contains = true;
							break;
						}
					}
					if (!contains)
					{
						allMinX_X.push_back(polygonX[trianglesObstacle[i]]);
						allMinX_Z.push_back(polygonZ[trianglesObstacle[i]]);
					}
				}
				if (polygonX[trianglesObstacle[i]] == maxX)
				{
					//if it is not in the list yet, add
					bool contains = false;
					for (int m = 0; m < allMaxX_X.size(); m++) {
						if (allMaxX_X[m] == polygonX[trianglesObstacle[i]] && allMaxX_Z[m] == polygonZ[trianglesObstacle[i]]) {
							contains = true;
							break;
						}
					}
					if (!contains)
					{
						allMaxX_X.push_back(polygonX[trianglesObstacle[i]]);
						allMaxX_Z.push_back(polygonZ[trianglesObstacle[i]]);
					}
				}
				if (polygonZ[trianglesObstacle[i]] == minZ)
				{
					bool contains = false;
					for (int m = 0; m < allMinZ_X.size(); m++) {
						if (allMinZ_X[m] == polygonX[trianglesObstacle[i]] && allMinZ_Z[m] == polygonZ[trianglesObstacle[i]]) {
							contains = true;
							break;
						}
					}
					if (!contains)
					{
						allMinZ_X.push_back(polygonX[trianglesObstacle[i]]);
						allMinZ_Z.push_back(polygonZ[trianglesObstacle[i]]);
					}
				}
				if (polygonZ[trianglesObstacle[i]] == maxZ)
				{
					bool contains = false;
					for (int m = 0; m < allMaxZ_X.size(); m++) {
						if (allMaxZ_X[m] == polygonX[trianglesObstacle[i]] && allMaxZ_Z[m] == polygonZ[trianglesObstacle[i]]) {
							contains = true;
							break;
						}
					}
					if (!contains)
					{
						allMaxZ_X.push_back(polygonX[trianglesObstacle[i]]);
						allMaxZ_Z.push_back(polygonZ[trianglesObstacle[i]]);
					}
				}
			}
		}
		/*
		List<Vector3> fourVertices = new List<Vector3>();
		//now we have the possible vertices. Lets decide which 4 to use
		//if allMinX just have 1 vertice, just check his z value
		if (fourVertices.Count < 4)
		{
			if (allMinX.Count == 1)
			{
				if (!fourVertices.Contains(allMinX[0]))
				{
					fourVertices.Add(allMinX[0]);
				}
			}
			//else, it must already contain bottom and top
			else
			{
				//find the highest and lowest z
				Vector3 lZ = new Vector3(1000f, 0, 1000f);
				Vector3 hZ = new Vector3(-1000f, 0, -1000f);
				foreach(Vector3 cont in allMinX)
				{
					if (cont.z < lZ.z)
					{
						lZ = cont;
					}
					if (cont.z > hZ.z)
					{
						hZ = cont;
					}
				}

				if (!fourVertices.Contains(lZ))
				{
					fourVertices.Add(lZ);
				}
				if (!fourVertices.Contains(hZ))
				{
					fourVertices.Add(hZ);
				}
			}
		}

		//if allMaxX just have 1 vertice, just check his z value
		if (fourVertices.Count < 4)
		{
			if (allMaxX_X.size() == 1)
			{
				if (!fourVertices.Contains(allMaxX[0]))
				{
					fourVertices.Add(allMaxX[0]);
				}
			}
			//else, it must already contain bottom and top
			else
			{
				//find the highest and lowest z
				Vector3 lZ = new Vector3(1000f, 0, 1000f);
				Vector3 hZ = new Vector3(-1000f, 0, -1000f);
				foreach(Vector3 cont in allMaxX)
				{
					if (cont.z < lZ.z)
					{
						lZ = cont;
					}
					if (cont.z > hZ.z)
					{
						hZ = cont;
					}
				}
				if (!fourVertices.Contains(lZ))
				{
					fourVertices.Add(lZ);
				}
				if (!fourVertices.Contains(hZ))
				{
					fourVertices.Add(hZ);
				}
			}
		}

		if (fourVertices.Count < 4)
		{
			//if allMinZ just have 1 vertice, just check his x value
			if (allMinZ.Count == 1)
			{
				if (!fourVertices.Contains(allMinZ[0]))
				{
					fourVertices.Add(allMinZ[0]);
				}
			}
			//else, it must already contain left and right
			else
			{
				//find the highest and lowest z
				Vector3 lX = new Vector3(1000f, 0, 1000f);
				Vector3 hX = new Vector3(-1000f, 0, -1000f);
				foreach(Vector3 cont in allMinZ)
				{
					if (cont.x < lX.x)
					{
						lX = cont;
					}
					if (cont.x > hX.x)
					{
						hX = cont;
					}
				}
				if (!fourVertices.Contains(lX))
				{
					fourVertices.Add(lX);
				}
				if (!fourVertices.Contains(hX))
				{
					fourVertices.Add(hX);
				}
			}
		}

		if (fourVertices.Count < 4)
		{
			//if allMaxZ just have 1 vertice, just check his x value
			if (allMaxZ.Count == 1)
			{
				if (!fourVertices.Contains(allMaxZ[0]))
				{
					fourVertices.Add(allMaxZ[0]);
				}
			}
			//else, it must already contain left and right
			else
			{
				//find the highest and lowest z
				Vector3 lX = new Vector3(1000f, 0, 1000f);
				Vector3 hX = new Vector3(-1000f, 0, -1000f);
				foreach(Vector3 cont in allMaxZ)
				{
					if (cont.x < lX.x)
					{
						lX = cont;
					}
					if (cont.x > hX.x)
					{
						hX = cont;
					}
				}
				if (!fourVertices.Contains(lX))
				{
					fourVertices.Add(lX);
				}
				if (!fourVertices.Contains(hX))
				{
					fourVertices.Add(hX);
				}
			}
		}

		//now, assign the values
		foreach(Vector3 four in fourVertices)
		{
			newVertices[verticesIndex] = four + new Vector3(500, 0, 500);
			verticesIndex++;
		}*/

		groupIT++;
	}

	std::cout << "Finished to check group vertices!\n";

	//verticesObstacles = newVertices;
}

//TEST TO READ THE 4X4.OBJ
void Simulation::ReadOBJFile()
{
	std::ifstream theReader;
	theReader.open(allSimulations + "/StreetBlock.obj");
	std::string line;
	int qntVertices = 0;
	int qntTriangles = 0;
	std::vector<float> verticesX;
	std::vector<float> verticesY;
	std::vector<float> verticesZ;
	std::vector<int> triangles;

	int lineCount = 1;
	// While there's lines left in the text file, do this:
	do
	{
		std::getline(theReader, line);

		if (line != "" && !line.empty())
		{
			//if it contains #, it is a comment
			if (line[0] == '#')
			{
				std::vector<std::string> entries;
				Split(line, ' ', entries);
				if (entries[entries.size() - 1] == "vertices")
				{
					qntVertices = std::stoi(entries[entries.size() - 2]);
				}
				if (entries[entries.size() - 1] == "facets")
				{
					qntTriangles = std::stoi(entries[entries.size() - 2]);
				}
			}
			else if (line != "")
			{
				std::vector<std::string> entries;
				Split(line, ' ', entries);
				//if it starts with v, it is vertice. else, if it starts with f, it is facet which form a triangle (hopefully!)
				if (entries[0] == "v")
				{
					verticesX.push_back(std::stof(entries[1]) + obstacleDisplacement);
					verticesY.push_back(std::stof(entries[2]) + obstacleDisplacement);
					verticesZ.push_back(std::stof(entries[3]) + obstacleDisplacement);
				}
				else if (entries[0] == "f")
				{
					//minus 1 because it starts in 1, and our code starts in 0
					triangles.push_back(std::stoi(entries[2]) - 1);
					triangles.push_back(std::stoi(entries[3]) - 1);
					triangles.push_back(std::stoi(entries[4]) - 1);
				}
			}
		}

		lineCount++;
	} while (!theReader.eof());

	DrawObstacle(verticesX, verticesY, verticesZ, triangles);
	/*for (int v = 0; v < verticesX.size(); v++) {
		std::cout << verticesX[v] << "\n";
	}*/

	// Done reading, close the reader and return true to broadcast success
	theReader.close();
}

//generate a new looking for
//it will be added as a goal for each agent. So, a simulation should generate qntAgents looking for goals
void Simulation::GenerateLookingFor() {
	//for each agent
	for (int i = 0; i < qntAgents; i++) {
		bool pCollider = true;
		//while i have an obstacle on the way
		while (pCollider) {
			//generate the new position
			float x = RandomFloat(0, scenarioSizeX);
			float z = RandomFloat(0, scenarioSizeZ);

			//check if it is not inside an obstacle
			bool pCollider = InsideObstacle(x, 0, z);

			//if not, new looking for!
			if (!pCollider) {
				DrawGoal("LookingFor", x, 0, z, true);
				break;
			}
		}
	}
}

//change the position of a looking for goal
void Simulation::ChangeLookingFor(Goal* changeLF) {
	bool pCollider = true;
	//while i have an obstacle on the way
	while (pCollider) {
		//generate the new position
		float x = RandomFloat(0, scenarioSizeX);
		float z = RandomFloat(0, scenarioSizeZ);

		//check if it is not inside an obstacle
		bool pCollider = InsideObstacle(x, 0, z);

		//if not, new looking for!
		if (!pCollider) {
			changeLF->posX = x;
			changeLF->posY = 0;
			changeLF->posZ = z;
			break;
		}
	}
}

//distance between 2 points
float Simulation::Distance(float x1, float y1, float z1, float x2, float y2, float z2)
{
	float result = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (z2 - z1)*(z2 - z1));

	return result;
}

//split a string
void Simulation::Split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss;
	ss.str(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
}

//generate a random between a range
float Simulation::RandomFloat(float min, float max)
{
	float r = (float)rand() / (float)RAND_MAX;
	return min + r * (max - min);
}

//verify if a point is inside the obstacles
//Leandro approach (not used)
//this approach need the 4 vertices of the obstacle. Thus, just works with quadrilaterals. Besides, would need to separate the Tharindu "blocks" (just like i did on Unity).
/*bool Simulation::InsideObstacle(float pX, float pY, float pZ)
{
	int i = (pZ - C1.y)*(C2.x - C1.x) - (pX - C1.x)*(C2.y - C1.y);
	int j = (pZ - C2.y)*(C3.x - C2.x) - (pX - C2.x)*(C3.y - C2.y);
	int k = (pZ - C3.y)*(C4.x - C3.x) - (pX - C3.x)*(C4.y - C3.y);
	int l = (pZ - C4.y)*(C1.x - C4.x) - (pX - C4.x)*(C1.y - C4.y);
	printf("%d %d %d %d", i, j, k, l);
	if ((i >= 0 && j >= 0 && k >= 0 && l >= 0) || (i <= 0 && j <= 0 && k <= 0 && l <= 0))
		return true;
	return false;
}*/

//verify if a point is inside the obstacles
//Soraia approach (from: http://alienryderflex.com/polygon/)
//works for any polygon, therefore, there is no need to separate the Tharindu "blocks".
//  Globals which should be set before calling these functions:
//
//  int    polyCorners  =  how many corners the polygon has (no repeats)
//  float  polyX[]      =  horizontal coordinates of corners
//  float  polyY[]      =  vertical coordinates of corners
//  float  x, y         =  point to be tested
//
//  The following global arrays should be allocated before calling these functions:
//
//  float  constant[] = storage for precalculated constants (same size as polyX)
//  float  multiple[] = storage for precalculated multipliers (same size as polyX)
//
//  (Globals are used in this example for purposes of speed.  Change as
//  desired.)
//
//  USAGE:
//  Call precalc_values() to initialize the constant[] and multiple[] arrays,
//  then call pointInPolygon(x, y) to determine if the point is in the polygon.
//
//  The function will return YES if the point x,y is inside the polygon, or
//  NO if it is not.  If the point is exactly on the edge of the polygon,
//  then the function may return YES or NO.
//
//  Note that division by zero is avoided because the division is protected
//  by the "if" clause which surrounds it.

void Simulation::PreCalcValues() {
	int   i, j = polygonX.size() - 1;

	for (i = 0; i < polygonX.size(); i++) {
		if (polygonZ[j] == polygonZ[i]) {
			constant.push_back(polygonX[i]);
			multiple.push_back(0);
		}
		else {
			constant.push_back(polygonX[i] - (polygonZ[i] * polygonX[j]) / (polygonZ[j] - polygonZ[i]) + (polygonZ[i] * polygonX[i]) /
				(polygonZ[j] - polygonZ[i]));
			multiple.push_back((polygonX[j] - polygonX[i]) / (polygonZ[j] - polygonZ[i]));
		}
		j = i;
	}
}

bool Simulation::InsideObstacle(float pX, float pY, float pZ) {
	int   i, j = polygonX.size() - 1;
	bool  oddNodes = false;

	for (i = 0; i < polygonX.size(); i++) {
		if ((polygonZ[i] < pZ && polygonZ[j] >= pZ
			|| polygonZ[j] < pZ && polygonZ[i] >= pZ)) {
			oddNodes ^= (pZ*multiple[i] + constant[i] < pX);
		}
		j = i;
	}

	return oddNodes;
}