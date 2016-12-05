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

	//@TODO: see how to draw the obstacles and interact with them
	//allObstacles = GameObject.FindGameObjectsWithTag("Obstacle");

	//if loadConfigFile is checked, we do not generate the initial scenario. We load it from the Config.csv file, as well from other defined files
	if (loadConfigFile)
	{
		//draw obstacles, auxins, cells and goals
		LoadCellsAuxins();

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

		//ReadOBJFile();
		DrawObstacles();
		//std::cout << "Qnt Obstacles: " << obstacles.size() << "\n";
		//std::cout << "Vertice: " << obstacles[0].verticesX[0] << "\n";
		DrawCells();
		PlaceAuxins();
		std::cout << "Qnt Cells: " << cells.size() << "\n";

		//instantiante some goals
		DrawGoal("Restaurant", 3, 0, 18);
		DrawGoal("Theater", 27, 0, 17);
		DrawGoal("Stadium", 5, 0, 3);
		DrawGoal("AppleStore", 25, 0, 5);

		//instantiate the goal's signs
		for (int p = 0; p < goals.size(); p++) {
			DrawSign(goals[p].posX, goals[p].posY, goals[p].posZ, &goals[p], 1);
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
			bool pCollider = CheckObstacle(x, 0, z, "Player", 0.5f);

			//even so, if we are an obstacle, cannot instantiate either
			//just need to check for obstacle if found no player, otherwise it will not be instantiated anyway
			if (!pCollider) {
				pCollider = CheckObstacle(x, 0, z, "Obstacle", 0.1f);
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
					//add a goal
					newAgent.go.push_back(&goals[j]);
					//add a random intention
					newAgent.intentions.push_back(RandomFloat(0, (intentionThreshold - 0.01)));
					//add a random desire
					newAgent.AddDesire(RandomFloat(0, 1));
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
	//obstacles filename
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
				while (CheckObstacle(newPositionX, newPositionY, newPositionZ, "Obstacle", 0.1f)) {
					//if there is an obstacle, test with new positions
					if (!CheckObstacle(newPositionX + grain, newPositionY, newPositionZ, "Obstacle", 0.1f))
					{
						newPositionX += grain;
						break;
					}
					else if (!CheckObstacle(newPositionX, newPositionY, newPositionZ - grain, "Obstacle", 0.1f))
					{
						newPositionZ -= grain;
						break;
					}
					else if (!CheckObstacle(newPositionX - grain, newPositionY, newPositionZ, "Obstacle", 0.1f))
					{
						newPositionX -= grain;
						break;
					}
					else if (!CheckObstacle(newPositionX, newPositionY, newPositionZ + grain, "Obstacle", 0.1f))
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
	//@TODO: see how to draw the obstacles and interact with them
	//ReadOBJFile();

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
				DrawGoal(entries[0], std::stof(entries[1]), 0, std::stof(entries[2]));
			}
		}
		lineCount++;
	} while (line != "" && !line.empty());
	// Done reading, close the reader and return true to broadcast success
	theReader.close();

	//instantiate the goal's signs
	for (int p = 0; p < goals.size(); p++) {
		DrawSign(goals[p].posX, goals[p].posY, goals[p].posZ, &goals[p], 1);
	}

	/*for (int i = 0; i < signs.size(); i++) {
		std::cout << signs[i].posX << "\n";
	}*/
}

//draw a goal
void Simulation::DrawGoal(std::string goalName, float goalPositionX, float goalPositionY, float goalPositionZ)
{
	Goal newGoal(goalName, goalPositionX, goalPositionY, goalPositionZ);
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
			//@TODO: see how to draw the obstacles and interact with them
			//bool collideRight = CheckObstacle(new Vector3(newPosition.x + i + cellRadius, newPosition.y, newPosition.z + j), "Obstacle", 0.1f);
			//bool collideLeft = CheckObstacle(new Vector3(newPosition.x + i - cellRadius, newPosition.y, newPosition.z + j), "Obstacle", 0.1f);
			//bool collideTop = CheckObstacle(new Vector3(newPosition.x + i, newPosition.y, newPosition.z + j + cellRadius), "Obstacle", 0.1f);
			//bool collideDown = CheckObstacle(new Vector3(newPosition.x + i, newPosition.y, newPosition.z + j - cellRadius), "Obstacle", 0.1f);

			//if did collide it all, means we have found at least 1 obstacle in each case. So, the cell is covered by an obstacle
			//otherwise, we go on
			//@TODO: see how to draw the obstacles and interact with them
			//if (!collideRight || !collideLeft || !collideTop || !collideDown)
			if (true)
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
				canIInstantiante = !CheckObstacle(x, 0, z, "Obstacle", auxinRadius);
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

		std::cout << cells[c].name << " done adding markers!\n";
	}

	/*for (int i = 0; i < cells.size(); i++) {
		//std::cout << cells[i].GetAuxins()->size() << "\n";
		std::cout << cells[i].GetAuxins()->at(0).name << "\n";
	}*/
}

//check if there is Obstacles or something on a given position
//@TODO: see how to draw the obstacles and interact with them
bool Simulation::CheckObstacle(float checkPositionX, float checkPositionY, float checkPositionZ, std::string tag, float radius)
{
	/*Collider[] hitCollider = Physics.OverlapSphere(checkPosition, radius);
	bool returning = false;

	foreach(Collider hit in hitCollider) {
		if (hit.gameObject.tag == tag)
		{
			returning = true;
			break;
		}
	}

	return returning;
	*/
	return false;
}

//save a csv config file
//files saved: Config.csv, Obstacles.csv, goals.dat
void Simulation::SaveConfigFile() {
	//config file
	std::ofstream file;
	file.open(configFilename);
	//obstacles file
	//std::ofstream fileObstacles;
	//fileObstacles.open(obstaclesFilename);
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
	//@TODO: ver como fazer os obstaculos
	/*
	if (allObstacles.Length > 0)
	{
		//separated with ;
		fileObstacles.WriteLine("qntObstacles:" + allObstacles.Length);
		//for each obstacle
		for (int i = 0; i < allObstacles.Length; i++)
		{
			//new line for the obstacle name
			fileObstacles.WriteLine("\nObstacle");
			//new line for the qnt vertices
			Vector3[] vertices = allObstacles[i].GetComponent<MeshFilter>().mesh.vertices;
			fileObstacles.WriteLine("qntVertices:" + vertices.Length);
			//for each vertice, one new line
			for (int j = 0; j < vertices.Length; j++)
			{
				fileObstacles.WriteLine(vertices[j].x + ";" + vertices[j].y + ";" + vertices[j].z);
			}

			//new line for the qnt triangles
			int[] triangles = allObstacles[i].GetComponent<MeshFilter>().mesh.triangles;
			fileObstacles.WriteLine("qntTriangles:" + triangles.Length);
			//for each triangle, one new line
			for (int j = 0; j < triangles.Length; j++)
			{
				fileObstacles.WriteLine(triangles[j]);
			}
		}
	}

	fileObstacles.Close();*/

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

				/*Collider[] lockHit = Physics.OverlapSphere(agentI.transform.position, agentRadius);
				foreach(Collider loki in lockHit)
				{
					//if it is the Player tag (agent) and it is not the agent itself and he can change position (to avoid forever changing)
					if (loki.gameObject.tag == "Player" && loki.gameObject.name != agentI.gameObject.name && agentIController.changePosition)
					{
						//the other agent will not change position in this frame
						loki.GetComponent<AgentController>().changePosition = false;
						Debug.Log(agentI.gameObject.name + " -- " + loki.gameObject.name);
						//exchange!!!
						Vector3 positionA = agentI.transform.position;
						agentI.transform.position = loki.gameObject.transform.position;
						loki.gameObject.transform.position = positionA;
					}
				}*/
			}

			//walk
			agents[i].Caminhe((double)(1.0/fps));

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
				if (agents[i].go.size() == 1)
				{
					SaveAgentsGoalFile(agents[i].name, goal->name);
					agents.erase(agents.begin() + i);
					//this agent is done. Back to the for
					continue;
				}//else, he must go to the next goal. Remove this actual goal and this intention
				else
				{
					//before we remove his actual go, we check if it is the looking for state.
					//if it is, we remove it, but add a new one, because he dont know where to go yet
					//@TODO: VER COMO FAZER O LOOKING FOR. APENAS CONTROLAR NAO VAI FUNCIONAR
					bool newLookingFor = false;
					/*if (agentIController.go[0].gameObject.tag == "LookingFor")
					{
						GameObject.Destroy(agentIController.go[0].gameObject);
						newLookingFor = true;
					}*/
					agents[i].go.erase(agents[i].go.begin());
					agents[i].intentions.erase(agents[i].intentions.begin());
					agents[i].RemoveDesire(0);

					/*if (newLookingFor) {
						//add the Looking For state, with a random position
						GameObject lookingFor = GenerateLookingFor();
						agentIController.go.Add(lookingFor);
						agentIController.intentions.Add(intentionThreshold);
						agentIController.AddDesire(1);

						//since we have a new one, reorder
						agentIController.ReorderGoals();
					}*/
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

	//set triangles
	std::vector<int> triangles;
	//triangle 1
	triangles.push_back(0);
	triangles.push_back(1);
	triangles.push_back(2);
	//triangle 2
	triangles.push_back(2);
	triangles.push_back(3);
	triangles.push_back(0);

	//"draw" it
	DrawObstacle(verticesX, verticesY, verticesZ, triangles);

	//draw pentagon
	verticesX.clear();
	verticesY.clear();
	verticesZ.clear();
	triangles.clear();

	//set vertices
	//vertice 1
	verticesX.push_back(20.0f);
	verticesY.push_back(0.0f);
	verticesZ.push_back(15.0f);
	//vertice 2
	verticesX.push_back(18.0f);
	verticesY.push_back(0.0f);
	verticesZ.push_back(18.0f);
	//vertice 3
	verticesX.push_back(22.0f);
	verticesY.push_back(0.0f);
	verticesZ.push_back(20.0f);
	//vertice 4
	verticesX.push_back(26.0f);
	verticesY.push_back(0.0f);
	verticesZ.push_back(18.0f);
	//vertice 4
	verticesX.push_back(24.0f);
	verticesY.push_back(0.0f);
	verticesZ.push_back(15.0f);

	//set triangles
	//triangle 1
	triangles.push_back(0);
	triangles.push_back(1);
	triangles.push_back(2);
	//triangle 2
	triangles.push_back(2);
	triangles.push_back(3);
	triangles.push_back(4);
	//triangle 3
	triangles.push_back(0);
	triangles.push_back(2);
	triangles.push_back(4);

	//"draw" it
	DrawObstacle(verticesX, verticesY, verticesZ, triangles);

	//build the navmesh at runtime
	//NavMeshBuilder.BuildNavMesh();
}

//draw each obstacle
//each polygon has vertices-2 triangles
void Simulation::DrawObstacle(std::vector<float> verticesX, std::vector<float> verticesY, std::vector<float> verticesZ, std::vector<int> triangles) {
	//new object obstacle
	std::string newName = "Obstacle" + std::to_string(obstacles.size());
	Obstacle newObstacle(newName, verticesX, verticesY, verticesZ, triangles);
	//to the global
	obstacles.push_back(newObstacle);
}
/*
//generate the metric between number of signs and time
public void GenerateMetric() {
	//get all subdirectories within the defined config directory
	allDirs = Directory.GetDirectories(Application.dataPath + "/" + allSimulations);

	//for each sim directory
	foreach(string dir in allDirs)
	{
		//we take the AgentsGoal file for information
		StreamReader theReader = new StreamReader(dir + "/AgentsGoal.csv", System.Text.Encoding.Default);
		float timeCount = 0;
		int lineCount = 0;
		string line;

		using (theReader)
		{
			do
			{
				//we get the updated simulation Index
				line = theReader.ReadLine();
				if (line != null && line != "")
				{
					string[] info = line.Split(';');
					//sums up the arrival times
					timeCount += System.Int32.Parse(info[2]);
					lineCount++;
				}
			} while (line != null);
		}
		theReader.Close();

		//now, we take the signs file for information about how many signs were
		theReader = new StreamReader(dir + "/signs.dat", System.Text.Encoding.Default);
		int qntSigns = 0;

		using (theReader)
		{
			do
			{
				//we get the updated simulation Index
				line = theReader.ReadLine();
				if (line != null && line != "")
				{
					qntSigns = System.Int32.Parse(line);
					//just need to read the first line, which has the qnt signs
					break;
				}
			} while (line != null);
		}
		theReader.Close();

		//mean time
		timeCount /= lineCount;

		//open metrci file to save info each frame
		exitFile = File.CreateText(dir + "/metric.csv");
		exitFile.WriteLine("Mean Time: " + timeCount);
		exitFile.WriteLine("Qnt Signs: " + qntSigns);
		//TODO: DEFINE FORMULA
		exitFile.WriteLine("Metric value: " + (timeCount * qntSigns));
		exitFile.Close();
	}
}

//check group vertices to find the corners
//TODO: SEE WHY IT IS CREATING MORE GROUPS THAN IT SHOULD
public void CheckGroupVertices()
{
	allObstacles = GameObject.FindGameObjectsWithTag("Obstacle");
	//first obstacle, vertices and triangles
	Vector3[] vertices = allObstacles[0].GetComponent<MeshFilter>().sharedMesh.vertices;
	int[] triangles = allObstacles[0].GetComponent<MeshFilter>().sharedMesh.triangles;

	int[] groupTriangles = new int[triangles.Length];
	//start each index with 0
	for (int i = 0; i < groupTriangles.Length; i++) {
		groupTriangles[i] = 0;
	}

	int group = 0;

	float elapsed = 0f;

	//go through the triangles checking if they are related each other
	while (true)
	{
		bool die = true;
		//int ind = 0;

		//check if we need to update group value
		for (int i = 0; i < groupTriangles.Length; i = i + 3)
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

		//if there are no more zero values, or time is up, break;
		if (die || elapsed > 600)
		{
			Debug.Log(elapsed);
			break;
		}

		//for each triangle, we get it...
		for (int i = 0; i < triangles.Length; i = i + 3)
		{
			int triangleI1 = triangles[i];
			int triangleI2 = triangles[i + 1];
			int triangleI3 = triangles[i + 2];

			//...to compare with other triangles
			for (int j = 0; j < triangles.Length; j = j + 3)
			{
				if (i != j)
				{
					int triangleJ1 = triangles[j];
					int triangleJ2 = triangles[j + 1];
					int triangleJ3 = triangles[j + 2];

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

		elapsed += Time.deltaTime;
	}

	//Debug.Log(group);

	for (int i = 0; i < groupTriangles.Length; i++)
	{
		//1 - OK
		//2 - OK
		//3 - WRONG! parte de baixo do bloco do grupo 2!
		//4 - OK
		//5 - WRONG! parte direita do bloco do grupo 4!
		//6 - OK
		//7 - WRONG! parte direita do bloco do grupo 6!
		//8 - OK
		//9 - WRONG! parte direita do bloco do grupo 8!
		//10 - OK
		//11 - WRONG! parte direita do bloco do grupo 10!
		//12 - OK
		//13 - OK
		//14 - WRONG! parte esquerda do bloco do grupo 13!
		//15 - WRONG! parte de cima do bloco do grupo 13!
		//16 - OK
		//17 - OK
		//18 - OK
		//19 - OK
		//20 - WRONG! parte direita do bloco do grupo 19!
		//21 - OK
		//22 - OK
		//23 - OK
		//24 - OK
		//25 - OK
		//26 - OK
		//27 - WRONG! parte esquerda do bloco do grupo 26!
		//OK = 18!!!
	}

	//FOR NOW: group set static, while we try to figure it out
	Vector3[] newVertices = new Vector3[group * 4]; //group*4
	int verticesIndex = 0;
	int groupIT = 1;
	while (groupIT <= group) {
		//FOR NOW: get the wrong groups out, while we try to figure it out
		if (groupIT != 3 && groupIT != 5 && groupIT != 7 && groupIT != 9 && groupIT != 11 && groupIT != 14 && groupIT != 15 && groupIT != 20 &&
			groupIT != 27 || true)
		{
			//the boundary
			float maxX = 0;
			float maxZ = 0;
			float minX = 0;
			float minZ = 0;

			bool first = true;
			//here, we get the boundary
			for (int i = 0; i < groupTriangles.Length; i++)
			{
				if (groupTriangles[i] == groupIT)
				{
					if (first)
					{
						maxX = vertices[triangles[i]].x;
						maxZ = vertices[triangles[i]].z;
						minX = vertices[triangles[i]].x;
						minZ = vertices[triangles[i]].z;
						first = false;
					}
					else
					{
						if (vertices[triangles[i]].x > maxX)
						{
							maxX = vertices[triangles[i]].x;
						}
						if (vertices[triangles[i]].x < minX)
						{
							minX = vertices[triangles[i]].x;
						}
						if (vertices[triangles[i]].z > maxZ)
						{
							maxZ = vertices[triangles[i]].z;
						}
						if (vertices[triangles[i]].z < minZ)
						{
							minZ = vertices[triangles[i]].z;
						}
					}
				}
			}

			//lists for the vertices which belong the boundary
			List<Vector3> allMinX = new List<Vector3>();
			List<Vector3> allMaxX = new List<Vector3>();
			List<Vector3> allMinZ = new List<Vector3>();
			List<Vector3> allMaxZ = new List<Vector3>();
			for (int i = 0; i < groupTriangles.Length; i++)
			{
				if (groupTriangles[i] == groupIT)
				{
					//new idea: find all vertices with minX, maxX, etc; and use them directly, with no bounding box
					if (vertices[triangles[i]].x == minX) {
						//if it is not in the list yet, add
						if (!allMinX.Contains(vertices[triangles[i]]))
						{
							allMinX.Add(vertices[triangles[i]]);
						}
					}
					if (vertices[triangles[i]].x == maxX)
					{
						//if it is not in the list yet, add
						if (!allMaxX.Contains(vertices[triangles[i]]))
						{
							allMaxX.Add(vertices[triangles[i]]);
						}
					}
					if (vertices[triangles[i]].z == minZ)
					{
						//if it is not in the list yet, add
						if (!allMinZ.Contains(vertices[triangles[i]]))
						{
							allMinZ.Add(vertices[triangles[i]]);
						}
					}
					if (vertices[triangles[i]].z == maxZ)
					{
						//if it is not in the list yet, add
						if (!allMaxZ.Contains(vertices[triangles[i]]))
						{
							allMaxZ.Add(vertices[triangles[i]]);
						}
					}
				}
			}

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
				if (allMaxX.Count == 1)
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
			}
		}

		groupIT++;
	}

	verticesObstacles = newVertices;
}

//TEST TO READ THE 4X4.OBJ
public void ReadOBJFile()
{
	StreamReader theReader = new StreamReader(Application.dataPath + "/Original/StreetBlock.obj", System.Text.Encoding.Default);
	string line;
	int qntVertices = 0;
	int qntTriangles = 0;
	Vector3[] vertices = new Vector3[qntVertices];
	int[] triangles = new int[qntTriangles];
	int controlVertice = 0;
	int controlTriangle = 0;

	using (theReader)
	{
		int lineCount = 1;
		// While there's lines left in the text file, do this:
		do
		{
			line = theReader.ReadLine();

			if (line != null)
			{
				//if it contains #, it is a comment
				if (line.Contains("#"))
				{
					if (line.Contains("vertices"))
					{
						string[] info = line.Split(' ');
						qntVertices = System.Int32.Parse(info[1]);
						vertices = new Vector3[qntVertices];
					}
					if (line.Contains("facets"))
					{
						string[] info = line.Split(' ');
						qntTriangles = System.Int32.Parse(info[1]);
						triangles = new int[qntTriangles * 3];
					}
				}
				else if (line != "")
				{
					string[] entries = line.Split(' ');
					//if it starts with v, it is vertice. else, if it starts with f, it is facet which form a triangle (hopefully!)
					if (entries[0] == "v")
					{
						vertices[controlVertice] = new Vector3(System.Convert.ToSingle(entries[1]), System.Convert.ToSingle(entries[2]), System.Convert.ToSingle(entries[3]));
						controlVertice++;
					}
					else if (entries[0] == "f")
					{
						triangles[controlTriangle] = System.Int32.Parse(entries[2]) - 1;
						controlTriangle++;

						triangles[controlTriangle] = System.Int32.Parse(entries[3]) - 1;
						controlTriangle++;

						triangles[controlTriangle] = System.Int32.Parse(entries[4]) - 1;
						controlTriangle++;
					}
				}
			}

			lineCount++;
		} while (line != null);
	}

	DrawObstacle(vertices, triangles);

	// Done reading, close the reader and return true to broadcast success
	theReader.Close();
}*/

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