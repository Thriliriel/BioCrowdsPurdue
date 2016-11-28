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

	scenarioSizeX = mapSizeX;
	scenarioSizeZ = mapSizeZ;

	//get all subdirectories within the defined config directory
	_finddata_t data;
	std::string path = allSimulations + "*.*";
	int ff = _findfirst(path.c_str(), &data);
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
	spawnPositionX = (int)floor(scenarioSizeX - 1);
	//spawnPositionZ = (int)Mathf.Floor(scenarioSizeZ - 1));//anywhere
	spawnPositionZ = (int)floor(scenarioSizeZ - (scenarioSizeZ - 2));//to make agents start near the bottom

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

		//@TODO: see how to draw the obstacles and interact with them
		//ReadOBJFile();
		DrawCells();
		PlaceAuxins();

		/*
		GameObject agents = GameObject.Find("Agents");
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

			//default
			//scenarioType 0
			//sort out a cell
			int cellIndex = Random.Range(0, allCells.Length - 1);

			//Debug.Log (x+"--"+z);
			GameObject[] allGoals = GameObject.FindGameObjectsWithTag("Goal");
			GameObject thisGoal = allGoals[0];

			//default
			GameObject foundCell = allCells[cellIndex];

			//generate the agent position
			float x = Random.Range(foundCell.transform.position.x - cellRadius, foundCell.transform.position.x + cellRadius);
			float z = Random.Range(foundCell.transform.position.z - cellRadius, foundCell.transform.position.z + cellRadius);

			//see if there are agents in this radius. if not, instantiante
			bool pCollider = CheckObstacle(new Vector3(x, 0, z), "Player", 0.5f);

			//even so, if we are an obstacle, cannot instantiate either
			//just need to check for obstacle if found no player, otherwise it will not be instantiated anyway
			if (!pCollider) {
				pCollider = CheckObstacle(new Vector3(x, 0, z), "Obstacle", 0.1f);
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
				GameObject newAgent = Instantiate(agent, new Vector3(x, 0f, z), Quaternion.identity) as GameObject;
				AgentController newAgentController = newAgent.GetComponent<AgentController>();
				//change his name
				newAgent.name = "agent" + i;
				//random agent color
				newAgentController.SetColor(new Color(Random.Range(0.0f, 1.0f), Random.Range(0.0f, 1.0f), Random.Range(0.0f, 1.0f)));
				//agent cell
				newAgentController.SetCell(foundCell);
				//agent radius
				newAgentController.agentRadius = agentRadius;

				newAgent.GetComponent<MeshRenderer>().material.color = newAgentController.GetColor();

				//agent goals
				for (int j = 0; j < allGoals.Length; j++)
				{
					newAgentController.go.Add(allGoals[j]);
					//add a random intention
					newAgentController.intentions.Add(Random.Range(0f, (intentionThreshold - 0.01f)));
					//add a random desire
					newAgentController.AddDesire(Random.Range(0f, 1f));
				}

				//add the Looking For state, with a random position
				GameObject lookingFor = GenerateLookingFor();
				newAgentController.go.Add(lookingFor);
				newAgentController.intentions.Add(intentionThreshold);
				newAgentController.AddDesire(1);

				//reorder following intentions
				newAgentController.ReorderGoals();
				//change parent
				newAgent.transform.parent = agents.transform;

				//agent ready, break!
				//break;
			}
		}

		//instantiante some signs
		Vector3 newVertice1 = verticesObstacles[Random.Range(0, verticesObstacles.Length - 1)];
		Vector3 newVertice2 = verticesObstacles[Random.Range(0, verticesObstacles.Length - 1)];

		//it is working, but has problems if the obstacle is rotated (i.e. 180 degrees)
		//Debug.Log(newVertice1+" -- "+ newVertice2);

		DrawSign(newVertice1, GameObject.Find("Goal1"), Random.Range(0f, 1f));
		DrawSign(newVertice2, GameObject.Find("Goal2"), Random.Range(0f, 1f));*/
	}

	//get all cells in scene
	//allCells = GameObject.FindGameObjectsWithTag("Cell");*/
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
}

//checks if simulation is over
//for that, we check if is there still an agent in the scene
//if there is none, we clear the scene, update the index, get the new set of files to setup the new simulation and start
void Simulation::EndSimulation() {
	/*GameObject[] allAgents = GameObject.FindGameObjectsWithTag("Player");
	if (allAgents.Length == 0) {
		//close exit file
		exitFile.Close();
		//close exit agents/goal file
		agentsGoalFile.Close();

		//update simulation index
		simulationIndex++;

		//if index is >= than allDirs, we have reached the end. So, GAME OVER!!!
		if (simulationIndex >= allDirs.Length)
		{
			gameOver = true;
			Debug.Log("FINISHED!!");

			//reset the iterator
			var file = File.CreateText(Application.dataPath + "/SimulationIterator.txt");
			file.WriteLine("0");
			file.Close();

			//reset the frame count
			file = File.CreateText(Application.dataPath + "/FrameCount.txt");
			file.WriteLine("0");
			file.Close();
		}
		else
		{
			//else, we keep it going
			var file = File.CreateText(Application.dataPath + "/SimulationIterator.txt");
			file.WriteLine(simulationIndex.ToString());
			file.Close();

			//update last frame count
			file = File.CreateText(Application.dataPath + "/FrameCount.txt");
			file.WriteLine(Time.frameCount.ToString());
			file.Close();

			//reset scene
			SceneManager.LoadScene(0);
		}
	}*/
}

//control all chained simulations
//get the new set of files to setup the new simulation and start
void Simulation::LoadChainSimulation() {
	// Create a new StreamReader, tell it which file to read and what encoding the file
	std::ifstream theReader;
	std::string line;

	theReader.open(allSimulations+"SimulationIterator.txt");
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
	
	int ff = _findfirst(path.c_str(), &data);
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
		if (allFiles[i].substr(allFiles[i].find_last_of(".") + 1) == "csv" || allFiles[i].substr(allFiles[i].find_last_of(".") + 1) == "dat"){
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
	
	LoadConfigFile();
}

//load a csv config file
void Simulation::LoadConfigFile() {
	/*string line;

	//parents
	GameObject parentAgents = GameObject.Find("Agents");

	// Create a new StreamReader, tell it which file to read and what encoding the file
	//schedule file, with agents and their schedules
	StreamReader theReader = new StreamReader(scheduleFilename, System.Text.Encoding.Default);

	using (theReader)
	{
		int lineCount = 1;
		// While there's lines left in the text file, do this:
		do
		{
			line = theReader.ReadLine();

			if (line != null)
			{
				//in first line, it is the qnt agents
				if (lineCount == 1)
				{
					qntAgents = System.Int32.Parse(line);
				}
				else {
					//each line 1 agent, separated by " "
					string[] entries = line.Split(' ');

					Vector3 newPosition = new Vector3(System.Convert.ToSingle(entries[0]), agent.transform.position.y, System.Convert.ToSingle(entries[1]));
					float grain = 1f;
					float originalGrain = grain;

					//check if there is an obstacle in this position
					while (CheckObstacle(newPosition, "Obstacle", 0.1f)) {
						//if there is an obstacle, test with new positions
						if (!CheckObstacle(newPosition + new Vector3(grain, 0f, 0f), "Obstacle", 0.1f))
						{
							newPosition = newPosition + new Vector3(grain, 0f, 0f);
							break;
						}
						else if (!CheckObstacle(newPosition + new Vector3(0f, 0f, -grain), "Obstacle", 0.1f))
						{
							newPosition = newPosition + new Vector3(0f, 0f, -grain);
							break;
						}
						else if (!CheckObstacle(newPosition + new Vector3(-grain, 0f, 0f), "Obstacle", 0.1f))
						{
							newPosition = newPosition + new Vector3(-grain, 0f, 0f);
							break;
						}
						else if (!CheckObstacle(newPosition + new Vector3(0f, 0f, grain), "Obstacle", 0.1f))
						{
							newPosition = newPosition + new Vector3(0f, 0f, grain);
							break;
						}
						else
						{
							//if none, update with twice the grain to try again
							grain += originalGrain;
						}
					}

					GameObject newAgent = Instantiate(agent, newPosition, Quaternion.identity) as GameObject;
					AgentController agentFoundController = newAgent.GetComponent<AgentController>();
					//change his name
					int agentName = lineCount - 2;
					newAgent.name = "agent" + agentName;
					//change his radius
					agentFoundController.agentRadius = agentRadius;
					//change his color
					newAgent.GetComponent<MeshRenderer>().material.color = new Color(Random.Range(0.0f, 1.0f), Random.Range(0.0f, 1.0f), Random.Range(0.0f, 1.0f));
					//find the cell in x - z coords, using his name
					//use the rest of division by cellRadius*2
					int restDivisionX = (int)(newPosition.x % (cellRadius * 2));
					int restDivisionZ = (int)(newPosition.z % (cellRadius * 2));

					//Debug.Log("Agent " + newAgent.name + ": RestX- " + restDivisionX + " -- RestZ- " + restDivisionZ);
					//Debug.Log("Agent " + newAgent.name + ": newPosition.x- " + newPosition.x + " -- cellRadius- " + cellRadius);

					int nameX = (int)(newPosition.x - restDivisionX);
					int nameZ = (int)(newPosition.z - restDivisionZ);

					GameObject foundCell = GameObject.Find("cell" + nameX + "-" + nameZ);
					if (foundCell)
					{
						agentFoundController.SetCell(foundCell);
					}
					else
					{
						Debug.Log("Agent " + newAgent.name + ": Cell not found! " + nameX + " - " + nameZ);
					}
					//change parent
					newAgent.transform.parent = parentAgents.transform;

					//set his goals
					//first and second information in the file are just the coordinates, which i already have in config file
					//go 2 in 2, since it is a pair between goal and intention to that goal
					for (int j = 2; j < entries.Length; j = j + 2)
					{
						//there is an empty space on the end of the line, dont know why.
						if (entries[j] == "") continue;

						//try to find this goal object
						GameObject goalFound = GameObject.Find("Goal" + entries[j]);

						if (goalFound)
						{
							agentFoundController.go.Add(goalFound);
							agentFoundController.intentions.Add(System.Convert.ToSingle(entries[j + 1]));
							//add a random desire
							agentFoundController.AddDesire(Random.Range(0f, 1f));
						}
					}

					//add the Looking For state, with a random position
					GameObject lookingFor = GenerateLookingFor();
					agentFoundController.go.Add(lookingFor);
					agentFoundController.intentions.Add(intentionThreshold);
					agentFoundController.AddDesire(1);

					//now, we need to reorder our goal/intentions array, so the agent will follow the goal of bigger intention
					agentFoundController.ReorderGoals();
				}
			}
			lineCount++;
		} while (line != null);
		// Done reading, close the reader and return true to broadcast success
		theReader.Close();
	}

	// Create a new StreamReader, tell it which file to read and what encoding the file
	//signs file, with signs and their appeals
	theReader = new StreamReader(signsFilename, System.Text.Encoding.Default);

	using (theReader)
	{
		int lineCount = 1;
		// While there's lines left in the text file, do this:
		do
		{
			line = theReader.ReadLine();

			if (line != null)
			{
				//in first line, it is the qnt agents
				if (lineCount == 1)
				{
					qntSigns = System.Int32.Parse(line);
				}
				else
				{
					//each line 1 agent, separated by " "
					string[] entries = line.Split(' ');

					//sign position
					Vector3 newPosition = new Vector3(0f, 0f, 0f);
					//define position based on obstacle vertices
					if (allObstacles.Length > 0) {
						if (allObstacles.Length == 1)
						{
							//check group vertices to find the corners
							newPosition = verticesObstacles[(int)Mathf.Floor(Random.Range(0, verticesObstacles.Length)) - 1];

							//while newPosition is inside the already used positions, we try again
							while (positionsSigns.Contains(newPosition))
							{
								newPosition = verticesObstacles[(int)Mathf.Floor(Random.Range(0, verticesObstacles.Length)) - 1];
							}
						}
						else
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
						}
					}

					//will file bring sign goal
					GameObject signGoal = GameObject.Find("Goal" + (System.Convert.ToSingle(entries[1]) - 1));

					DrawSign(newPosition, signGoal, System.Convert.ToSingle(entries[2]));

					//add this position in the list, so we dont draw here again
					positionsSigns.Add(newPosition);
				}
			}
			lineCount++;
		} while (line != null);
		// Done reading, close the reader and return true to broadcast success
		theReader.Close();
		//Need it no more
		positionsSigns.Clear();
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

	/*for (int i = 0; i < signs.size(); i++) {
		std::cout << signs[i].posX << "\n";
	}*/
}

//draw a goal
void Simulation::DrawGoal(std::string goalName, float goalPositionX, float goalPositionY, float goalPositionZ)
{
	Goal newGoal(goalName, goalPositionX, goalPositionY, goalPositionZ);
	goals.push_back(newGoal);

	//draw a sign on this position too, so if the agent is looking for around, he finds it
	DrawSign(goalPositionX, goalPositionY, goalPositionZ, &newGoal, 1);
}

//draw a sign
void Simulation::DrawSign(float signPositionX, float signPositionY, float signPositionZ, Goal* signGoal, float signAppeal) {
	Sign newSign(signPositionX, signPositionY, signPositionZ);
	newSign.SetGoal(signGoal);
	newSign.SetAppeal(signAppeal);
	signs.push_back(newSign);
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
			if(true)
			{
				//new cell
				Cell newCell(newPositionX+i, newPositionY, newPositionZ+j, "cell" + std::to_string(i) + "-" + std::to_string(j));
				cells.push_back(newCell);
			}
		}
	}

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

	qntAuxins = (int)floor(densityToQnt);

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
	}

	/*for (int i = 0; i < cells.size(); i++) {
		std::cout << cells[i].GetAuxins()->size() << "\n";
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

/*
void Start() {
	//all ready to go. If saveConfigFile is checked, save this config in a csv file
	if (saveConfigFile)
	{
		SaveConfigFile();
	}
}

// Update is called once per frame
void Update() {
	//if simulation should be running yet
	if (!gameOver)
	{
		//reset auxins
		//it must be here because we need to make sure they reset before calculate the new auxins
		//TODO: HERE, MAYBE WE CAN JUST FIND THE CELLS/AUXINS THAT ACTUALLY NEED TO BE RESETED! MAYBE USING AGENTS?
		//RESULT: NOPE, STILL SLOW... KEEP THIS WAY
		for (int i = 0; i < allCells.Length; i++)
		{
			List<AuxinController> allAuxins = allCells[i].GetComponent<CellController>().GetAuxins();
			for (int j = 0; j < allAuxins.Count; j++)
			{
				allAuxins[j].ResetAuxin();
			}
		}

		//find nearest auxins for each agent
		for (int i = 0; i < qntAgents; i++)
		{
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
				//find all auxins near him (Voronoi Diagram)
				agentI.GetComponent<AgentController>().FindNearAuxins(cellRadius);
			}
		}

		/*to find where the agent must move, we need to get the vectors from the agent to each auxin he has, and compare with
		the vector from agent to goal, generating a angle which must lie between 0 (best case) and 180 (worst case)
		The calculation formula was taken from the Bicho�s mastery tesis and from Paravisi algorithm, all included
		in AgentController.
		*/

		/*for each agent, we:
		1 - verify if he is in the scene. If he is...
		2 - find him
		3 - for each auxin near him, find the distance vector between it and the agent
		4 - calculate the movement vector (CalculaDirecaoM())
		5 - calculate speed vector (CalculaVelocidade())
		6 - walk (Caminhe())
		7 - verify if the agent has reached the goal. If so, destroy it //TODO: try to find a better value/way
		*//*
		for (int i = 0; i < qntAgents; i++)
		{

			//verify if agent is not destroyed
			bool destroyed = false;
			for (int j = 0; j < agentsDestroyed.Count; j++)
			{
				if (agentsDestroyed[j] == i) destroyed = true;
			}

			if (!destroyed)
			{
				//find the agent
				GameObject agentI = GameObject.Find("agent" + i);
				AgentController agentIController = agentI.GetComponent<AgentController>();
				GameObject goal = agentIController.go[0];
				List<AuxinController> agentAuxins = agentIController.GetAuxins();

				//vector for each auxin
				for (int j = 0; j < agentAuxins.Count; j++)
				{
					//add the distance vector between it and the agent
					agentIController.vetorDistRelacaoMarcacao.Add(agentAuxins[j].position - agentI.transform.position);

					//just draw the little spider legs xD
					Debug.DrawLine(agentAuxins[j].position, agentI.transform.position);
				}

				//calculate the movement vector
				agentIController.CalculaDirecaoM();
				//calculate speed vector
				agentIController.CalculaVelocidade();

				//now, we check if agent is stuck with another agent
				//if so, change places
				if (agentIController.speed.Equals(Vector3.zero))
				{
					Collider[] lockHit = Physics.OverlapSphere(agentI.transform.position, agentRadius);
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
					}
				}

				//walk
				agentIController.Caminhe();

				//verify agent position, in relation to the goal.
				//if the distance between them is less than 1 (arbitrary, maybe authors have a better solution), he arrived. Destroy it so
				float dist = Vector3.Distance(goal.transform.position, agentI.transform.position);
				if (dist < agentIController.agentRadius)
				{
					//he arrived! Lets save this on file
					//open exit file to save info each frame
					//JUST NEED TO SAVE THE LAST ONE, SO BYEEEE...
					//SaveAgentsGoalFile(agentI.name, goal.name);

					//if we are already at the last agent goal, he arrived
					//if he has 2 goals yet, but the second one is the Looking For, he arrived too
					if (agentIController.go.Count == 1 ||
						(agentIController.go.Count == 2 && agentIController.go[1].gameObject.tag == "LookingFor"))
					{
						SaveAgentsGoalFile(agentI.name, goal.name);
						agentsDestroyed.Add(i);
						Destroy(agentI);
					}//else, he must go to the next goal. Remove this actual goal and this intention
					else
					{
						//before we remove his actual go, we check if it is the looking for state.
						//if it is, we remove it, but add a new one, because he dont know where to go yet
						bool newLookingFor = false;
						if (agentIController.go[0].gameObject.tag == "LookingFor")
						{
							GameObject.Destroy(agentIController.go[0].gameObject);
							newLookingFor = true;
						}
						agentIController.go.RemoveAt(0);
						agentIController.intentions.RemoveAt(0);
						agentIController.RemoveDesire(0);

						if (newLookingFor) {
							//add the Looking For state, with a random position
							GameObject lookingFor = GenerateLookingFor();
							agentIController.go.Add(lookingFor);
							agentIController.intentions.Add(intentionThreshold);
							agentIController.AddDesire(1);

							//since we have a new one, reorder
							agentIController.ReorderGoals();
						}
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



//draw obstacles on the scene
public void DrawObstacles() {
	//draw rectangle
	Vector3[] vertices = new Vector3[4];
	vertices[0] = new Vector3(5, 0, 10);
	vertices[1] = new Vector3(5, 0, 13);
	vertices[2] = new Vector3(15, 0, 13);
	vertices[3] = new Vector3(15, 0, 10);
	DrawObstacle(vertices);

	//draw pentagon
	vertices = new Vector3[5];
	vertices[0] = new Vector3(20, 0, 15);
	vertices[1] = new Vector3(18, 0, 18);
	vertices[2] = new Vector3(22, 0, 20);
	vertices[3] = new Vector3(26, 0, 18);
	vertices[4] = new Vector3(24, 0, 15);
	DrawObstacle(vertices);

	//build the navmesh at runtime
	NavMeshBuilder.BuildNavMesh();
}

//draw each obstacle
//each polygon has vertices-2 triangles
protected void DrawObstacle(Vector3[] vertices, int[] triangles = null) {
	GameObject obstacles = GameObject.Find("Obstacles");

	GameObject go = new GameObject();
	go.transform.parent = obstacles.transform;

	go.AddComponent<MeshFilter>();
	go.AddComponent<MeshRenderer>();
	MeshFilter mf = go.GetComponent<MeshFilter>();
	var mesh = new Mesh();
	mf.mesh = mesh;

	//set the vertices
	mesh.vertices = vertices;

	if (triangles == null)
	{
		//calc the qnt triangles
		int qntTriangles = vertices.Length - 2;

		//set all triangles vertices
		int[] tri = new int[qntTriangles * 3];

		int verticeIndex = 0;
		//since when restart the verticeIndex we will not want to draw the same triangle, we use a multiply factor to get the next vertice
		//TODO: NEED TO FIND A BETTER WAY, SINCE IT WILL NOT WORK FOR EVERY CASE
		int multiplyFactor = 1;
		for (int i = 0; i < tri.Length; i++)
		{
			tri[i] = verticeIndex * multiplyFactor;
			//Debug.Log (tri[i]);
			//if it is the last one, we will use the same vertice to start next triangle
			if ((i + 1) % 3 != 0)
			{
				verticeIndex++;
			}
			//if vertice is out of bounds, back to 0
			if (verticeIndex >= vertices.Length)
			{
				verticeIndex = 0;
				multiplyFactor++;
			}
		}

		//set triangles
		mesh.triangles = tri;
	}
	else
	{
		mesh.triangles = triangles;

		//obstacle has center at 0x0, so, need to place it 500 forward
		go.transform.position = new Vector3(500f, 0, 500f);
	}

	go.AddComponent<MeshCollider>();
	//go.GetComponent<MeshCollider>().isTrigger = true;
	go.tag = "Obstacle";
	go.name = "Obstacle";

	//change the static navigation to draw it dinamically
	GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.NavigationStatic);
	GameObjectUtility.SetNavMeshArea(go, 1);
}

//save a csv config file
//files saved: Config.csv, Obstacles.csv, goals.dat
protected void SaveConfigFile() {
	//config file
	var file = File.CreateText(Application.dataPath + "/" + configFilename);
	//obstacles file
	var fileObstacles = File.CreateText(Application.dataPath + "/" + obstaclesFilename);
	//goals file
	var fileGoals = File.CreateText(Application.dataPath + "/" + goalsFilename);

	//first, we save the terrain dimensions
	Terrain terrain = GameObject.Find("Terrain").GetComponent<Terrain>();
	file.WriteLine("terrainSize:" + terrain.terrainData.size.x + "," + terrain.terrainData.size.z);

	//then, camera position and height
	GameObject camera = GameObject.Find("Camera");
	file.WriteLine("camera:" + camera.transform.position.x + "," + camera.transform.position.y + "," +
		camera.transform.position.z + "," + camera.GetComponent<Camera>().orthographicSize);

	List<AuxinController> allAuxins = new List<AuxinController>();

	//get cells info
	GameObject[] allCells = GameObject.FindGameObjectsWithTag("Cell");
	if (allCells.Length > 0)
	{
		//each line: name, positionx, positiony, positionz, cell radius
		//separated with ;

		file.WriteLine("qntCells:" + allCells.Length);
		//for each auxin
		for (int i = 0; i < allCells.Length; i++)
		{
			file.WriteLine(allCells[i].name + ";" + allCells[i].transform.position.x + ";" + allCells[i].transform.position.y +
				";" + allCells[i].transform.position.z + ";" + cellRadius);

			//add all cell auxins to write later
			List<AuxinController> allCellAuxins = allCells[i].GetComponent<CellController>().GetAuxins();
			for (int j = 0; j < allCellAuxins.Count; j++)
			{
				//Debug.Log(allCellAuxins[j].name+" -- "+ allCellAuxins[j].position);
				allAuxins.Add(allCellAuxins[j]);
			}
		}
	}

	//get auxins info
	if (allAuxins.Count > 0) {
		//each line: name, positionx, positiony, positionz, auxinRadius, cell
		//separated with ;

		file.WriteLine("qntAuxins:" + allAuxins.Count);
		//for each auxin
		for (int i = 0; i < allAuxins.Count; i++) {
			file.WriteLine(allAuxins[i].name + ";" + allAuxins[i].position.x + ";" + allAuxins[i].position.y +
				";" + allAuxins[i].position.z + ";" + auxinRadius + ";" + allAuxins[i].GetCell().name);
		}
	}

	file.Close();

	//get obstacles info
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

	fileObstacles.Close();

	//get goals info
	GameObject[] allGoals = GameObject.FindGameObjectsWithTag("Goal");
	if (allGoals.Length > 0)
	{
		//separated with " "
		fileGoals.WriteLine(allGoals.Length);
		//for each goal
		for (int i = 0; i < allGoals.Length; i++)
		{
			//new line for the goal name and position
			fileGoals.WriteLine(allGoals[i].name + " " + allGoals[i].transform.position.x + " " + allGoals[i].transform.position.z);
		}
	}

	fileGoals.Close();
}

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

//generates a new looking for GameObject
private GameObject GenerateLookingFor() {
	GameObject lookingFor = new GameObject();
	lookingFor.name = "LookingFor";
	lookingFor.tag = "LookingFor";
	bool foundPosition = false;
	Vector3 LFPosition = Vector3.zero;
	//while bool. Use this to check for possible obstacles.
	while (!foundPosition)
	{
		foundPosition = true;
		LFPosition = new Vector3(Random.Range(0f, terrain.terrainData.size.x),
			0f, Random.Range(0f, terrain.terrainData.size.z));
		foundPosition = !CheckObstacle(LFPosition, "Obstacle", 0.1f);
	}
	lookingFor.transform.position = LFPosition;
	return lookingFor;
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

//save a csv exit file, with positions of all agents in function of time
protected void SaveExitFile() {
	//get agents info
	GameObject[] allAgents = GameObject.FindGameObjectsWithTag("Player");
	if (allAgents.Length > 0)
	{
		//each line: frame, agents name, positionx, positiony, positionz, goal object name, cell name
		//separated with ;
		//for each agent
		for (int i = 0; i < allAgents.Length; i++)
		{
			exitFile.WriteLine(Time.frameCount - lastFrameCount + ";" + allAgents[i].name + ";" + allAgents[i].transform.position.x + ";" +
				allAgents[i].transform.position.y + ";" + allAgents[i].transform.position.z + ";" +
				allAgents[i].GetComponent<AgentController>().go[0].name + ";" +
				allAgents[i].GetComponent<AgentController>().GetCell().name);
		}
	}
}

protected void SaveAgentsGoalFile(string agentName, string goalName) {
	//we save: Agent name, Goal name, Time he arrived
	agentsGoalFile.WriteLine(agentName + ";" + goalName + ";" + (Time.frameCount - lastFrameCount));
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