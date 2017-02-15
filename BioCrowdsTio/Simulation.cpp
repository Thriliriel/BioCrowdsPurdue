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

Simulation::Simulation(float mapSizeX, float mapSizeZ, float newCellRadius, int argcp, char **argv) {
	//start with default values
	DefaultValues();
	
	std::cout << "STARTING TO DEPLOY!!\n";

	scenarioSizeX = mapSizeX;
	scenarioSizeZ = mapSizeZ;
	cellRadius = newCellRadius;

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

	//if loadConfigFile is checked, we do not generate the initial scenario. We load it from the Config.csv file, as well from other defined files
	if (loadConfigFile)
	{
		//draw obstacles, auxins, cells and goals
		LoadCellsAuxins();

		//check group vertices
		CheckGroupVertices();
		//std::cout << verticesObstaclesX.size() << " -- " << verticesObstaclesZ.size() << "\n";

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
		//ReadOBJFile();
		//check group vertices
		CheckGroupVertices();

		//new obstacles
		DrawObstacles();
		//std::cout << verticesObstaclesX.size() << " -- " << verticesObstaclesZ.size() << "\n";

		//std::cout << "Qnt Obstacles: " << obstacles.size() << "\n";
		//std::cout << "Vertice: " << obstacles[0].verticesX[0] << "\n";

		//draw cells
		DrawCells();

		//place the markers
		//PlaceAuxins();
		PlaceAuxinsAsGrid();
		std::cout << "Qnt Cells: " << cells.size() << "\n";

		//instantiante some goals
		DrawGoal("Goal0", Vector3(50, 0, 50), false);
		DrawGoal("Goal1", Vector3(42, 0, 30), false);
		DrawGoal("Goal2", Vector3(5, 0, 80), false);
		DrawGoal("Goal3", Vector3(92, 0, 16), false);

		//generate all looking for states
		GenerateLookingFor();

		//instantiate the goal's signs
		for (int p = 0; p < goals.size(); p++) {
			if (!goals[p].isLookingFor) {
				DrawSign(goals[p].position, &goals[p], 1);
			}
		}

		//std::cout << goals.size() << " -- " << signs.size();

		//first, start the groups
		//for default, has no groups (1 agent per group)
		int qntGroupAgents = qntAgents;
		//if qntgroups is higher than zero, and lower than qntAgents, we start this qnt of groups
		if (qntGroups > 0 && qntGroups < qntAgents) {
			qntGroupAgents = qntGroups;
		}
		for (int i = 0; i < qntGroupAgents; i++)
		{
			//sort out a cell
			int cellIndex = (int)round(RandomFloat(0, cells.size() - 0.1f));
			Vector3 newPosition;
			bool pCollider = true;
			int tries = 0;

			while (pCollider) {
				if (tries > 50) {
					//sort out a new cell
					cellIndex = (int)round(RandomFloat(0, cells.size() - 0.1f));
					tries = 0;
				}
				//generate the group position
				newPosition = Vector3(RandomFloat(cells[cellIndex].position.x - cellRadius, cells[cellIndex].position.x + cellRadius), 0, 
					RandomFloat(cells[cellIndex].position.z - cellRadius, cells[cellIndex].position.z + cellRadius));
				pCollider = InsideObstacle(newPosition);
				if (pCollider) {
					tries++;
				}
			}

			//new group center position
			//varying group max speed: 0.5f * (i + 1) * (i + 1)
			AgentGroup newAgentGroup(newPosition, &cells[cellIndex], 1.5f);

			//if using hofstede
			if (useHofstede) {
				//calculate the group hofstede
				//start the hofstede
				Hofstede hof(1);
				//Hofstede::CalculateHofstede(int pdi, int mas, int lto, int ing)
				hof.CalculateHofstede((i + 1) * (i + 1) * (i + 1) * (i + 1) * (i + 1) * (i + 1), 1, 1, 1);

				//static values for cohesion
				if (i == 0) {
					hof.SetMeanCohesion(0.5f);
				}
				else {
					hof.SetMeanCohesion(2.5f);
				}

				newAgentGroup.SetHofstede(hof);
			}//else, start it empty
			else {
				newAgentGroup.SetHofstede(Hofstede());
			}

			//agent group goals
			for (int j = 0; j < goals.size(); j++)
			{
				//if goal is not looking for...
				if (!goals[j].isLookingFor) {
					//add a goal
					newAgentGroup.go.push_back(&goals[j]);
					//add a random intention
					newAgentGroup.intentions.push_back(RandomFloat(0, (intentionThreshold - 0.01)));
					//add a random desire
					newAgentGroup.desire.push_back(RandomFloat(0, 1));
				}
			}

			//get the first non taken looking for state
			for (int j = 0; j < goals.size(); j++)
			{
				//if goal is looking for and is free...
				if (goals[j].isLookingFor && !goals[j].isTaken) {
					//add a goal
					newAgentGroup.go.push_back(&goals[j]);
					//add intention 0.8
					newAgentGroup.intentions.push_back(0.8);
					//add desire 1
					newAgentGroup.desire.push_back(1);
					//this looking for goal is taken now
					goals[j].isTaken = true;
					//already have one, get out!
					break;
				}
			}

			//reorder following intentions
			newAgentGroup.ReorderGoals();

			agentsGroups.push_back(newAgentGroup);
		}

		//to avoid a freeze
		int doNotFreeze = 0;
		//instantiate qntAgents Agents
		for (int i = 0; i < qntAgents; i++)
		{
			int groupIndex = 0;
			//we need to make sure that all groups have, at least, 1 agent
			bool alreadyOne = true;
			for (int j = 0; j < agentsGroups.size(); j++) {
				if (agentsGroups[j].agents.size() == 0) {
					groupIndex = j;
					alreadyOne = false;
					break;
				}
			}

			//if all groups already have at least 1 agent, sort out
			if (alreadyOne) {
				//sort out a group for this agent
				groupIndex = (int)RandomFloat(0, agentsGroups.size() - 0.128f);
			}

			//if we are not finding space to set the agent, thats it
			if (doNotFreeze > qntAgents) {
				std::cout << "There is no enough space for all agents.\n";
			}

			float meanDist = 0.1f;
			//use the mean distance from group Hofstead
			if (useHofstede) {
				meanDist = agentsGroups[groupIndex].GetHofstede().GetMeanDist();
			}

			//generate the agent position
			Vector3 newAgentPosition = GeneratePosition(groupIndex, !alreadyOne);
			
			Agent newAgent(newAgentPosition, "agent" + std::to_string(i));
			//agent cell
			newAgent.SetCell(agentsGroups[groupIndex].cell);
			//agent radius
			newAgent.agentRadius = agentRadius;
			//group max speed
			newAgent.maxSpeed = agentsGroups[groupIndex].GetMaxSpeed();

			//add to group
			newAgent.groupIndex = groupIndex;
			agentsGroups[groupIndex].agents.push_back(newAgent);
		}
	}
	for (int i = 0; i < agentsGroups.size(); i++) {
		for (int j = 0; j < agentsGroups[i].agents.size(); j++) {
			std::cout << agentsGroups[i].agents[j].name << ": PosX - " << agentsGroups[i].agents[j].position.x << " -- PosZ - " << agentsGroups[i].agents[j].position.z << "\n";
		}
	}
	/*for (int i = 0; i < agents[0].go.size(); i++) {
		std::cout << agents[0].go[i]->name << ": PosX - " << agents[0].go[i]->posX << " -- PosZ - " << agents[0].go[i]->posZ << " -- Intention: " << agents[0].intentions[i] << "\n";
	}*/

	/*for (int p = 0; p < signs.size(); p++) {
		std::cout << signs[p].GetGoal()->name << "\n";
	}*/

	//just need to triangulate if using A*
	if (useAStar) {
		//triangulate the scene
		std::vector<Vector3> points;

		//scenario boundaries
		points.push_back(Vector3(0, 0, 0));
		points.push_back(Vector3(0, 0, scenarioSizeZ));
		points.push_back(Vector3(scenarioSizeX, 0, scenarioSizeZ));
		points.push_back(Vector3(scenarioSizeX, 0, 0));

		//add the points
		for (int o = 0; o < obstacles.size(); o++) {
			for (int q = 0; q < obstacles[o].size(); q++) {
				points.push_back(Vector3(obstacles[o][q].x, 0, obstacles[o][q].z));
			}
		}

		Delaunay triangulation;
		std::vector<Triangle> triangles = triangulation.triangulate(points);

		//calculate the mean points of each triangle
		//here we prepare the graph
		CalculateMeanPoints(&triangles);

		//done, clear it
		points.clear();
		triangles.clear();
		//end triangulate the scene

		//create the graph nodes - FOR GRID GRAPH
		/*for (float j = 0; j < scenarioSizeZ; j = j + nodeSize) {
			for (float i = 0; i < scenarioSizeX; i = i + nodeSize) {
				int weight = 1;

				if (InsideObstacle(i, 0, j)) {
					weight = 9;
				}

				graphNodes.push_back(weight);
			}
		}*/

		/*for (float j = 0; j < scenarioSizeZ; j = j + nodeSize) {
			for (float i = 0; i < scenarioSizeX; i = i + nodeSize) {
				//std::cout << graphNodes[(j*scenarioSizeX) + i] << "\t";
				//(((i - cellRadius) / (cellRadius * 2)) * (worldSizeZ / (cellRadius * 2))) + ((j - cellRadius) / (cellRadius * 2))
				//std::cout << i << "-" << j << "--" << ((i*scenarioSizeZ) + j) << ": " << graphNodes[(i*scenarioSizeZ) + j] << "\t";
				std::cout << i << "-" << j << "--" << (((i / nodeSize)*(scenarioSizeZ / nodeSize)) + (j / nodeSize)) << ": " << graphNodes[(((i / nodeSize)*(scenarioSizeZ / nodeSize)) + (j / nodeSize))] << "\t";
			}
			std::cout << "\n";
		}*/
		//std::cout << graphNodes.size() << "\n";

		/////////////////////////////////////////
		//A* SEARCH
		//for each agent
		std::cout << "Generating paths...\n";

		//goals repositioning
		for (int i = 0; i < goals.size(); i++) {
			//set his initial position based on the graph nodes
			float distance = scenarioSizeX;
			int index = -1;
			for (int g = 0; g < graphNodesPos.size(); g++) {
				float thisDistance = Distance(goals[i].position, graphNodesPos[g].position);
				if (thisDistance < distance) {
					index = g;
					distance = thisDistance;
				}
			}

			if (index > -1) {
				//need to reposition his sign too
				for (int s = 0; s < signs.size(); s++) {
					if (signs[s].position == goals[i].position) {
						signs[s].position = graphNodesPos[index].position;
						break;
					}
				}

				goals[i].position = graphNodesPos[index].position;
			}
		}

		for (int i = 0; i < agentsGroups.size(); i++) {
			AStarPath(&agentsGroups[i]);

			//start default values for each agent
			for (int j = 0; j < agentsGroups[i].agents.size(); j++) {
				if (agentsGroups[i].path.size() == 0) {
					agentsGroups[i].agents[j].goal = agentsGroups[i].go[0]->position;
				}
				else {
					agentsGroups[i].agents[j].goal = agentsGroups[i].path[0];
				}

				agentsGroups[i].agents[j].ClearAgent();
			}
		}
		//END A* SEARCH
		////////////////////////////////////////
		std::cout << "Paths done!\n";
	}

	//all ready to go. If saveConfigFile is checked, save this config in a csv file
	if (saveConfigFile)
	{
		std::cout << "SAVING CONFIG FILE!!\n";

		Simulation::SaveConfigFile();
	}	

	//std::system("PAUSE");

	std::cout << "STARTING TO RUN!!\n";

	//initiate both timers
	startTime = clock();
	simulationTime = ((double)clock() / CLOCKS_PER_SEC);
	//std::cout << (double)clock() / CLOCKS_PER_SEC << " -- " << simulationTime << "\n";

	//start
	StartSimulation(argcp, argv);
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
	//how much is the obstacle far away from the world origin
	obstacleDisplacement = 500;
	//what is the obstacle scale
	obstacleScale = 0.1;
	//obstacleScale = 1;
	//do we plot the scene?
	plot = true;
	//default node size
	nodeSize = 1;
	//quantity of groups that agents will form. If it is zero, means no groups will be used (like groups = false), therefore, each agent will be alone in a group
	qntGroups = 0;
	//using A*?
	useAStar = true;
	//using Hofstede?
	useHofstede = false;
}

//start the simulation and control the update
void Simulation::StartSimulation(int argcp, char **argv) {
	//fps control
	double fpsTime = 0;

	//to center at screen
	int screenExtraSize = 10;

	//Plot preparing
	//need to create the attribute here, to refer later
	// SFML window
	sf::RenderWindow window(sf::VideoMode(scenarioSizeX+screenExtraSize, scenarioSizeZ+screenExtraSize), "BioCrowds");

	//if is not to plot, we close the window
	if (!plot) {
		window.close();
	}

	//each time step, call again
	while (true) {
		//std::cout << (((float)clock() - startTime) / CLOCKS_PER_SEC) - lastFrameCount;
		//update simulation timer
		simulationTime += (((double)clock()) / CLOCKS_PER_SEC) - simulationTime;
		//std::cout << fpsTime << " -- " << simulationTime << "\n";
		//update fps timer
		fpsTime += (((double)clock()) / CLOCKS_PER_SEC) - simulationTime;

		//if time variation if bigger than defined FPS, we "reset" it and Update.
		/*if (fpsTime > (double)((double)1 / (double)fps)) {
			Update(fpsTime);
			fpsTime -= (1 / fps);
		}*/
		Update(1);

		//update plot
		if (plot) {
			if (window.isOpen()) {
				// Make the markers (yellow)
				std::vector<sf::RectangleShape*> squaresMarkers;
				// Make the agents squares (white)
				std::vector<sf::RectangleShape*> squaresAgents;
				// Make the goals squares (green)
				std::vector<sf::RectangleShape*> squaresGoals;
				// Make the signs squares (blue)
				std::vector<sf::RectangleShape*> squaresSigns;
				// Make the world lines (white)
				std::vector<std::array<sf::Vertex, 2>> linesWorld;
				// Make the obstacle lines (red)
				std::vector<std::array<sf::Vertex, 2>> linesObstacles;

				//world lines
				linesWorld.push_back({ {
						sf::Vertex(sf::Vector2f(0 + (screenExtraSize / 2), 0 + ((screenExtraSize / 2)))),
						sf::Vertex(sf::Vector2f(0 + ((screenExtraSize / 2)), scenarioSizeZ + ((screenExtraSize / 2))))
					} });
				linesWorld.push_back({ {
						sf::Vertex(sf::Vector2f(0 + ((screenExtraSize / 2)), scenarioSizeZ + ((screenExtraSize / 2)))),
						sf::Vertex(sf::Vector2f(scenarioSizeX + ((screenExtraSize / 2)), scenarioSizeX + ((screenExtraSize / 2))))
					} });
				linesWorld.push_back({ {
						sf::Vertex(sf::Vector2f(scenarioSizeX + ((screenExtraSize / 2)), scenarioSizeZ + ((screenExtraSize / 2)))),
						sf::Vertex(sf::Vector2f(scenarioSizeX + ((screenExtraSize / 2)), 0 + ((screenExtraSize / 2))))
					} });
				linesWorld.push_back({ {
						sf::Vertex(sf::Vector2f(scenarioSizeX + ((screenExtraSize / 2)), 0 + ((screenExtraSize / 2)))),
						sf::Vertex(sf::Vector2f(0 + ((screenExtraSize / 2)), 0 + ((screenExtraSize / 2))))
					} });

				//obstacles lines
				if (useAStar) {
					for (int q = 0; q < graphNodesPos.size(); q++) {
						linesObstacles.push_back({ {
								sf::Vertex(sf::Vector2f(graphNodesPos[q].v1X + ((screenExtraSize / 2)), graphNodesPos[q].v1Z + ((screenExtraSize / 2))), sf::Color::Red),
								sf::Vertex(sf::Vector2f(graphNodesPos[q].v2X + ((screenExtraSize / 2)), graphNodesPos[q].v2Z + ((screenExtraSize / 2))), sf::Color::Red)
							} });
						linesObstacles.push_back({ {
								sf::Vertex(sf::Vector2f(graphNodesPos[q].v2X + ((screenExtraSize / 2)), graphNodesPos[q].v2Z + ((screenExtraSize / 2))), sf::Color::Red),
								sf::Vertex(sf::Vector2f(graphNodesPos[q].v3X + ((screenExtraSize / 2)), graphNodesPos[q].v3Z + ((screenExtraSize / 2))), sf::Color::Red)
							} });
						linesObstacles.push_back({ {
								sf::Vertex(sf::Vector2f(graphNodesPos[q].v3X + ((screenExtraSize / 2)), graphNodesPos[q].v3Z + ((screenExtraSize / 2))), sf::Color::Red),
								sf::Vertex(sf::Vector2f(graphNodesPos[q].v1X + ((screenExtraSize / 2)), graphNodesPos[q].v1Z + ((screenExtraSize / 2))), sf::Color::Red)
							} });
					}
				}else{
					for (int o = 0; o < obstacles.size(); o++) {
						for (int q = 0; q < obstacles[o].size(); q++) {
							int nextIndex = q + 1;
							if (nextIndex >= obstacles[o].size()) {
								nextIndex = 0;
							}

							linesObstacles.push_back({ {
									sf::Vertex(sf::Vector2f(obstacles[o][q].x + ((screenExtraSize / 2)), obstacles[o][q].z + ((screenExtraSize / 2))), sf::Color::Red),
									sf::Vertex(sf::Vector2f(obstacles[o][nextIndex].x + ((screenExtraSize / 2)), obstacles[o][nextIndex].z + ((screenExtraSize / 2))), sf::Color::Red)
								} });
						}
					}
				}

				//prepare markers to draw
				/*for (int c = 0; c < cells.size(); c++) {
					for (int a = 0; a < cells[c].myAuxins.size(); a++) {
						sf::RectangleShape *c1 = new sf::RectangleShape(sf::Vector2f(1, 1));
						c1->setPosition(cells[c].myAuxins[a].position.x + ((screenExtraSize / 2)), cells[c].myAuxins[a].position.z + ((screenExtraSize / 2)));
						sf::Color *color = new sf::Color(255, 255, 0, 255);
						c1->setFillColor(*color);
						squaresMarkers.push_back(c1);
					}
				}*/
				/*for (int a = 0; a < graphNodesPos.size(); a++) {
					sf::RectangleShape *c1 = new sf::RectangleShape(sf::Vector2f(1, 1));
					c1->setPosition(graphNodesPos[a].x + ((screenExtraSize / 2)), graphNodesPos[a].z + ((screenExtraSize / 2)));
					sf::Color *color = new sf::Color(255, 255, 0, 255);
					c1->setFillColor(*color);
					squaresMarkers.push_back(c1);
				}*/
				/*for (int b = 0; b < agentsGroups.size(); b++) {
					for (int a = 0; a < agentsGroups[b].agents.size(); a++) {
						for (int h = 0; h < agentsGroups[b].agents[a].GetAuxins().size(); h++) {
							sf::RectangleShape *c1 = new sf::RectangleShape(sf::Vector2f(1, 1));
							c1->setPosition(agentsGroups[b].agents[a].GetAuxins()[h]->position.x + ((screenExtraSize / 2)), agentsGroups[b].agents[a].GetAuxins()[h]->position.z + ((screenExtraSize / 2)));
							sf::Color *color = new sf::Color(255, 255, 0, 255);
							c1->setFillColor(*color);
							squaresMarkers.push_back(c1);
						}

						sf::RectangleShape *c1 = new sf::RectangleShape(sf::Vector2f(1, 1));
						c1->setPosition(agentsGroups[b].agents[a].goal.x + ((screenExtraSize / 2)), agentsGroups[b].agents[a].goal.z + ((screenExtraSize / 2)));
						sf::Color *color = new sf::Color(255, 255, 0, 255);
						c1->setFillColor(*color);
						squaresMarkers.push_back(c1);
					}
				}*/

				//prepare agents to draw
				for (int b = 0; b < agentsGroups.size(); b++) {
					for (int a = 0; a < agentsGroups[b].agents.size(); a++) {
						sf::RectangleShape *c1 = new sf::RectangleShape(sf::Vector2f(1, 1));
						c1->setPosition(agentsGroups[b].agents[a].position.x + ((screenExtraSize / 2)), agentsGroups[b].agents[a].position.z + ((screenExtraSize / 2)));
						squaresAgents.push_back(c1);
					}
				}

				//prepare goals to draw
				for (int a = 0; a < goals.size(); a++) {
					//if it is not looking for
					if (!goals[a].isLookingFor) {
						sf::RectangleShape *c1 = new sf::RectangleShape(sf::Vector2f(1, 1));
						c1->setPosition(goals[a].position.x + ((screenExtraSize / 2)), goals[a].position.z + ((screenExtraSize / 2)));
						sf::Color *color = new sf::Color(0, 255, 0, 255);
						c1->setFillColor(*color);
						squaresGoals.push_back(c1);
					}
				}

				//prepare signs to draw
				for (int a = 0; a < signs.size(); a++) {
					//if it is not a goal sign
					if (signs[a].position.x != signs[a].GetGoal()->position.x && signs[a].position.z != signs[a].GetGoal()->position.z) {
						sf::RectangleShape *c1 = new sf::RectangleShape(sf::Vector2f(1, 1));
						c1->setPosition(signs[a].position.x + ((screenExtraSize / 2)), signs[a].position.z + ((screenExtraSize / 2)));
						sf::Color *color = new sf::Color(0, 0, 255, 255);
						c1->setFillColor(*color);
						squaresSigns.push_back(c1);
					}
				}

				//start
				sf::Event event;
				while (window.pollEvent(event))
				{
					if (event.type == sf::Event::Closed)
						window.close();
				}

				//clear window
				window.clear();

				//start to draw
				// Draw the world lines
				for (auto l = begin(linesWorld); l != end(linesWorld); l++) {
					window.draw((*l).data(), 2, sf::Lines);
				}

				// Draw the obstacles lines
				for (auto l = begin(linesObstacles); l != end(linesObstacles); l++) {
					window.draw((*l).data(), 2, sf::Lines);
				}

				// Draw the markers squares
				/*for (auto s = begin(squaresMarkers); s != end(squaresMarkers); s++) {
					window.draw(**s);
				}*/

				// Draw the agents squares
				for (auto s = begin(squaresAgents); s != end(squaresAgents); s++) {
					window.draw(**s);
				}
				
				// Draw the goals squares
				for (auto s = begin(squaresGoals); s != end(squaresGoals); s++) {
					window.draw(**s);
				}

				// Draw the signs squares
				for (auto s = begin(squaresSigns); s != end(squaresSigns); s++) {
					window.draw(**s);
				}

				window.display();
			}
		}

		//if game over, byyye
		if (gameOver) {
			break;
		}
	}
}

//checks if simulation is over
//for that, we check if is there still an agent in the scene
//just used when loadConfigFile = true
void Simulation::EndSimulation() {
	int agentsSize = 0;
	for (int i = 0; i < agentsGroups.size(); i++) {
		agentsSize += agentsGroups[i].agents.size();
	}

	if (agentsSize == 0) {
		std::cout << "Finishing Simulation " + std::to_string(simulationIndex) << "\n";

		//close exit file
		exitFile.close();
		//close exit agents/goal file
		agentsGoalFile.close();

		//save finish time
		std::ofstream timeFile;
		timeFile.open(allSimulations + "/FinishTime" + std::to_string(simulationIndex) + ".txt");
		timeFile << std::to_string((((float)clock() - startTime) / CLOCKS_PER_SEC) - lastFrameCount);
		timeFile.close();

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
			agentsGroups.clear();
			signs.clear();

			//reinstantiate the goals signs
			//instantiate the goal's signs
			for (int p = 0; p < goals.size(); p++) {
				if (!goals[p].isLookingFor) {
					DrawSign(goals[p].position, &goals[p], 1);
				}
			}

			for (int g = 0; g < goals.size(); g++) {
				goals[g].isTaken = false;
			}
			std::cout << "Loading Simulation " + std::to_string(simulationIndex) << "\n";
			LoadChainSimulation();
		}
	}
}

//control all chained simulations
//get the new set of files to setup the new simulation and start
//just used when loadConfigFile = true
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
	//since agentsgoal and exit files may not exist yet, we empty them to test after
	agentsGoalFilename = "";
	exitFilename = "";
	for (int i = 0; i < allFiles.size(); i++)
	{
		//just csv, xml and dat files
		if (allFiles[i].substr(allFiles[i].find_last_of(".") + 1) == "csv" || allFiles[i].substr(allFiles[i].find_last_of(".") + 1) == "dat"
			|| allFiles[i].substr(allFiles[i].find_last_of(".") + 1) == "xml") {
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

	//if they are empty, we restart them
	if (exitFilename.empty()) {
		schNam.clear();
		Split(scheduleFilename, '/', schNam);
		for (int f = 0; f < schNam.size()-1; f++) {
			exitFilename += schNam[f] + '/';
		}
		exitFilename += extNam[extNam.size() - 1];
	}
	if (agentsGoalFilename.empty()) {
		schNam.clear();
		Split(scheduleFilename, '/', schNam);
		for (int f = 0; f < schNam.size() - 1; f++) {
			agentsGoalFilename += schNam[f] + '/';
		}
		agentsGoalFilename += agfNam[agfNam.size() - 1];
	}
	exitFile.open(exitFilename);
	agentsGoalFile.open(agentsGoalFilename);

	//save start time
	std::ofstream timeFile;
	timeFile.open(allSimulations + "/StartTime" + std::to_string(simulationIndex) + ".txt");
	timeFile << std::to_string((((float)clock() - startTime) / CLOCKS_PER_SEC) - lastFrameCount);
	timeFile.close();

	std::cout << "Loading Files\n";

	LoadConfigFile();
}

//load a csv config file
//just used when loadConfigFile = true
void Simulation::LoadConfigFile() {
	std::string line;

	//change his name
	int agentNameCounter = 0;

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

				int qntAgentsGroup = std::stoi(entries[0]);
				Vector3 newPosition;
				//the index where the pairies goal/intentions start
				int pairIndex = 3;
				//if using Hofstede, the next 4 index are PDI, MAS, LTO and ING. So, position is right after
				if (useHofstede) {
					newPosition = Vector3(std::stof(entries[5]), 0, std::stof(entries[6]));
					pairIndex = 7;
				}//else, it is right after the qntAgents in group
				else {
					newPosition = Vector3(std::stof(entries[1]), 0, std::stof(entries[2]));
				}
				float grain = 1;
				float originalGrain = grain;

				//check if there is an obstacle in this position
				while (InsideObstacle(newPosition)) {
					//if there is an obstacle, test with new positions
					if (!InsideObstacle(Vector3(newPosition.x + grain, newPosition.y, newPosition.z)))
					{
						newPosition.x += grain;
						break;
					}
					else if (!InsideObstacle(Vector3(newPosition.x, newPosition.y, newPosition.z - grain)))
					{
						newPosition.z -= grain;
						break;
					}
					else if (!InsideObstacle(Vector3(newPosition.x - grain, newPosition.y, newPosition.z)))
					{
						newPosition.x -= grain;
						break;
					}
					else if (!InsideObstacle(Vector3(newPosition.x, newPosition.y, newPosition.z + grain)))
					{
						newPosition.z += grain;
						break;
					}
					else
					{
						//if none, update with twice the grain to try again
						grain += originalGrain;
					}
				}

				//find the cell in x - z coords, using his name
				//use the rest of division by cellRadius*2
				int restDivisionX = (int)((int)newPosition.x % (int)(cellRadius * 2));
				int restDivisionZ = (int)((int)newPosition.z % (int)(cellRadius * 2));
				int cellIndex = -1;
				int nameX = (int)(newPosition.x - restDivisionX);
				int nameZ = (int)(newPosition.z - restDivisionZ);
				for (int c = 0; c < cells.size(); c++) {
					if (cells[c].name == "cell" + std::to_string((float)nameX) + "-" + std::to_string((float)nameZ)) {
						cellIndex = c;
						break;
					}
				}

				if (cellIndex > -1) {
					//new group center position
					AgentGroup newAgentGroup(newPosition, &cells[cellIndex], 1.5f);

					//if using hofstede
					if (useHofstede) {
						//calculate the group hofstede
						//start the hofstede
						Hofstede hof;
						//Hofstede::CalculateHofstede(int pdi, int mas, int lto, int ing)
						hof.CalculateHofstede(std::stoi(entries[1]), std::stoi(entries[2]), std::stoi(entries[3]), std::stoi(entries[4]));

						newAgentGroup.SetHofstede(hof);
					}//else, start it empty
					else {
						newAgentGroup.SetHofstede(Hofstede());
					}

					//set his goals
					//go 2 in 2, since it is a pair between goal and intention to that goal
					for (int j = pairIndex; j < entries.size(); j = j + 2)
					{
						//there is an empty space on the end of the line, dont know why.
						if (entries[j] == "") continue;

						//try to find this goal object
						//the file start at 1, so we adapt since we start at 0
						if (std::stoi(entries[j]) <= goals.size())
						{
							//add a goal
							newAgentGroup.go.push_back(&goals[std::stoi(entries[j])-1]);
							//add intention
							newAgentGroup.intentions.push_back(std::stof(entries[j + 1]));
							//add a random desire
							newAgentGroup.desire.push_back(RandomFloat(0, 1));
						}
					}

					//get the first non taken looking for state
					for (int j = 0; j < goals.size(); j++)
					{
						//if goal is looking for and is free...
						if (goals[j].isLookingFor && !goals[j].isTaken) {
							//add a goal
							newAgentGroup.go.push_back(&goals[j]);
							//add intention 0.8
							newAgentGroup.intentions.push_back(0.8);
							//add desire 1
							newAgentGroup.desire.push_back(1);
							//this looking for goal is taken now
							goals[j].isTaken = true;
							//already have one, get out!
							break;
						}
					}

					//reorder following intentions
					newAgentGroup.ReorderGoals();

					agentsGroups.push_back(newAgentGroup);

					//create the agents
					for (int i = 0; i < qntAgentsGroup; i++) {
						bool alreadyOne = true;
						if (i == 0) {
							alreadyOne = false;
						}
						Vector3 newAgentPosition = GeneratePosition(agentsGroups.size()-1, !alreadyOne);

						Agent newAgent(newAgentPosition, "agent" + std::to_string(agentNameCounter));
						//agent cell
						newAgent.SetCell(agentsGroups[agentsGroups.size() - 1].cell);
						//agent radius
						newAgent.agentRadius = agentRadius;
						//group max speed
						newAgent.maxSpeed = agentsGroups[agentsGroups.size() - 1].GetMaxSpeed();

						//add to group
						newAgent.groupIndex = agentsGroups.size() - 1;
						agentsGroups[agentsGroups.size() - 1].agents.push_back(newAgent);

						agentNameCounter++;
					}
				}
				else {
					std::cout << ": " << "Celula nao encontrada! CellNameX: " + std::to_string((float)nameX) + " -- CellNameZ: "
						+ std::to_string((float)nameZ) + "\n";
					break;
				}
			}
		}
		lineCount++;
	} while (!theReader.eof());
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
			if (lineCount > 1)
			{
				//each line 1 agent, separated by " "
				std::vector<std::string> entries;
				Split(line, ' ', entries);

				//sign position
				Vector3 newPosition;
				newPosition.x = newPosition.y = newPosition.z = 0;
				//define position based on obstacle vertices
				if (obstacles.size() > 0) {
					if (obstacles.size() == 1)
					{
						//check group vertices to find the corners
						int ind = (int)(RandomFloat(0, obstacles[0].size() - 0.1f));
						newPosition = obstacles[0][ind];
						bool newPositionOK = true;

						//check every sign
						for (int p = 0; p < signs.size(); p++) {
							if (signs[p].position == newPosition) {
								newPositionOK = false;
								break;
							}
						}

						//while newPosition is inside the already used positions, we try again
						while (!newPositionOK)
						{
							ind = (int)(RandomFloat(0, obstacles[0].size() - 0.1f));
							newPosition = obstacles[0][ind];

							newPositionOK = true;

							//check every sign
							for (int p = 0; p < signs.size(); p++) {
								if (signs[p].position == newPosition) {
									newPositionOK = false;
									break;
								}
							}
						}
					}
					else
					{
						//sort out an obstacle
						int obsInd = (int)(RandomFloat(0, obstacles.size() - 0.1f));
						//sort out a vertice index for this obstacle
						int ind = (int)(RandomFloat(0, obstacles[obsInd].size() - 0.1f));

						//new position
						newPosition = obstacles[0][ind];
						bool newPositionOK = true;

						//check every sign
						for (int p = 0; p < signs.size(); p++) {
							if (signs[p].position == newPosition) {
								newPositionOK = false;
								break;
							}
						}

						//while newPosition is inside the already used positions, we try again
						while (!newPositionOK)
						{
							//sort out an obstacle
							int obsInd = (int)(RandomFloat(0, obstacles.size() - 0.1f));
							//sort out a vertice index for this obstacle
							int ind = (int)(RandomFloat(0, obstacles[obsInd].size() - 0.1f));

							//new position
							newPosition = obstacles[0][ind];
							newPositionOK = true;

							//check every sign
							for (int p = 0; p < signs.size(); p++) {
								if (signs[p].position == newPosition) {
									newPositionOK = false;
									break;
								}
							}
						}
					}
				}

				//file sign goal
				std::string signGoalName = "Goal" + std::to_string((std::stoi(entries[1])));
				//find its goal
				for (int g = 0; g < goals.size(); g++) {
					if (goals[g].name == signGoalName) {
						Vector3 signPos;
						signPos = newPosition;
						DrawSign(newPosition, &goals[g], std::stof(entries[2]));
					}
				}
			}
		}
		lineCount++;
	} while (!theReader.eof());
	// Done reading, close the reader and return true to broadcast success
	theReader.close();

	std::cout << "Qnt Signs: " << signs.size() << "\n";
	/*for (int p = 0; p < signs.size(); p++) {
		std::cout << signs[p].GetGoal()->name << "\n";
	}*/

	if (!graphNodes.empty()) {
		//calculate agents A* path
		for (int i = 0; i < qntAgents; i++) {
			//AStarPath(&agents[i]);

			//start default values
			//agents[i].Start();
		}
	}
}

//load the obstacle file
void Simulation::LoadObstacleFile() {
	// Create a new reader, tell it which file to read
	std::ifstream theReader;
	std::string line;
	int lineCount = 1;
	theReader.open(obstaclesFilename);

	int qntObstacles = 0;
	int qntVertices = 0;
	int qntTriangles = 0;
	std::vector<Vector3> vertices;
	std::vector<int> triangles;

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
				Split(line, ':', entries);

				qntObstacles = std::stoi(entries[1]);
			}
			//in the third line, we have a new obstacle
			else if (line == "Obstacle")
			{
				//reset
				qntVertices = 0;
				qntTriangles = 0;
				vertices.clear();
				triangles.clear();

				//read next line with qntVertices
				std::getline(theReader, line);
				std::vector<std::string> entries;
				Split(line, ':', entries);
				qntVertices = std::stoi(entries[1]);

				//read the next qntVertices lines for the vertices
				for (int i = 0; i < qntVertices; i++) {
					std::getline(theReader, line);
					entries.clear();
					Split(line, ';', entries);
					Vector3 vertex(std::stof(entries[0]), std::stof(entries[1]), std::stof(entries[2]));
					vertices.push_back(vertex);
				}

				//read next line with qntTriangles
				std::getline(theReader, line);
				entries.clear();
				Split(line, ':', entries);
				qntTriangles = std::stoi(entries[1]);

				//read the next qntTriangles lines for the triangles
				for (int i = 0; i < qntTriangles; i++) {
					std::getline(theReader, line);
					triangles.push_back(std::stoi(line));
				}

				//create it
				DrawObstacle(vertices, triangles);
			}
		}
		lineCount++;
	} while (!theReader.eof());
	// Done reading, close the reader and return true to broadcast success
	theReader.close();
}

//load cells and auxins and obstacles and goals (static stuff)
//just used with loadConfigFile = true
void Simulation::LoadCellsAuxins() {
	//read the obstacle file
	//ReadOBJFile();
	//DrawObstacles();
	LoadObstacleFile();

	// Create a new reader, tell it which file to read
	std::ifstream theReader;
	std::string line;
	int lineCount = 1;
	theReader.open(configFilename);

	int qntCells = 0;
	
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
						Vector3 cellPos(std::stof(entries[1]), std::stof(entries[2]), std::stof(entries[3]));
						Cell newCell(cellPos, entries[0]);
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
							Vector3 markerPos(std::stof(entries[1]), std::stof(entries[2]), std::stof(entries[3]));
							Marker newMarker(markerPos);
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
	//DrawCells();
	//PlaceAuxinsAsGrid();

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
			if (lineCount > 1)
			{
				//each line 1 agent, separated by " "
				std::vector<std::string> entries;
				Split(line, ' ', entries);

				//instantiante it
				Vector3 goalPos(std::stof(entries[1]), 0, std::stof(entries[2]));
				DrawGoal(entries[0], goalPos, false);
			}
		}
		lineCount++;
	} while (line != "" && !line.empty());
	// Done reading, close the reader and return true to broadcast success
	theReader.close();

	std::cout << "Qnt Goals: " << goals.size() << "\n";

	//generate all looking for states
	GenerateLookingFor();

	//instantiate the goal's signs
	for (int p = 0; p < goals.size(); p++) {
		if (!goals[p].isLookingFor) {
			DrawSign(goals[p].position, &goals[p], 1);
		}
	}

	/*for (int i = 0; i < signs.size(); i++) {
		std::cout << signs[i].position.x << "\n";
	}*/
}

//draw a goal
void Simulation::DrawGoal(std::string goalName, Vector3 goalPosition, bool isLF)
{
	Goal newGoal(goalName, goalPosition, isLF);
	goals.push_back(newGoal);
}

//draw a sign
void Simulation::DrawSign(Vector3 signPosition, Goal* signGoal, float signAppeal) {
	Sign newSign(signPosition);
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
	Vector3 newPosition(cellRadius, 0, cellRadius);

	for (float j = 0; j < scenarioSizeZ; j = j + cellRadius * 2)
	{
		for (float i = 0; i < scenarioSizeX; i = i + cellRadius * 2)
		{
			//verify if collides with some obstacle. We dont need cells in objects.
			//for that, we need to check all 4 sides of the cell. Otherwise, we may not have cells in some free spaces (for example, half of a cell would be covered by an obstacle, so that cell
			//would not be instantied)
			//UPDATE: WE ALWAYS CREATE THE CELL, SO WE CAN EASILY FIND THE NEIGHBOR CELLS LATER
			/*bool collideRight = InsideObstacle(newPositionX + i + cellRadius, newPositionY, newPositionZ + j + cellRadius);
			bool collideLeft = InsideObstacle(newPositionX + i - cellRadius, newPositionY, newPositionZ + j + cellRadius);
			bool collideTop = InsideObstacle(newPositionX + i + cellRadius, newPositionY, newPositionZ + j - cellRadius);
			bool collideDown = InsideObstacle(newPositionX + i - cellRadius, newPositionY, newPositionZ + j - cellRadius);*/

			//if did collide it all, means we have found at least 1 obstacle in each case. So, the cell is covered by an obstacle
			//otherwise, we go on
			//if (!collideRight || !collideLeft || !collideTop || !collideDown)
			//UPDATE: WE ALWAYS CREATE THE CELL, SO WE CAN EASILY FIND THE NEIGHBOR CELLS LATER
			if (true)
			{
				//new cell
				Cell newCell(Vector3(newPosition.x + i, 0, newPosition.z + j), "cell" + std::to_string(i) + "-" + std::to_string(j));
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
			Vector3 newPosition(RandomFloat(cells[c].position.x - cellRadius, cells[c].position.x + cellRadius), 0, RandomFloat(cells[c].position.z - cellRadius, cells[c].position.z + cellRadius));
			//std::cout << cells[c].name << ": PosX - " << cells[c].position.x << " -- " << newPosition.x << " -- " << newPosition.z << "\n";

			//see if there are auxins in this radius. if not, instantiante
			std::vector<Marker>* allAuxinsInCell = cells[c].GetAuxins();
			bool canIInstantiante = true;

			for (int j = 0; j < allAuxinsInCell->size(); j++)
			{
				float distanceAA = Distance(newPosition, (*allAuxinsInCell)[j].position);

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
				canIInstantiante = !InsideObstacle(newPosition);
			}

			//canIInstantiante???
			if (canIInstantiante)
			{
				Marker newMarker(newPosition);
				newMarker.name = "marker" + std::to_string(newPosition.x) + "-" + std::to_string(newPosition.z);
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

		//std::cout << cells[c].name << " done adding markers! Qnt markers: " << cells[c].GetAuxins()->size() << "\n";
	}

	/*for (int i = 0; i < cells.size(); i++) {
		//std::cout << cells[i].GetAuxins()->size() << "\n";
		std::cout << cells[i].GetAuxins()->at(0).name << "\n";
	}*/
}

//place auxins as a grid
void Simulation::PlaceAuxinsAsGrid() {
	//lets set the qntAuxins for each cell according the density estimation
	float densityToQnt = PORC_QTD_Marcacoes;

	densityToQnt *= (cellRadius * 2) / (2.0f * auxinRadius);
	densityToQnt *= (cellRadius * 2) / (2.0f * auxinRadius);

	qntAuxins = (int)round(densityToQnt);

	//with the qnt auxins, we found out how many markers we need on each line and column
	int qntAuxinsPerLineColumn = floor(sqrtf(qntAuxins));

	//now, find the space variation between each marker
	float spaceVariation = (cellRadius * 2) / qntAuxinsPerLineColumn;

	//if spacevariation is lower than auxinRadius, use auxinRadius
	if (spaceVariation < auxinRadius) {
		spaceVariation = auxinRadius;
	}

	//std::cout << qntAuxinsPerLineColumn << " -- " << qntAuxins << " -- " << spaceVariation << "\n";
	//for each cell, we generate his auxins
	for (int c = 0; c < cells.size(); c++)
	{
		//for each column, variating by spacevariation
		for (float j = cells[c].position.z - cellRadius; j < cells[c].position.z + cellRadius; j = j + spaceVariation) {
			//for each line, variating by spacevariation
			for (float i = cells[c].position.x - cellRadius; i < cells[c].position.x + cellRadius; i = i + spaceVariation) {
				//if it is not inside an obstacle
				if (!InsideObstacle(Vector3(i, 0, j)))
				{
					Marker newMarker(Vector3(i, 0, j));
					newMarker.name = "marker" + std::to_string(i) + "-" + std::to_string(j);
					cells[c].AddAuxin(newMarker);
				}
			}
		}

		//std::cout << cells[c].name << " done adding markers! Qnt markers: " << cells[c].GetAuxins()->size() << "\n";
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
			file << cells[i].name + ";" + std::to_string(cells[i].position.x) + ";" + std::to_string(cells[i].position.y) +
				";" + std::to_string(cells[i].position.z) + ";" + std::to_string(cellRadius) + "\n";

			//add all cell auxins to write later
			std::vector<Marker>* allCellAuxins = cells[i].GetAuxins();
			for (int j = 0; j < allCellAuxins->size(); j++)
			{
				allAuxins += (*allCellAuxins).at(j).name + ";" + std::to_string((*allCellAuxins).at(j).position.x) + ";" +
					std::to_string((*allCellAuxins).at(j).position.y) + ";" + std::to_string((*allCellAuxins).at(j).position.z) + ";" +
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
	if (obstacles.size() > 0)
	{
		//separated with ;
		fileObstacles << "qntObstacles:" << obstacles.size() << "\n";
		
		//for each obstacle
		for (int o = 0; o < obstacles.size(); o++) {
			//new line for the obstacle name
			fileObstacles << "\nObstacle\n";
			//new line for the qnt vertices
			fileObstacles << "qntVertices:" + std::to_string(obstacles[o].size()) + "\n";

			//for each vertice
			for (int i = 0; i < obstacles[o].size(); i++)
			{
				fileObstacles << std::to_string(obstacles[o][i].x) + ";0;" + std::to_string(obstacles[o][i].z) + "\n";
			}

			//new line for the qnt triangles
			fileObstacles << "qntTriangles:" + std::to_string(allTriangles[o].size()) + "\n";

			//for each triangle
			for (int i = 0; i < allTriangles[o].size(); i++)
			{
				fileObstacles << std::to_string(allTriangles[o][i]) + "\n";
			}
		}
	}

	fileObstacles.close();

	//get goals info
	if (goals.size() > 0)
	{
		//separated with " "
		std::string allGoals = "";
		int qntGoals = 0;
		
		//for each goal
		for (int i = 0; i < goals.size(); i++)
		{
			if (!goals[i].isLookingFor) {
				//new line for the goal name and position
				allGoals += goals[i].name + " " + std::to_string(goals[i].position.x) + " " + std::to_string(goals[i].position.z) + "\n";
				qntGoals++;
			}
		}

		fileGoals << std::to_string(qntGoals) + "\n";
		fileGoals << allGoals;
	}

	fileGoals.close();
}

// Update is called once per frame
void Simulation::Update(double elapsed) {
	//if simulation should be running yet
	if (!gameOver)
	{
		//update simulationTime
		simulationTime = clock() - simulationTime;

		//reset auxins
		for (int j = 0; j < agentsGroups.size(); j++) {
			for (int i = 0; i < agentsGroups[j].agents.size(); i++) {
				std::vector<Marker*> axAge = agentsGroups[j].agents[i].GetAuxins();
				for (int j = 0; j < axAge.size(); j++)
				{
					axAge[j]->ResetAuxin();
				}
			}
		}

		//find nearest auxins for each agent
		for (int j = 0; j < agentsGroups.size(); j++) {
			for (int i = 0; i < agentsGroups[j].agents.size(); i++)
			{
				//find all auxins near him (Voronoi Diagram)
				agentsGroups[j].agents[i].FindNearAuxins(cellRadius, &cells, &agentsGroups[j].agents, scenarioSizeX, scenarioSizeZ);
			}
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
		for (int f = 0; f < agentsGroups.size(); f++) {
			for (int i = 0; i < agentsGroups[f].agents.size(); i++)
			{
				//update agent
				agentsGroups[f].agents[i].Update(&signs);
				//check signs in view
				bool recalculatePath = agentsGroups[f].CheckSignsInView(&signs);
				if (recalculatePath && useAStar) {
					//need to recalculate path
					AStarPath(&agentsGroups[f]);
				}

				//find his goal
				Goal *goal = agentsGroups[f].go[0];
				std::vector<Marker*> agentAuxins = agentsGroups[f].agents[i].GetAuxins();

				//vector for each auxin
				for (int j = 0; j < agentAuxins.size(); j++)
				{
					//add the distance vector between it and the agent
					Vector3 newDistRel(agentAuxins[j]->position.x - agentsGroups[f].agents[i].position.x, agentAuxins[j]->position.y - agentsGroups[f].agents[i].position.y,
						agentAuxins[j]->position.z - agentsGroups[f].agents[i].position.z);
					agentsGroups[f].agents[i].vetorDistRelacaoMarcacao.push_back(newDistRel);
				}
				/*for (int v = 0; v < agents[i].vetorDistRelacaoMarcacao.size(); v++) {
					std::cout << agents[i].vetorDistRelacaoMarcacao[v] << "\n";
				}*/

				//calculate the movement vector
				agentsGroups[f].agents[i].CalculaDirecaoM();
				//calculate speed vector
				agentsGroups[f].agents[i].CalculaVelocidade(agentsGroups[f].position, agentsGroups[f].GetHofstede().GetMeanCohesion(), elapsed);

				//now, we check if agent is stuck with another agent
				//if so, change places
				if (agentsGroups[f].agents[i].speed == 0.0f)
				{
					//check distance between this agent and every other agent
					bool agentNear = false;
					for (int q = 0; q < agentsGroups.size(); q++) {
						for (int j = 0; j < agentsGroups[q].agents.size(); j++) {
							//if they are too near and both with zero speed, probally stuck. Swap positions if this agent may do it
							if (Distance(agentsGroups[f].agents[i].position, agentsGroups[q].agents[j].position) < 0.1f && (agentsGroups[q].agents[j].speed == 0.0f) &&
								agentsGroups[f].agents[i].changePosition && i != j) {
								std::cout << "\n LOCKED: " + agentsGroups[f].agents[i].name + " with " + agentsGroups[q].agents[j].name + "\n";
								//system("PAUSE");

								Vector3 posAux = agentsGroups[f].agents[i].position;
								agentsGroups[f].agents[i].position = agentsGroups[q].agents[j].position;
								agentsGroups[q].agents[j].position = posAux;

								//the other agent doesnt change position
								agentsGroups[q].agents[j].changePosition = false;

								agentNear = true;

								break;
							}
						}
					}

					//if agent's speed is zero and found no agent near, he is probally locked. Let's unlock him!
					if (!agentNear) {
						//first, check if he is too much time idle
						if (agentsGroups[f].agents[i].idleTimer > agentsGroups[f].agents[i].maxIdleTimer) {
							//unlock him
							UnlockAgent(&agentsGroups[f].agents[i]);
							//reset counter
							agentsGroups[f].agents[i].idleTimer = 0;
						}//otherwise, keep counting
						else {
							agentsGroups[f].agents[i].idleTimer++;
						}
					}
				}//if he moved, we reset the idleTimer too
				else {
					agentsGroups[f].agents[i].idleTimer = 0;
				}

				//walk
				agentsGroups[f].agents[i].Caminhe(elapsed);

				//std::cout << "Segundos: " << ((float)simulationT) / CLOCKS_PER_SEC << "\n";
				//std::cout << agents[i].name << ": " << agents[i].position.x << "-" << agents[i].position.z << "\n";

				//verify agent position, in relation to the goal.
				//if the distance between them is less than 1 (arbitrary, maybe authors have a better solution), he arrived. Destroy it so
				float dist = Distance(goal->position, agentsGroups[f].agents[i].position);
				
				if (dist < agentsGroups[f].agents[i].agentRadius)
				{
					//he arrived! Lets save this on file
					//open exit file to save info each frame
					//JUST NEED TO SAVE THE LAST ONE, SO BYEEEE...
					//SaveAgentsGoalFile(agentI.name, goal.name);

					//if we are already at the last agent goal, he arrived
					//if he has 2 goals yet, but the second one is the Looking For, he arrived too
					if (agentsGroups[f].go.size() == 1 || (agentsGroups[f].go.size() == 2 && agentsGroups[f].go[1]->name == "LookingFor"))
					{
						std::cout << agentsGroups[f].agents[i].name << " arrived at goal " << agentsGroups[f].go[0]->name << ", finished!!\n";
						SaveAgentsGoalFile(agentsGroups[f].agents[i].name, goal->name);
						agentsGroups[f].agents.erase(agentsGroups[f].agents.begin() + i);
						//this agent is done. Back to the for
						continue;
					}//else, he must go to the next goal. Remove this actual goal and this intention
					else
					{
						//before we remove his actual go, we check if it is the looking for state.
						//if it is, we change its position, because he doesnt know where to go yet
						if (agentsGroups[f].go[0]->name == "LookingFor") {
							ChangeLookingFor(agentsGroups[f].go[0]);
							/*for (int j = 0; j < agents[i].go.size(); j++) {
								std::cout << agents[i].go[j]->name << ": PosX - " << agents[i].go[j]->position.x << " -- PosZ - "
									<< agents[i].go[j]->position.z << " -- Intention: " << agents[i].intentions[j] << "\n";
							}*/

							//need to recalculate path
							if (useAStar) {
								AStarPath(&agentsGroups[f]);
							}
						}//else, just remove it
						else {
							std::cout << agentsGroups[f].agents[i].name << " arrived at goal " << agentsGroups[f].go[0]->name << "\n";
							agentsGroups[f].go.erase(agentsGroups[f].go.begin());
							agentsGroups[f].intentions.erase(agentsGroups[f].intentions.begin());
							agentsGroups[f].desire.erase(agentsGroups[f].desire.begin());

							//need to recalculate path
							if (useAStar) {
								AStarPath(&agentsGroups[f]);
							}

							for (int j = 0; j < agentsGroups[f].agents.size(); j++) {
								if (agentsGroups[f].path.size() == 0) {
									agentsGroups[f].agents[j].goal = agentsGroups[f].go[0]->position;
								}
								else {
									agentsGroups[f].agents[j].goal = agentsGroups[f].path[0];
								}
							}
						}
					}
				}
			}

			//after all agents walked, find the centroid of all agents to update center position of the group
			Vector3 center(0, 0, 0);
			for (int j = 0; j < agentsGroups[f].agents.size(); j++) {
				center = center + agentsGroups[f].agents[j].position;
			}
			Vector3 newPosition = center / (float)agentsGroups[f].agents.size();
			agentsGroups[f].position = newPosition;

			//need to change path node?
			agentsGroups[f].ChangePath();
		}

		//write the exit file
		SaveExitFile();

		//if there are agents no more, and it is not loaded, bye
		//get agents info
		int agentsSize = 0;
		for (int i = 0; i < agentsGroups.size(); i++) {
			agentsSize += agentsGroups[i].agents.size();
		}
		if (agentsSize == 0 && !loadConfigFile) {
			gameOver = true;
		}

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
	int agentsSize = 0;
	for (int i = 0; i < agentsGroups.size(); i++) {
		agentsSize += agentsGroups[i].agents.size();
	}
	if (agentsSize > 0)
	{
		//each line: frame, agents name, positionx, positiony, positionz, goal object name, cell name
		//separated with ;
		//for each agent
		for (int j = 0; j < agentsGroups.size(); j++) {
			for (int i = 0; i < agentsGroups[j].agents.size(); i++)
			{
				exitFile << std::to_string((((float)clock() - startTime) / CLOCKS_PER_SEC) - lastFrameCount) + ";" + agentsGroups[j].agents[i].name + ";"
					+ std::to_string(agentsGroups[j].agents[i].position.x) + ";" + std::to_string(agentsGroups[j].agents[i].position.y) + ";" + std::to_string(agentsGroups[j].agents[i].position.z) + ";" +
					agentsGroups[j].go[0]->name + ";" + agentsGroups[j].agents[i].GetCell()->name + "\n";
			}
		}
	}
}

void Simulation::SaveAgentsGoalFile(std::string agentName, std::string goalName) {
	//we save: Agent name, Goal name, Time he arrived
	agentsGoalFile << agentName + ";" + goalName + ";" + std::to_string((((float)clock() - startTime) / CLOCKS_PER_SEC) - lastFrameCount) + "\n";
}


//"draw" obstacles on the scene
void Simulation::DrawObstacles() {
	obstacles.clear();
	allTriangles.clear();
	
	/*for (int i = 0; i < verticesObstaclesX.size(); i = i + 4) {
		std::vector<float> verticesX;
		std::vector<float> verticesY;
		std::vector<float> verticesZ;
		std::vector<int> triangles;

		//set vertices
		//vertice 1
		verticesX.push_back(verticesObstaclesX[i]);
		verticesY.push_back(0.0f);
		verticesZ.push_back(verticesObstaclesZ[i]);
		//vertice 2
		verticesX.push_back(verticesObstaclesX[i+1]);
		verticesY.push_back(0.0f);
		verticesZ.push_back(verticesObstaclesZ[i+1]);
		//vertice 3
		verticesX.push_back(verticesObstaclesX[i+2]);
		verticesY.push_back(0.0f);
		verticesZ.push_back(verticesObstaclesZ[i+2]);
		if (i + 3 < verticesObstaclesX.size()) {
			//vertice 4
			verticesX.push_back(verticesObstaclesX[i + 3]);
			verticesY.push_back(0.0f);
			verticesZ.push_back(verticesObstaclesZ[i + 3]);
		}

		//triangles

		triangles.push_back(0);
		triangles.push_back(1);
		triangles.push_back(2);
		if (i + 3 < verticesObstaclesX.size()) {
			triangles.push_back(2);
			triangles.push_back(3);
			triangles.push_back(0);
		}

		//"draw" it
		DrawObstacle(verticesX, verticesY, verticesZ, triangles);
	}*/

	//draw rectangle 1
	std::vector<Vector3> vertices;
	std::vector<int> triangles;

	//set vertices
	//vertice 1
	vertices.push_back(Vector3(10.0f, 0.0f, 10.0f));
	//vertice 2
	vertices.push_back(Vector3(10.0f, 0.0f, 40.0f));
	//vertice 3
	vertices.push_back(Vector3(40.0f, 0.0f, 40.0f));
	//vertice 4
	vertices.push_back(Vector3(40.0f, 0.0f, 10.0f));

	//triangles
	triangles.push_back(0);
	triangles.push_back(1);
	triangles.push_back(2);
	triangles.push_back(2);
	triangles.push_back(3);
	triangles.push_back(0);

	//"draw" it
	DrawObstacle(vertices, triangles);
	//end rectangle 1

	//rectangle 2
	vertices.clear();
	triangles.clear();

	//set vertices
	//vertice 1
	vertices.push_back(Vector3(10.0f, 0.0f, 60.0f));
	//vertice 2
	vertices.push_back(Vector3(10.0f, 0.0f, 90.0f));
	//vertice 3
	vertices.push_back(Vector3(40.0f, 0.0f, 90.0f));
	//vertice 4
	vertices.push_back(Vector3(40.0f, 0.0f, 60.0f));

	//triangles
	triangles.push_back(0);
	triangles.push_back(1);
	triangles.push_back(2);
	triangles.push_back(2);
	triangles.push_back(3);
	triangles.push_back(0);

	//"draw" it
	DrawObstacle(vertices, triangles);
	//end rectangle 2

	//rectangle 3
	vertices.clear();
	triangles.clear();

	//set vertices
	//vertice 1
	vertices.push_back(Vector3(60.0f, 0.0f, 10.0f));
	//vertice 2
	vertices.push_back(Vector3(60.0f, 0.0f, 40.0f));
	//vertice 3
	vertices.push_back(Vector3(90.0f, 0.0f, 40.0f));
	//vertice 4
	vertices.push_back(Vector3(90.0f, 0.0f, 10.0f));

	//triangles
	triangles.push_back(0);
	triangles.push_back(1);
	triangles.push_back(2);
	triangles.push_back(2);
	triangles.push_back(3);
	triangles.push_back(0);

	//"draw" it
	DrawObstacle(vertices, triangles);
	//end rectangle 3

	//rectangle 4
	vertices.clear();
	triangles.clear();

	//set vertices
	//vertice 1
	vertices.push_back(Vector3(60.0f, 0.0f, 60.0f));
	//vertice 2
	vertices.push_back(Vector3(60.0f, 0.0f, 90.0f));
	//vertice 3
	vertices.push_back(Vector3(90.0f, 0.0f, 90.0f));
	//vertice 4
	vertices.push_back(Vector3(90.0f, 0.0f, 60.0f));

	//triangles
	triangles.push_back(0);
	triangles.push_back(1);
	triangles.push_back(2);
	triangles.push_back(2);
	triangles.push_back(3);
	triangles.push_back(0);

	//"draw" it
	DrawObstacle(vertices, triangles);
	//end rectangle 4
}

//draw each obstacle, placing its vertices on the verticesObstacles. So, signs can be instantiated on those places
void Simulation::DrawObstacle(std::vector<Vector3> vertices, std::vector<int> triangles) {
	//vertices
	std::vector<Vector3> polygon;

	for (int i = 0; i < vertices.size(); i++) {
		//polygon for InsideObstacle calculus
		polygon.push_back(vertices[i]);
	}

	obstacles.push_back(polygon);

	//triangles
	allTriangles.push_back(triangles);
}

//check group vertices to find the corners
//@TODO: JUST WORKS FOR 1 OBSTACLE! NEED TO SEE IF CAN DO FOR 2 OR MORE (not necessary for Purdue)
//just work for 1 obstacle, so, deprecated
void Simulation::CheckGroupVertices()
{
	/*
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
						minZ = polygonZ[trianglesObstacle[i]];
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
					if (!Contains(allMinX_X, allMinX_Z, polygonX[trianglesObstacle[i]], polygonZ[trianglesObstacle[i]]))
					{
						allMinX_X.push_back(polygonX[trianglesObstacle[i]]);
						allMinX_Z.push_back(polygonZ[trianglesObstacle[i]]);
					}
				}
				if (polygonX[trianglesObstacle[i]] == maxX)
				{
					//if it is not in the list yet, add
					if (!Contains(allMaxX_X, allMaxX_Z, polygonX[trianglesObstacle[i]], polygonZ[trianglesObstacle[i]]))
					{
						allMaxX_X.push_back(polygonX[trianglesObstacle[i]]);
						allMaxX_Z.push_back(polygonZ[trianglesObstacle[i]]);
					}
				}
				if (polygonZ[trianglesObstacle[i]] == minZ)
				{
					//if it is not in the list yet, add
					if (!Contains(allMinZ_X, allMinZ_Z, polygonX[trianglesObstacle[i]], polygonZ[trianglesObstacle[i]]))
					{
						allMinZ_X.push_back(polygonX[trianglesObstacle[i]]);
						allMinZ_Z.push_back(polygonZ[trianglesObstacle[i]]);
					}
				}
				if (polygonZ[trianglesObstacle[i]] == maxZ)
				{
					//if it is not in the list yet, add
					if (!Contains(allMaxZ_X, allMaxZ_Z, polygonX[trianglesObstacle[i]], polygonZ[trianglesObstacle[i]]))
					{
						allMaxZ_X.push_back(polygonX[trianglesObstacle[i]]);
						allMaxZ_Z.push_back(polygonZ[trianglesObstacle[i]]);
					}
				}
			}
		}
		
		std::vector<float> fourVerticesX;
		std::vector<float> fourVerticesZ;
		//now we have the possible vertices. Lets decide which 4 to use
		//if allMinX just have 1 vertice, just check his z value
		if (fourVerticesX.size() < 4)
		{
			if (allMinX_X.size() == 1)
			{
				//if it is not in the list yet, add
				if (!Contains(fourVerticesX, fourVerticesZ, allMinX_X[0], allMinX_Z[0]))
				{
					fourVerticesX.push_back(allMinX_X[0]);
					fourVerticesZ.push_back(allMinX_Z[0]);
				}
			}
			//else, it must already contain bottom and top
			else
			{
				//find the highest and lowest z
				float lZ_X = 1000;
				float lZ_Z = 1000;
				float hZ_X = -1000;
				float hZ_Z = -1000;

				for (int i = 0; i < allMinX_X.size(); i++) {
					if (allMinX_Z[i] < lZ_Z)
					{
						lZ_X = allMinX_X[i];
						lZ_Z = allMinX_Z[i];
					}
					if (allMinX_Z[i] > hZ_Z)
					{
						hZ_X = allMinX_X[i];
						hZ_Z = allMinX_Z[i];
					}
				}

				if(!Contains(fourVerticesX, fourVerticesZ, lZ_X, lZ_Z))
				{
					fourVerticesX.push_back(lZ_X);
					fourVerticesZ.push_back(lZ_Z);
				}
				if (!Contains(fourVerticesX, fourVerticesZ, hZ_X, hZ_Z))
				{
					fourVerticesX.push_back(hZ_X);
					fourVerticesZ.push_back(hZ_Z);
				}
			}
		}

		//if allMaxX just have 1 vertice, just check his z value
		if (fourVerticesX.size() < 4)
		{
			if (allMaxX_X.size() == 1)
			{
				if (!Contains(fourVerticesX, fourVerticesZ, allMaxX_X[0], allMaxX_Z[0]))
				{
					fourVerticesX.push_back(allMaxX_X[0]);
					fourVerticesZ.push_back(allMaxX_Z[0]);
				}
			}
			//else, it must already contain bottom and top
			else
			{
				//find the highest and lowest z
				float lZ_X = 1000;
				float lZ_Z = 1000;
				float hZ_X = -1000;
				float hZ_Z = -1000;

				for (int i = 0; i < allMaxX_X.size(); i++) {
					if (allMaxX_Z[i] < lZ_Z)
					{
						lZ_X = allMaxX_X[i];
						lZ_Z = allMaxX_Z[i];
					}
					if (allMaxX_Z[i] > hZ_Z)
					{
						hZ_X = allMaxX_X[i];
						hZ_Z = allMaxX_Z[i];
					}
				}

				if (!Contains(fourVerticesX, fourVerticesZ, lZ_X, lZ_Z))
				{
					fourVerticesX.push_back(lZ_X);
					fourVerticesZ.push_back(lZ_Z);
				}
				if (!Contains(fourVerticesX, fourVerticesZ, hZ_X, hZ_Z))
				{
					fourVerticesX.push_back(hZ_X);
					fourVerticesZ.push_back(hZ_Z);
				}
			}
		}

		if (fourVerticesX.size() < 4)
		{
			//if allMinZ just have 1 vertice, just check his x value
			if (allMinZ_X.size() == 1)
			{
				if (!Contains(fourVerticesX, fourVerticesZ, allMinZ_X[0], allMinZ_Z[0]))
				{
					fourVerticesX.push_back(allMinZ_X[0]);
					fourVerticesZ.push_back(allMinZ_Z[0]);
				}
			}
			//else, it must already contain left and right
			else
			{
				//find the highest and lowest x
				float lX_X = 1000;
				float lX_Z = 1000;
				float hX_X = -1000;
				float hX_Z = -1000;

				for (int i = 0; i < allMinZ_X.size(); i++) {
					if (allMinZ_X[i] < lX_X)
					{
						lX_X = allMinZ_X[i];
						lX_Z = allMinZ_Z[i];
					}
					if (allMinZ_X[i] > hX_X)
					{
						hX_X = allMinZ_X[i];
						hX_Z = allMinZ_Z[i];
					}
				}

				if (!Contains(fourVerticesX, fourVerticesZ, lX_X, lX_Z))
				{
					fourVerticesX.push_back(lX_X);
					fourVerticesZ.push_back(lX_Z);
				}
				if (!Contains(fourVerticesX, fourVerticesZ, hX_X, hX_Z))
				{
					fourVerticesX.push_back(hX_X);
					fourVerticesZ.push_back(hX_Z);
				}
			}
		}

		if (fourVerticesX.size() < 4)
		{
			//if allMaxZ just have 1 vertice, just check his x value
			if (allMaxZ_X.size() == 1)
			{
				if (!Contains(fourVerticesX, fourVerticesZ, allMaxZ_X[0], allMaxZ_Z[0]))
				{
					fourVerticesX.push_back(allMaxZ_X[0]);
					fourVerticesZ.push_back(allMaxZ_Z[0]);
				}
			}
			//else, it must already contain left and right
			else
			{
				//find the highest and lowest x
				float lX_X = 1000;
				float lX_Z = 1000;
				float hX_X = -1000;
				float hX_Z = -1000;

				for (int i = 0; i < allMaxZ_X.size(); i++) {
					if (allMaxZ_X[i] < lX_X)
					{
						lX_X = allMaxZ_X[i];
						lX_Z = allMaxZ_Z[i];
					}
					if (allMaxZ_X[i] > hX_X)
					{
						hX_X = allMaxZ_X[i];
						hX_Z = allMaxZ_Z[i];
					}
				}

				if (!Contains(fourVerticesX, fourVerticesZ, lX_X, lX_Z))
				{
					fourVerticesX.push_back(lX_X);
					fourVerticesZ.push_back(lX_Z);
				}
				if (!Contains(fourVerticesX, fourVerticesZ, hX_X, hX_Z))
				{
					fourVerticesX.push_back(hX_X);
					fourVerticesZ.push_back(hX_Z);
				}
			}
		}

		//reorder for x
		for (int i = 0; i < fourVerticesX.size()-1; i++) {
			if (fourVerticesX[i] > fourVerticesX[i + 1]) {
				float temp = fourVerticesX[i];
				fourVerticesX[i] = fourVerticesX[i + 1];
				fourVerticesX[i + 1] = temp;

				temp = fourVerticesZ[i];
				fourVerticesZ[i] = fourVerticesZ[i + 1];
				fourVerticesZ[i + 1] = temp;

				i = 0;
			}
		}

		//now, invert a y
		//index 2 get 3, 3 get 4 and 4 get 2
		if (fourVerticesX.size() == 4) {
			float temp2 = fourVerticesX[1];
			float temp3 = fourVerticesX[2];
			float temp4 = fourVerticesX[3];
			fourVerticesX[1] = temp3;
			fourVerticesX[2] = temp4;
			fourVerticesX[3] = temp2;

			temp2 = fourVerticesZ[1];
			temp3 = fourVerticesZ[2];
			temp4 = fourVerticesZ[3];
			fourVerticesZ[1] = temp3;
			fourVerticesZ[2] = temp4;
			fourVerticesZ[3] = temp2;
		}

		//now, assign the values
		for (int i = 0; i < fourVerticesX.size(); i++) {
			newVerticesX.push_back(fourVerticesX[i]);
			newVerticesY.push_back(0);
			newVerticesZ.push_back(fourVerticesZ[i]);
		}

		groupIT++;
	}

	std::cout << "Finished to check group vertices!\n";

	//ready to go
	verticesObstaclesX = newVerticesX;
	verticesObstaclesY = newVerticesY;
	verticesObstaclesZ = newVerticesZ;
	*/
}

//READ THE 4X4.OBJ
void Simulation::ReadOBJFile()
{
	std::ifstream theReader;
	theReader.open(obstaclesFilename);
	std::string line;
	int qntVertices = 0;
	int qntTriangles = 0;
	std::vector<Vector3> vertices;
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
					vertices.push_back(Vector3((std::stof(entries[1]) + obstacleDisplacement)*obstacleScale, (std::stof(entries[2]) + obstacleDisplacement)*obstacleScale, 
						(std::stof(entries[3]) + obstacleDisplacement)*obstacleScale));
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

	DrawObstacle(vertices, triangles);
	/*for (int v = 0; v < vertices.size(); v++) {
		std::cout << vertices[v] << "\n";
	}*/

	// Done reading, close the reader and return true to broadcast success
	theReader.close();
}

//generate a new looking for
//it will be added as a goal for each agent group. So, a simulation should generate qntAgents looking for goals
void Simulation::GenerateLookingFor() {
	int qntLF = qntAgents;
	if (qntGroups > 0 && qntGroups < qntAgents) {
		qntLF = qntGroups;
	}
	//for each agent
	for (int i = 0; i < qntLF; i++) {
		bool pCollider = true;
		//while i have an obstacle on the way
		while (pCollider) {
			//generate the new position
			Vector3 newPosition(RandomFloat(0, scenarioSizeX), 0, RandomFloat(0, scenarioSizeZ));

			//check if it is not inside an obstacle
			bool pCollider = InsideObstacle(newPosition);

			//if not, new looking for!
			if (!pCollider) {
				DrawGoal("LookingFor", newPosition, true);
				break;
			}
		}
	}
}

Vector3 Simulation::GeneratePosition(int groupIndex, bool useCenter) {
	//generate the position
	Vector3 newPosition(0, 0, 0);
	//if alreadyOne == false, it is the first agent in this group. So, we can set it on the center
	if (useCenter) {
		newPosition = agentsGroups[groupIndex].position;
	}//else, rand
	else {
		//@TODO: mean dist is with weird values
		//x = RandomFloat(agentsGroups[groupIndex].position.x - meanDist * 2, agentsGroups[groupIndex].position.x + meanDist * 2);
		//z = RandomFloat(agentsGroups[groupIndex].position.z - meanDist * 2, agentsGroups[groupIndex].position.z + meanDist * 2);
		newPosition.x = RandomFloat(agentsGroups[groupIndex].position.x - 0.2f, agentsGroups[groupIndex].position.x + 0.2f);
		newPosition.z = RandomFloat(agentsGroups[groupIndex].position.z - 0.2f, agentsGroups[groupIndex].position.z + 0.2f);
	}

	//see if there are agents in this radius. if not, instantiante
	bool pCollider = false;

	for (int j = 0; j < agentsGroups[groupIndex].agents.size(); j++) {
		//@TODO: mean dist is with weird values
		//if (Distance(newPosition, agentsGroups[groupIndex].agents[j].position) < meanDist) {
		if (Distance(newPosition, agentsGroups[groupIndex].agents[j].position) < 0.1f) {
			pCollider = true;
			break;
		}
	}

	//even so, if we are inside an obstacle, cannot instantiate either
	//just need to check for obstacle if found no player, otherwise it will not be instantiated anyway
	if (!pCollider) {
		pCollider = InsideObstacle(newPosition);
	}

	//if found a player in the radius, do not instantiante. try again
	if (pCollider)
	{
		//try again
		return GeneratePosition(groupIndex, useCenter);
	}
	else
	{
		return newPosition;
	}
}

//change the position of a looking for goal
void Simulation::ChangeLookingFor(Goal* changeLF) {
	bool pCollider = true;
	//while i have an obstacle on the way
	while (pCollider) {
		//generate the new position
		//choose a new random node
		if (useAStar) {
			int index = (int)RandomFloat(0, graphNodesPos.size() - 1);
			changeLF->position = graphNodesPos[index].position;
			break;
		}else{
			Vector3 newPosition((int)RandomFloat(0, scenarioSizeX), 0, (int)RandomFloat(0, scenarioSizeZ));

			//check if it is not inside an obstacle
			bool pCollider = InsideObstacle(newPosition);

			//if not, new looking for!
			if (!pCollider) {
				changeLF->position = newPosition;
				break;
			}
		}
	}
}

//distance between 2 points
float Simulation::Distance(Vector3 start, Vector3 end)
{
	float result = sqrt((end.x - start.x)*(end.x - start.x) + (end.y - start.y)*(end.y - start.y) + (end.z - start.z)*(end.z - start.z));

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

//verify if needle is inside arrayToSearch
bool Simulation::Contains(std::vector<float> arrayToSearch, float needle) {
	bool contains = false;
	for (int m = 0; m < arrayToSearch.size(); m++) {
		if (arrayToSearch[m] == needle) {
			contains = true;
			break;
		}
	}
	return contains;
}

//verify if needle is inside arrayToSearch AND needle2 inside arrayToSeach2
//both arrays need to have the same size!!!
bool Simulation::Contains(std::vector<float> arrayToSearch, std::vector<float> arrayToSearch2, float needle, float needle2) {
	bool contains = false;
	for (int m = 0; m < arrayToSearch.size(); m++) {
		if (arrayToSearch[m] == needle && arrayToSearch2[m] == needle2) {
			contains = true;
			break;
		}
	}
	return contains;
}

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
//  Call InsideObstacle(x, y, z) to determine if the point is in the polygon.
//
//  The function will return YES if the point x,y is inside the polygon, or
//  NO if it is not.  If the point is exactly on the edge of the polygon,
//  then the function may return YES or NO.
//
//  Note that division by zero is avoided because the division is protected
//  by the "if" clause which surrounds it.
bool Simulation::InsideObstacle(Vector3 p) {
	bool  oddNodes = false;

	for (int o = 0; o < obstacles.size(); o++) {
		int   i, j = obstacles[o].size() - 1;
		oddNodes = false;

		for (i = 0; i < obstacles[o].size(); i++) {
			if ((obstacles[o][i].z < p.z && obstacles[o][j].z >= p.z
				|| obstacles[o][j].z < p.z && obstacles[o][i].z >= p.z)
				&& (obstacles[o][i].x <= p.x || obstacles[o][j].x <= p.x)) {
				oddNodes ^= (obstacles[o][i].x + (p.z - obstacles[o][i].z) / (obstacles[o][j].z - obstacles[o][i].z)*(obstacles[o][j].x - obstacles[o][i].x) < p.x);
			}
			j = i;
		}

		if (oddNodes) return oddNodes;
	}

	return oddNodes;
}

//new version based on the triangles
//finds out if a point is inside a triangle
/*bool PointInTriangle(Vec2f p, Vec2f p0, Vec2f p1, Vec2f p2)
{
	float s = p0.y * p2.x - p0.x * p2.y + (p2.y - p0.y) * p.x + (p0.x - p2.x) * p.y;
	float t = p0.x * p1.y - p0.y * p1.x + (p0.y - p1.y) * p.x + (p1.x - p0.x) * p.y;

	if ((s < 0) != (t < 0))
		return false;

	float A = -p1.y * p2.x + p0.y * (p2.x - p1.x) + p0.x * (p1.y - p2.y) + p1.x * p2.y;
	if (A < 0.0)
	{
		s = -s;
		t = -t;
		A = -A;
	}
	return s > 0 && t > 0 && (s + t) <= A;
}

bool Simulation::InsideObstacle(float pX, float pY, float pZ) {
	//foreach triangle, check if the point is inside it or not
	int qntInside = 0;
	for (int i = 0; i < allTriangles[0].size(); i = i + 3) {
		if (PointInTriangle(Vec2f(pX, pZ), Vec2f(obstaclesX[0][allTriangles[0][i]], obstaclesZ[0][allTriangles[0][i]]), Vec2f(obstaclesX[0][allTriangles[0][i + 1]], obstaclesZ[0][allTriangles[0][i + 1]]),
			Vec2f(obstaclesX[0][allTriangles[0][i + 2]], obstaclesZ[0][allTriangles[0][i + 2]]))) {
			qntInside++;
		}
	}

	//if reading the blocks file, return true else false
	//if reading the roads file, return false else true
	if (qntInside > 0) {
		return true;
	}
	else {
		return false;
	}
}*/

//unlock agent if he stops because an obstacle
void Simulation::UnlockAgent(Agent* agentToUnlock) {
	std::cout << agentToUnlock->name << " has locked at position " << agentToUnlock->position.x << " -- " << agentToUnlock->position.z << "!\n";
	int agentDisplacement = 10;
	while (true) {
		//generate a new random position inside a radius
		float minX = agentToUnlock->position.x - agentDisplacement;
		float maxX = agentToUnlock->position.x + agentDisplacement;
		float minZ = agentToUnlock->position.z - agentDisplacement;
		float maxZ = agentToUnlock->position.z + agentDisplacement;

		if (minX < 0) minX = 0;
		if (minZ < 0) minZ = 0;
		if (maxX > scenarioSizeX) maxX = scenarioSizeX;
		if (maxZ > scenarioSizeZ) maxZ = scenarioSizeZ;

		Vector3 newPosition((int)RandomFloat(minX, maxX), 0, (int)RandomFloat(minZ, maxZ));

		//see if there are agents in this radius. if not, new position
		bool pCollider = false;
		for (int i = 0; i < agentsGroups.size(); i++) {
			for (int j = 0; j < agentsGroups[i].agents.size(); j++) {
				if (Distance(newPosition, agentsGroups[i].agents[j].position) < 0.1f) {
					pCollider = true;
					break;
				}
			}
		}

		//even so, if we are an obstacle, cannot change position either
		//just need to check for obstacle if found no player, otherwise it will not be changed anyway
		if (!pCollider) {
			pCollider = InsideObstacle(newPosition);
		}

		//if found, yay! go on
		if (!pCollider) {
			agentToUnlock->position = newPosition;

			std::cout << agentToUnlock->name << " new position: " << newPosition.x << " -- " << newPosition.z << "\n";

			//need to recalculate path
			if (useAStar) {
				AStarPath(&agentsGroups[agentToUnlock->groupIndex]);
			}

			//his next goal
			if (agentsGroups[agentToUnlock->groupIndex].path.size() > 0) {
				agentToUnlock->goal = agentsGroups[agentToUnlock->groupIndex].path[0];
			}

			break;
		}
	}
}

//calculate mean points
void Simulation::CalculateMeanPoints(std::vector<Triangle>* triangles) {
	for (auto &t : *triangles) {
		Vector3 soma((t.p1.x + t.p2.x + t.p3.x) / 3, 0, (t.p1.z + t.p2.z + t.p3.z) / 3);

		if (!InsideObstacle(soma)) {
			Node node;
			node.position = soma;
			node.v1X = t.p1.x;
			node.v1Z = t.p1.z;
			node.v2X = t.p2.x;
			node.v2Z = t.p2.z;
			node.v3X = t.p3.x;
			node.v3Z = t.p3.z;

			graphNodes.push_back(1);
			graphNodesPos.push_back(node);
		}
	}
}

//A Star Search Path
void Simulation::AStarPath(AgentGroup* agentPath) {
	//clear actual path
	agentPath->path.clear();

	//A* SEARCH
	AStarSearch<AStarSearchNode> astarsearch;

	unsigned int SearchCount = 0;

	const unsigned int NumSearches = 1;

	while (SearchCount < NumSearches)
	{
		// Create a start state
		AStarSearchNode nodeStart;

		nodeStart.position = agentPath->position;
		nodeStart.maxSizeX = scenarioSizeX;
		nodeStart.maxSizeZ = scenarioSizeZ;
		nodeStart.graphNodes = &graphNodes;
		nodeStart.nodeSize = nodeSize;
		nodeStart.graphNodesPos = &graphNodesPos;

		// Define the goal state
		AStarSearchNode nodeEnd;
		nodeEnd.position = agentPath->go[0]->position;
		nodeEnd.maxSizeX = scenarioSizeX;
		nodeEnd.maxSizeZ = scenarioSizeZ;
		nodeEnd.graphNodes = &graphNodes;
		nodeEnd.nodeSize = nodeSize;
		nodeEnd.graphNodesPos = &graphNodesPos;
		/*std::cout << nodeStart.x << " -- " << nodeStart.y << "\n";
		std::cout << nodeEnd.x << " -- " << nodeEnd.y << "\n";
		std::cout << agentPath->go[0]->name << "\n";
		std::cout << graphNodes[((nodeStart.x / nodeSize)*(scenarioSizeZ / nodeSize)) + (nodeStart.y / nodeSize)] << "\n";
		std::cout << graphNodes[((nodeEnd.x / nodeSize)*(scenarioSizeZ / nodeSize)) + (nodeEnd.y / nodeSize)] << "\n";
		if (InsideObstacle(nodeStart.x, 0, nodeStart.y)) {
			std::cout << "RAAAA\n";
		}*/
		// Set Start and goal states
		astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd);
		//std::system("PAUSE");
		unsigned int SearchState;
		unsigned int SearchSteps = 0;

		do
		{
			SearchState = astarsearch.SearchStep();

			SearchSteps++;
		} while (SearchState == AStarSearch<AStarSearchNode>::SEARCH_STATE_SEARCHING);

		if (SearchState == AStarSearch<AStarSearchNode>::SEARCH_STATE_SUCCEEDED)
		{
			//cout << "Search found goal state\n";

			AStarSearchNode *node = astarsearch.GetSolutionStart();

			int steps = 0;

			//node->PrintNodeInfo();
			for (;; )
			{
				node = astarsearch.GetSolutionNext();

				if (!node)
				{
					break;
				}

				agentPath->path.push_back(node->position);

				//node->PrintNodeInfo();
				steps++;
			};

			//cout << "Solution steps " << steps << endl;

			// Once you're done with the solution you can free the nodes up
			astarsearch.FreeSolutionNodes();
		}
		else if (SearchState == AStarSearch<AStarSearchNode>::SEARCH_STATE_FAILED)
		{
			cout << "Search terminated. Did not find goal state\n";
			cout << "SearchSteps : " << SearchSteps << "\n";
		}

		// Display the number of loops the search went through
		//cout << "SearchSteps : " << SearchSteps << "\n";

		SearchCount++;

		astarsearch.EnsureMemoryFreed();
	}
}