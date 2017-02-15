#include "stdafx.h"

AStarSearchNode::AStarSearchNode()
{
	position = Vector3(0, 0, 0);
}

AStarSearchNode::AStarSearchNode(Vector3 newPosition, float newMaxSizeX, float newMaxSizeZ, std::vector<int>* newGraphNodes, float newNodeSize, std::vector<Node>* newGraphNodesPos) {
	position = newPosition;
	maxSizeX = newMaxSizeX;
	maxSizeZ = newMaxSizeZ;
	nodeSize = newNodeSize;

	graphNodes = newGraphNodes;
	graphNodesPos = newGraphNodesPos;
}

AStarSearchNode::~AStarSearchNode()
{
}

bool AStarSearchNode::IsSameState(AStarSearchNode &rhs)
{
	// same state in a maze search is simply when (x,y) are the same
	if ((position.x == rhs.position.x) &&
		(position.z == rhs.position.z))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void AStarSearchNode::PrintNodeInfo()
{
	//std::cout << "Node position : (" << x << "," << y << ")\n";
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal.
float AStarSearchNode::GoalDistanceEstimate(AStarSearchNode &nodeGoal)
{
	return fabsf(position.x - nodeGoal.position.x) + fabsf(position.z - nodeGoal.position.z);
}

bool AStarSearchNode::IsGoal(AStarSearchNode &nodeGoal)
{

	if ((position.x == nodeGoal.position.x) &&
		(position.z == nodeGoal.position.z))
	{
		return true;
	}

	return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool AStarSearchNode::GetSuccessors(AStarSearch<AStarSearchNode> *astarsearch, AStarSearchNode *parent_node)
{
	float parent_x = -1;
	float parent_y = -1;

	if (parent_node)
	{
		parent_x = parent_node->position.x;
		parent_y = parent_node->position.z;
	}

	AStarSearchNode NewNode;

	// push each possible move except allowing the search to go backwards
	//need to find the nodes which share 2 vertices with this one
	int index = -1;
	for (int i = 0; i < graphNodesPos->size(); i++) {
		if ((*graphNodesPos)[i].position.x == position.x && (*graphNodesPos)[i].position.z == position.z) {
			index = i;
			break;
		}
	}

	//found the actual node! Now, lets find the sucessors
	if (index > -1) {
		for (int i = 0; i < graphNodesPos->size(); i++) {
			//if they share 2 vertices, they are neighbours
			int qntShared = 0;
			if ( ((*graphNodesPos)[i].v1X == (*graphNodesPos)[index].v1X && (*graphNodesPos)[i].v1Z == (*graphNodesPos)[index].v1Z) ||
				 ((*graphNodesPos)[i].v1X == (*graphNodesPos)[index].v2X && (*graphNodesPos)[i].v1Z == (*graphNodesPos)[index].v2Z) ||
				 ((*graphNodesPos)[i].v1X == (*graphNodesPos)[index].v3X && (*graphNodesPos)[i].v1Z == (*graphNodesPos)[index].v3Z)) {
				qntShared++;
			}
			if (((*graphNodesPos)[i].v2X == (*graphNodesPos)[index].v1X && (*graphNodesPos)[i].v2Z == (*graphNodesPos)[index].v1Z) ||
				((*graphNodesPos)[i].v2X == (*graphNodesPos)[index].v2X && (*graphNodesPos)[i].v2Z == (*graphNodesPos)[index].v2Z) ||
				((*graphNodesPos)[i].v2X == (*graphNodesPos)[index].v3X && (*graphNodesPos)[i].v2Z == (*graphNodesPos)[index].v3Z)) {
				qntShared++;
			}
			if (((*graphNodesPos)[i].v3X == (*graphNodesPos)[index].v1X && (*graphNodesPos)[i].v3Z == (*graphNodesPos)[index].v1Z) ||
				((*graphNodesPos)[i].v3X == (*graphNodesPos)[index].v2X && (*graphNodesPos)[i].v3Z == (*graphNodesPos)[index].v2Z) ||
				((*graphNodesPos)[i].v3X == (*graphNodesPos)[index].v3X && (*graphNodesPos)[i].v3Z == (*graphNodesPos)[index].v3Z)) {
				qntShared++;
			}

			if (qntShared == 2) {
				if (!((parent_x == (*graphNodesPos)[i].position.x) && (parent_y == (*graphNodesPos)[i].position.z))) {
					NewNode = AStarSearchNode((*graphNodesPos)[i].position, maxSizeX, maxSizeZ, graphNodes, nodeSize, graphNodesPos);
					astarsearch->AddSuccessor(NewNode);
				}
			}
		}
	}//else, the agent is not at the exact node position. So, we set the nearest
	else {
		float distance = maxSizeX;
		int index2 = -1;
		for (int g = 0; g < (*graphNodesPos).size(); g++) {
			float thisDistance = Simulation::Distance(position, (*graphNodesPos)[g].position);
			if (thisDistance < distance) {
				index2 = g;
				distance = thisDistance;
			}
		}
		
		if (index2 > -1) {
			NewNode = AStarSearchNode((*graphNodesPos)[index2].position, maxSizeX, maxSizeZ, graphNodes, nodeSize, graphNodesPos);
			astarsearch->AddSuccessor(NewNode);
		}
	}

	//old way
	/*if ((GetMap(x - nodeSize, y) < 9)
		&& !((parent_x == x - nodeSize) && (parent_y == y))
		)
	{
		NewNode = AStarSearchNode(x - nodeSize, y, maxSizeX, maxSizeZ, graphNodes, nodeSize, graphNodesPos);
		astarsearch->AddSuccessor(NewNode);
	}

	if ((GetMap(x, y - nodeSize) < 9)
		&& !((parent_x == x) && (parent_y == y - nodeSize))
		)
	{
		NewNode = AStarSearchNode(x, y - nodeSize, maxSizeX, maxSizeZ, graphNodes, nodeSize, graphNodesPos);
		astarsearch->AddSuccessor(NewNode);
	}

	if ((GetMap(x + nodeSize, y) < 9)
		&& !((parent_x == x + nodeSize) && (parent_y == y))
		)
	{
		NewNode = AStarSearchNode(x + nodeSize, y, maxSizeX, maxSizeZ, graphNodes, nodeSize, graphNodesPos);
		astarsearch->AddSuccessor(NewNode);
	}


	if ((GetMap(x, y + nodeSize) < 9)
		&& !((parent_x == x) && (parent_y == y + nodeSize))
		)
	{
		NewNode = AStarSearchNode(x, y + nodeSize, maxSizeX, maxSizeZ, graphNodes, nodeSize, graphNodesPos);
		astarsearch->AddSuccessor(NewNode);
	}*/

	return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving
float AStarSearchNode::GetCost(AStarSearchNode &successor)
{
	return (float)GetMap(position.x, position.z);
}

int AStarSearchNode::GetMap(float px, float py)
{
	if (px < 0 ||
		px >= maxSizeX ||
		py < 0 ||
		py >= maxSizeZ
		)
	{
		return 9;
	}

	//find by position
	int index = -1;
	for (int i = 0; i < graphNodesPos->size(); i++) {
		if ((*graphNodesPos)[i].position.x == px && (*graphNodesPos)[i].position.z == py) {
			index = i;
			break;
		}
	}

	if (index > -1) {
		return (*graphNodes)[index];
	}
	else {
		return 9;
	}

	//std::cout << px << " - " << py << ": " << (*graphNodes)[(px*maxSizeZ) + py] << "\n";
	//return (*graphNodes)[(py*maxSizeX) + px];
	//return (*graphNodes)[((px / nodeSize)*(maxSizeZ / nodeSize)) + (py / nodeSize)];
}