#include "stdafx.h"

AStarSearchNode::AStarSearchNode()
{
	x = y = 0;
}

AStarSearchNode::AStarSearchNode(float px, float py, float newMaxSizeX, float newMaxSizeZ, std::vector<int>* newGraphNodes, float newNodeSize, std::vector<Node>* newGraphNodesPos) {
	x = px; 
	y = py; 
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
	if ((x == rhs.x) &&
		(y == rhs.y))
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
	return fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y);
}

bool AStarSearchNode::IsGoal(AStarSearchNode &nodeGoal)
{

	if ((x == nodeGoal.x) &&
		(y == nodeGoal.y))
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
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}

	AStarSearchNode NewNode;

	// push each possible move except allowing the search to go backwards
	//need to find the nodes which share 2 vertices with this one
	int index = -1;
	for (int i = 0; i < graphNodesPos->size(); i++) {
		if ((*graphNodesPos)[i].x == x && (*graphNodesPos)[i].z == y) {
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
				if (!((parent_x == (*graphNodesPos)[i].x) && (parent_y == (*graphNodesPos)[i].z))) {
					NewNode = AStarSearchNode((*graphNodesPos)[i].x, (*graphNodesPos)[i].z, maxSizeX, maxSizeZ, graphNodes, nodeSize, graphNodesPos);
					astarsearch->AddSuccessor(NewNode);
				}
			}
		}
	}//else, the agent is not at the exact node position. So, we set the nearest
	else {
		float distance = maxSizeX;
		int index2 = -1;
		for (int g = 0; g < (*graphNodesPos).size(); g++) {
			float thisDistance = Distance(x, 0, y, (*graphNodesPos)[g].x, 0, (*graphNodesPos)[g].z);
			if (thisDistance < distance) {
				index2 = g;
				distance = thisDistance;
			}
		}
		
		if (index2 > -1) {
			NewNode = AStarSearchNode((*graphNodesPos)[index2].x, (*graphNodesPos)[index2].z, maxSizeX, maxSizeZ, graphNodes, nodeSize, graphNodesPos);
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
	return (float)GetMap(x, y);
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
		if ((*graphNodesPos)[i].x == px && (*graphNodesPos)[i].z == py) {
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

//distance between 2 points
float AStarSearchNode::Distance(float x1, float y1, float z1, float x2, float y2, float z2)
{
	float result = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (z2 - z1)*(z2 - z1));

	return result;
}