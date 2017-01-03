#include "stdafx.h"

AStarSearchNode::AStarSearchNode()
{
	x = y = 0;
}

AStarSearchNode::AStarSearchNode(int px, int py, float newMaxSizeX, float newMaxSizeZ, std::vector<int>* newGraphNodes) {
	x = px; 
	y = py; 
	maxSizeX = newMaxSizeX;
	maxSizeZ = newMaxSizeZ;

	graphNodes = newGraphNodes;
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
	std::cout << "Node position : (" << x << "," << y << ")\n";
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
	int parent_x = -1;
	int parent_y = -1;

	if (parent_node)
	{
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}

	AStarSearchNode NewNode;

	// push each possible move except allowing the search to go backwards
	if ((GetMap(x - 1, y) < 9)
		&& !((parent_x == x - 1) && (parent_y == y))
		)
	{
		NewNode = AStarSearchNode(x - 1, y, maxSizeX, maxSizeZ, graphNodes);
		astarsearch->AddSuccessor(NewNode);
	}

	if ((GetMap(x, y - 1) < 9)
		&& !((parent_x == x) && (parent_y == y - 1))
		)
	{
		NewNode = AStarSearchNode(x, y - 1, maxSizeX, maxSizeZ, graphNodes);
		astarsearch->AddSuccessor(NewNode);
	}

	if ((GetMap(x + 1, y) < 9)
		&& !((parent_x == x + 1) && (parent_y == y))
		)
	{
		NewNode = AStarSearchNode(x + 1, y, maxSizeX, maxSizeZ, graphNodes);
		astarsearch->AddSuccessor(NewNode);
	}


	if ((GetMap(x, y + 1) < 9)
		&& !((parent_x == x) && (parent_y == y + 1))
		)
	{
		NewNode = AStarSearchNode(x, y + 1, maxSizeX, maxSizeZ, graphNodes);
		astarsearch->AddSuccessor(NewNode);
	}

	return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving
float AStarSearchNode::GetCost(AStarSearchNode &successor)
{
	return (float)GetMap(x, y);
}

int AStarSearchNode::GetMap(int px, int py)
{
	if (px < 0 ||
		px >= maxSizeX ||
		py < 0 ||
		py >= maxSizeZ
		)
	{
		return 9;
	}
	//std::cout << px << " - " << py << ": " << (*graphNodes)[(px*maxSizeZ) + py] << "\n";
	//return (*graphNodes)[(py*maxSizeX) + px];
	return (*graphNodes)[(px*maxSizeZ) + py];
}