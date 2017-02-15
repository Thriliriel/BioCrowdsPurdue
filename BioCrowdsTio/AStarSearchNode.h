#pragma once
class AStarSearchNode
{
	//public attributes
	public:
		//node position
		Vector3 position;
		//max world size
		float maxSizeX;
		float maxSizeZ;
		//graph information
		std::vector<int>* graphNodes;
		std::vector<Node>* graphNodesPos;
		//node size (default = 1)
		float nodeSize;

	//public methods
	public:
		AStarSearchNode();
		~AStarSearchNode();
		AStarSearchNode(Vector3 newPosition, float newMaxSizeX, float newMaxSizeZ, std::vector<int>* newGraphNodes, float newNodeSize, std::vector<Node>* newGraphNodesPos);
		float GoalDistanceEstimate(AStarSearchNode &nodeGoal);
		bool IsGoal(AStarSearchNode &nodeGoal);
		bool GetSuccessors(AStarSearch<AStarSearchNode> *astarsearch, AStarSearchNode *parent_node);
		float GetCost(AStarSearchNode &successor);
		bool IsSameState(AStarSearchNode &rhs);
		void PrintNodeInfo();
		int GetMap(float x, float y);
};

