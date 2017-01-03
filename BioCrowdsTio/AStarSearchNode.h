#pragma once
class AStarSearchNode
{
	//public attributes
	public:
		int x;	 // the (x,y) positions of the node - IN OUR CASE: X, Z
		int y;
		float maxSizeX;
		float maxSizeZ;
		std::vector<int>* graphNodes;

	//public methods
	public:
		AStarSearchNode();
		~AStarSearchNode();
		AStarSearchNode(int px, int py, float newMaxSizeX, float newMaxSizeZ, std::vector<int>* newGraphNodes);
		float GoalDistanceEstimate(AStarSearchNode &nodeGoal);
		bool IsGoal(AStarSearchNode &nodeGoal);
		bool GetSuccessors(AStarSearch<AStarSearchNode> *astarsearch, AStarSearchNode *parent_node);
		float GetCost(AStarSearchNode &successor);
		bool IsSameState(AStarSearchNode &rhs);
		void PrintNodeInfo();
		int GetMap(int x, int y);
};

