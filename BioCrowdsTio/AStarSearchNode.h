#pragma once
class AStarSearchNode
{
	//public attributes
	public:
		float x;	 // the (x,y) positions of the node - IN OUR CASE: X, Z
		float y;
		float maxSizeX;
		float maxSizeZ;
		std::vector<int>* graphNodes;
		std::vector<Node>* graphNodesPos;
		float nodeSize;

	//public methods
	public:
		AStarSearchNode();
		~AStarSearchNode();
		AStarSearchNode(float px, float py, float newMaxSizeX, float newMaxSizeZ, std::vector<int>* newGraphNodes, float newNodeSize, std::vector<Node>* newGraphNodesPos);
		float GoalDistanceEstimate(AStarSearchNode &nodeGoal);
		bool IsGoal(AStarSearchNode &nodeGoal);
		bool GetSuccessors(AStarSearch<AStarSearchNode> *astarsearch, AStarSearchNode *parent_node);
		float GetCost(AStarSearchNode &successor);
		bool IsSameState(AStarSearchNode &rhs);
		void PrintNodeInfo();
		int GetMap(float x, float y);

	//private methods
	private:
		float Distance(float x1, float y1, float z1, float x2, float y2, float z2);
};

