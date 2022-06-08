#pragma once

#include <unordered_set>

#include "Misc/PathfindingDetails.hpp"

enum DIRECTION
{
	UP,
	DOWN,
	LEFT,
	RIGHT,
	UP_LEFT,
	UP_RIGHT,
	DOWN_LEFT,
	DOWN_RIGHT,

	MAX
};

class AStarPather
{
public:
	/*
		The class should be default constructible, so you may need to define a constructor.
		If needed, you can modify the framework where the class is constructed in the
		initialize functions of ProjectTwo and ProjectThree.
	*/

	/* ************************************************** */
	// DO NOT MODIFY THESE SIGNATURES
	bool initialize();
	void shutdown();
	PathResult compute_path(PathRequest& request);
	/* ************************************************** */

	/*
		You should create whatever functions, variables, or classes you need.
		It doesn't all need to be in this header and cpp, structure it whatever way
		makes sense to you.
	*/

	enum ONLIST
	{
		NONE,
		OPEN,
		CLOSED
	};


	struct Node
	{
		GridPos pos;
		GridPos parentNodePos;
		float fCost;
		int gCost;

		ONLIST onList { NONE };

		Node(GridPos p = GridPos(), GridPos par = GridPos(), float f = FLT_MAX, int g = 0.0f);

		void Reset();
	};

	struct NodeCmp
	{
		bool operator()(const Node* lhs, const Node* rhs) const
		{
			return lhs->fCost > rhs->fCost;
		}
	};

	//variables
	std::vector<Node> nodeArr;
	std::priority_queue<Node*, std::vector<Node*>, NodeCmp> openList;
	std::unordered_set<Node*> closedList;

	//reference varables 
	PathResult pathResult;
	GridPos gridSize;

	//helper functions for nodes
	float CalculateHeuristicCost(const GridPos& start, const GridPos& end, const Heuristic& h);
	void UpdateCost(Node* child, Node* parent, float newF, int newG);
	Node* PopCheapestOpenListNode();
	int SingleIndexConverter(const GridPos& pos);

	//algo functions
	void runASTAR(PathRequest& request);

	void rubberbanding(PathRequest& request);
	void addPointsBackIn(PathRequest& request);
	void smoothing(PathRequest& request);

};