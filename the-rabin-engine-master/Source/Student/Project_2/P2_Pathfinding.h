#pragma once
#include "Misc/PathfindingDetails.hpp"

struct Vec2Int
{
	int x, y;

	Vec2Int(int _x = 0, int _y = 0);
	Vec2Int(Vec3 v);
	Vec3 ConvertToVec3();
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

	enum class ONLIST
	{
		NONE,
		OPEN,
		CLOSED
	};

	struct Node
	{
		Vec2Int pos;
		Vec2Int parentNodePos;
		float fCost, gCost;
		ONLIST onList;

		Node(Vec2Int p = Vec2Int(), Vec2Int par = Vec2Int(), float f = FLT_MAX, float g = 0.0f, ONLIST ol = ONLIST::NONE);
	};

	//variables
	std::array<Node, 1600> nodeArr;
	std::vector<Node*> openList;
	std::vector<Node*> closedList;

	//reference varables 
	PathResult pathResult;
	Vec2Int gridSize;

	//helper functions for nodes
	float CalculateHeuristicCost(const Vec2Int& start, const Vec2Int& end, const Heuristic& h);
	void UpdateCost(Node* child, Node* parent, float newF, float newG);
	Node* PopCheapestOpenListNode();
	int SingleIndexConverter(const Vec2Int& pos);

	//algo functions
	void runASTAR(PathRequest& request);

};