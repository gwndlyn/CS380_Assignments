#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
	return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
	return false;
}

bool ProjectTwo::implemented_jps_plus()
{
	return false;
}
#pragma endregion

bool AStarPather::initialize()
{
	// handle any one-time setup requirements you have

	/*
		If you want to do any map-preprocessing, you'll need to listen
		for the map change message.  It'll look something like this:

		Callback cb = std::bind(&AStarPather::your_function_name, this);
		Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

		There are other alternatives to using std::bind, so feel free to mix it up.
		Callback is just a typedef for std::function<void(void)>, so any std::invoke'able
		object that std::function can wrap will suffice.
	*/

	//gridSize = Vec3((float)terrain->get_map_width(), (float)terrain->get_map_height(), 0.0f);

	//openList.clear();
	//closedList.clear();
	//std::fill(nodeArr.begin(), nodeArr.end(), Node());
	Messenger::listen_for_message(Messages::MAP_CHANGE, [this]()
								  {
									  gridSize = Vec3(static_cast<float>(terrain->get_map_width()), static_cast<float>(terrain->get_map_height()), 0.0f);

									  openList.clear();
									  closedList.clear();
									  std::fill(nodeArr.begin(), nodeArr.end(), Node());

									  req = nullptr;

								  });


	return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
	/*
		Free any dynamically allocated memory or any other general house-
		keeping you need to do during shutdown.
	*/
}

PathResult AStarPather::compute_path(PathRequest& request)
{
	/*
		This is where you handle pathing requests, each request has several fields:

		start/goal - start and goal world positions
		path - where you will build the path upon completion, path should be
			start to goal, not goal to start
		heuristic - which heuristic calculation to use
		weight - the heuristic weight to be applied
		newRequest - whether this is the first request for this path, should generally
			be true, unless single step is on

		smoothing - whether to apply smoothing to the path
		rubberBanding - whether to apply rubber banding
		singleStep - whether to perform only a single A* step
		debugColoring - whether to color the grid based on the A* state:
			closed list nodes - yellow
			open list nodes - blue

			use terrain->set_color(row, col, Colors::YourColor);
			also it can be helpful to temporarily use other colors for specific states
			when you are testing your algorithms

		method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
			will be A* generally, unless you implement extra credit features

		The return values are:
			PROCESSING - a path hasn't been found yet, should only be returned in
				single step mode until a path is found
			COMPLETE - a path to the goal was found and has been built in request.path
			IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
	*/

	if (req != nullptr || request.newRequest)
	{
		req = &request;
		req->newRequest = false;

		initialize();
	}

	//Run chosen algo
	switch (request.settings.method)
	{
	case Method::ASTAR:
	{
		runASTAR();
		break;
	}
	case Method::FLOYD_WARSHALL:
	{
		//runFLOYD_WARSHALL(req);
		break;
	}
	case Method::GOAL_BOUNDING:
	{
		//runGOAL_BOUNDING();
		break;
	}
	case Method::JPS_PLUS:
	{
		//runJPS_PLUS();
		break;
	}
	}

	for (const auto& c : closedList)
		req->path.push_back(c->pos.ConvertToVec3());

	//draw grid with color
	auto openColor = Colors::LightPink;
	auto closedColor = Colors::BlueViolet;

	for (const Node* o : openList)
		terrain->set_color(GridPos{ o->pos.y, o->pos.x }, openColor);

	for (const Node* c : closedList)
		terrain->set_color(GridPos{ c->pos.y, c->pos.x }, closedColor);

	return pathResult;

	// Just sample code, safe to delete
	//GridPos start = terrain->get_grid_position(request.start);
	//GridPos goal = terrain->get_grid_position(request.goal);
	//terrain->set_color(start, Colors::Orange);
	//terrain->set_color(goal, Colors::Orange);
	//request.path.push_back(request.start);
	//request.path.push_back(request.goal);
	//return PathResult::COMPLETE;
}

void AStarPather::runASTAR()
{
	//Push Start Node onto the Open List.
	//While(Open List is not empty)
	//{
	//	Pop cheapest node off Open List(parent node).
	//		If node is the Goal Node, then path found(RETURN “found”).
	//		For(all neighboring child nodes)
	//		{
	//			Compute its cost, f(x) = g(x) + h(x)
	//			If child node isn’t on Open or Closed list, put it on Open List.
	//			Else if child node is on Open or Closed List, AND this new one is cheaper,
	//			then take the old expensive one off both listsand put this new
	//			cheaper one on the Open List.
	//		}
	//		Place parent node on the Closed List(we’re done with it).
	//		If taken too much time this frame(or in single step mode),
	//		abort search for nowand resume next frame(RETURN “working”).
	//}
	//Open List empty, thus no path possible(RETURN “fail”).

	/*
	openList.emplace_back(Node{ req.start , 0.0f });

	while (!openList.empty())
	{
		Node parentNode = *openList.begin();
		std::find(openList.begin(), openList.end(),
				  [&](Node& a) { parentNode = (parentNode.fCost > a.fCost) ? parentNode : a; });
		openList.erase(std::find(openList.begin(), openList.end(), parentNode));

		if (parentNode.pos == req.start)
		{
			pathResult = PathResult::COMPLETE;
			return;
		}

		std::vector<Vec3> neighbours =
		{
			parentNode.pos.Up,
			parentNode.pos.Down,
			parentNode.pos.Left,
			parentNode.pos.Right
		};

		for (Vec3& n : neighbours)
		{
			//todo check if this is right
			float cost = parentNode.cost + (CalculateHeuristicCost(n) * req.settings.weight);

			if ((std::find(openList.begin(), openList.end(), n) != openList.end())
				|| (std::find(closedList.begin(), closedList.end(), n) != closedList.end()))
			{
				openList.emplace_back(Node{ n, cost });
			}
			else
			{
				//if (openList)
			}

			closedList.emplace_back(parentNode);

			//todo add timeout
			if (req.settings.singleStep)
			{
				pathResult = PathResult::PROCESSING;
				return;
			}
		}
	}
	if (openList.empty())
	{
		pathResult = PathResult::IMPOSSIBLE;
		return;
	}
	*/

	Node* startNode = &nodeArr[SingleIndexConverter(req->start)];
	openList.emplace_back(startNode);
	startNode->onList = OPEN;

	while (!openList.empty())
	{
		Node* pNode = PopCheapestOpenListNode();

		if (static_cast<float>(pNode->pos.x) == req->goal.x
			&& static_cast<float>(pNode->pos.y) == req->goal.y)
		{
			pathResult = PathResult::COMPLETE;
			return;
		}

		//todo make map bounds check here
		Vec3 v3PNode = pNode->pos.ConvertToVec3();
		std::vector<Node*> neighbours =
		{
			&nodeArr[SingleIndexConverter(v3PNode.Up)],
			&nodeArr[SingleIndexConverter(v3PNode.Down)],
			&nodeArr[SingleIndexConverter(v3PNode.Left)],
			&nodeArr[SingleIndexConverter(v3PNode.Right)]
		};

		for (Node* n : neighbours)
		{
			//check if it is a wall
			if (terrain->is_wall(n->pos.x, n->pos.y))
				continue;

			//calculate cost
			float g = pNode->gCost + 1;
			float h = CalculateHeuristicCost(n->pos, req->goal) * req->settings.weight;
			float f = g + h;

			if (n->onList == OPEN || n->onList == CLOSED)
			{
				openList.emplace_back(n);
				n->onList = OPEN;
				n->fCost = f;
				n->gCost = g;
				n->parentNodePos = pNode->pos;
			}
			else
			{
				UpdateCost(n, pNode, f, g);
			}

			closedList.emplace_back(pNode);
			pNode->onList = CLOSED;

			//TODO check if timeout is correct
			if (req->settings.singleStep || engine->get_timer().GetElapsedSeconds() >= 0.1f)
			{
				pathResult = PathResult::PROCESSING;
				return;
			}
		}

	}
	if (openList.empty())
	{
		pathResult = PathResult::IMPOSSIBLE;
		return;
	}

}

AStarPather::Node::Node(Vec2Int p, Vec2Int par, float f, float g, ONLIST ol)
	: pos{ p }, parentNodePos{ par }, fCost{ f }, gCost{ g }, onList{ ol } { }

float AStarPather::CalculateHeuristicCost(Vec2Int start, Vec2Int end)
{
	//Grid heuristic distance calculations
	Vec3 diff = Vec2(static_cast<float>(std::abs(start.x - end.x)), static_cast<float>(std::abs(start.y - end.y)));

	switch (req->settings.heuristic)
	{
	default:
	case Heuristic::OCTILE:
	{
		return std::min(diff.x, diff.y) * (float)sqrt(2) + std::max(diff.x, diff.y) - std::min(diff.x, diff.y);
	}
	case Heuristic::CHEBYSHEV:
	{
		return std::max(diff.x, diff.y);
	}
	case Heuristic::MANHATTAN:
	{
		return diff.x + diff.y;
	}
	case Heuristic::EUCLIDEAN:
	{
		return sqrt(diff.x * diff.x + diff.y * diff.y);
	}
	}
}

AStarPather::Node* AStarPather::PopCheapestOpenListNode()
{
	std::sort(openList.begin(), openList.end(), [](Node* a, Node* b) { return a->fCost > b->fCost; });

	auto retNode = openList.back();
	retNode->onList = NONE;

	auto temp = openList.end() - 1;
	openList.erase(temp);

	return retNode;
}

int AStarPather::SingleIndexConverter(const Vec2Int& pos)
{
	return pos.y * gridSize.x + pos.x;
}

void AStarPather::UpdateCost(Node* child, Node* parent, float newF, float newG)
{
	if (newF > child->fCost)
		return;

	child->fCost = newF;
	child->gCost = newG;
	child->parentNodePos = parent->pos;
}

Vec2Int::Vec2Int(int _x, int _y)
	: x{ _x }, y{ _y } { }

Vec2Int::Vec2Int(Vec3 v)
	: x{ static_cast<int>(v.x) }, y{ static_cast<int>(v.y) } { }

Vec3 Vec2Int::ConvertToVec3()
{
	return Vec3(static_cast<float>(x), static_cast<float>(y), 0.0f);
}

