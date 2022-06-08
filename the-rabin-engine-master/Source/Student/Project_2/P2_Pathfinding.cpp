#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

constexpr int diag = 14, linear = 10;

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

	Messenger::listen_for_message(Messages::MAP_CHANGE, [this] ()
		{
			gridSize = GridPos { terrain->get_map_height(), terrain->get_map_width() };

			openList = {};
			closedList.clear();

			nodeArr = std::vector<Node>(gridSize.col * gridSize.row);

			for (int x = 0; x < gridSize.col; ++x)
				for (int y = 0; y < gridSize.row; ++y)
					nodeArr[SingleIndexConverter(GridPos { y, x })].pos = GridPos { y, x };
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

	if (request.newRequest)
	{
		openList = {};
		closedList.clear();

		for (int x = 0; x < gridSize.col; ++x)
			for (int y = 0; y < gridSize.row; ++y)
			{
				nodeArr[SingleIndexConverter(GridPos { y, x })].Reset();
			}
	}

	//Run chosen algo
	switch (request.settings.method)
	{
		case Method::ASTAR:
		{
			runASTAR(request);
			break;
		}
		case Method::FLOYD_WARSHALL:
		case Method::GOAL_BOUNDING:
		case Method::JPS_PLUS:
			return PathResult::IMPOSSIBLE;
			break;
	}

	//set up path
	if (pathResult == PathResult::COMPLETE)
	{
		GridPos pPos = terrain->get_grid_position(request.goal);
		Node* node = nullptr;

		while (pPos != terrain->get_grid_position(request.start))
		{
			Node* node = &nodeArr[SingleIndexConverter(pPos)];
			request.path.emplace_front(terrain->get_world_position(node->pos));
			pPos = node->parentNodePos;
			terrain->set_color(pPos, Colors::LightPink);
		}

		node = &nodeArr[SingleIndexConverter(pPos)];
		request.path.emplace_front(terrain->get_world_position(node->pos));
		terrain->set_color(pPos, Colors::LightPink);

		//handle rubberbanding and smoothing
		if (request.settings.rubberBanding)
			rubberbanding(request);

		if (request.settings.smoothing)
		{
			addPointsBackIn(request);
			smoothing(request);
		}
	}

	//draw grid with color
	if (request.settings.debugColoring)
	{
		auto openColor = Colors::BlueViolet;
		auto closedColor = Colors::LightPink;

		for (int i = 0; i < openList.size(); ++i)
		{
			const Node* const* ptr = (&openList.top()) + i;
			terrain->set_color((*ptr)->pos, openColor);
		}

		for (const Node* c : closedList)
			terrain->set_color(c->pos, closedColor);
	}

	return pathResult;

}

void AStarPather::runASTAR(PathRequest& request)
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

	const GridPos startPos = terrain->get_grid_position(request.start);
	const GridPos endPos = terrain->get_grid_position(request.goal);

	if (request.newRequest)
	{
		Node* startNode = &nodeArr[SingleIndexConverter(startPos)];
		openList.emplace(startNode);
		startNode->onList = OPEN;
	}

	while (!openList.empty())
	{
		Node* pNode = PopCheapestOpenListNode();
		const GridPos& tNode = pNode->pos;

		if (tNode == endPos)
		{
			closedList.emplace(pNode);
			pathResult = PathResult::COMPLETE;
			return;
		}

		//get all 8 neighbours
		std::vector<GridPos> neighbourPos(DIRECTION::MAX);

		neighbourPos[DIRECTION::UP] = GridPos { tNode.row + 1, tNode.col };
		neighbourPos[DIRECTION::DOWN] = GridPos { tNode.row - 1, tNode.col };
		neighbourPos[DIRECTION::LEFT] = GridPos { tNode.row, tNode.col - 1 };
		neighbourPos[DIRECTION::RIGHT] = GridPos { tNode.row, tNode.col + 1 };
		neighbourPos[DIRECTION::UP_LEFT] = GridPos { tNode.row + 1, tNode.col - 1 };
		neighbourPos[DIRECTION::UP_RIGHT] = GridPos { tNode.row + 1, tNode.col + 1 };
		neighbourPos[DIRECTION::DOWN_LEFT] = GridPos { tNode.row - 1, tNode.col - 1 };
		neighbourPos[DIRECTION::DOWN_RIGHT] = GridPos { tNode.row - 1, tNode.col + 1 };

		for (int i = 0; i < DIRECTION::MAX; ++i)
		{
			const GridPos& pos = neighbourPos[i];

			if (pos.col < 0 || pos.col >= gridSize.col || pos.row < 0 || pos.row >= gridSize.row)
			{
				continue;
			}

			if (terrain->is_wall(pos))
			{
				continue;
			}

			auto adjNode = &nodeArr[SingleIndexConverter(pos)];

			if (closedList.find(adjNode) != closedList.end())
			{
				continue;
			}

			switch (i)
			{
				case DIRECTION::UP_LEFT:
				{
					if (terrain->is_wall(neighbourPos[DIRECTION::UP]) || terrain->is_wall(neighbourPos[DIRECTION::LEFT]))
						continue;
					break;
				}
				case DIRECTION::UP_RIGHT:
				{
					if (terrain->is_wall(neighbourPos[DIRECTION::UP]) || terrain->is_wall(neighbourPos[DIRECTION::RIGHT]))
						continue;
					break;
				}
				case DIRECTION::DOWN_LEFT:
				{
					if (terrain->is_wall(neighbourPos[DIRECTION::DOWN]) || terrain->is_wall(neighbourPos[DIRECTION::LEFT]))
						continue;
					break;
				}
				case DIRECTION::DOWN_RIGHT:
				{
					if (terrain->is_wall(neighbourPos[DIRECTION::DOWN]) || terrain->is_wall(neighbourPos[DIRECTION::RIGHT]))
						continue;
					break;
				}
				default:
					break;
			}

			//calculate cost
			int g = (i < 4) ? pNode->gCost + linear : pNode->gCost + diag;
			float h = CalculateHeuristicCost(adjNode->pos, endPos, request.settings.heuristic);
			float f = (float)g + h * request.settings.weight;

			UpdateCost(adjNode, pNode, f, g);
		}

		pNode->onList = CLOSED;
		closedList.emplace(pNode);

		if (request.settings.singleStep) // || engine->get_timer().GetElapsedSeconds() >= 1.0f)
		{
			pathResult = PathResult::PROCESSING;
			return;
		}
	}

	pathResult = PathResult::IMPOSSIBLE;
	return;
}

AStarPather::Node::Node(GridPos p, GridPos par, float f, int g)
	: pos { p }, parentNodePos { par }, fCost { f }, gCost { g } { }

void AStarPather::Node::Reset()
{
	*this = { pos };
}

float AStarPather::CalculateHeuristicCost(const GridPos& start, const GridPos& end, const Heuristic& h)
{
	//Grid heuristic distance calculations
	int diffX = std::abs(end.col - start.col);
	int diffY = std::abs(end.row - start.row);

	switch (h)
	{
		case Heuristic::OCTILE:
		{
			return static_cast<float>(std::min(diffX, diffY) * diag + (std::max(diffX, diffY) - std::min(diffX, diffY)) * linear);
		}
		case Heuristic::CHEBYSHEV:
		{
			return static_cast<float>(std::max(diffX, diffY));
		}
		case Heuristic::MANHATTAN:
		{
			return static_cast<float>(diffX + diffY);
		}
		case Heuristic::EUCLIDEAN:
		{
			return static_cast<float>(sqrt(diffX * diffX * 100 + diffY * diffY * 100));
		}
		default:
			return 0;
	}
}

AStarPather::Node* AStarPather::PopCheapestOpenListNode()
{
	auto retNode = openList.top();
	retNode->onList = NONE;
	openList.pop();

	return retNode;
}

int AStarPather::SingleIndexConverter(const GridPos& pos)
{
	return pos.row * gridSize.col + pos.col;
}

void AStarPather::UpdateCost(Node* child, Node* parent, float newF, int newG)
{
	if (newF >= child->fCost)
		return;

	auto exNode = closedList.find(child);
	if (exNode != closedList.end())
	{
		closedList.erase(exNode);
		child->onList = (child->onList == CLOSED) ? NONE : child->onList;
	}

	child->fCost = newF;
	child->gCost = newG;
	child->parentNodePos = parent->pos;

	if (child->onList == OPEN)
	{
		std::make_heap(const_cast<Node**>(&openList.top()),
			const_cast<Node**>(&openList.top()) + openList.size(),
			NodeCmp());
	}
	else
	{
		openList.emplace(child);
		child->onList = OPEN;
	}
}

void AStarPather::rubberbanding(PathRequest& request)
{
	if (request.path.size() < 3)
		return;

	std::vector<GridPos> gridPath;
	for (const auto& p : request.path)
		gridPath.emplace_back(terrain->get_grid_position(p));
	std::reverse(gridPath.begin(), gridPath.end());

	size_t str = gridPath.size() - 3;
	size_t mid = gridPath.size() - 2;
	size_t end = gridPath.size() - 1;

	while (str != mid)
	{
		GridPos minPos = GridPos { std::min(gridPath[str].row, gridPath[end].row), std::min(gridPath[str].col, gridPath[end].col) };
		GridPos maxPos = GridPos { std::max(gridPath[str].row, gridPath[end].row), std::max(gridPath[str].col, gridPath[end].col) };

		bool hasWall = false;

		for (int x = 0; x <= maxPos.col - minPos.col; ++x)
		{
			for (int y = 0; y <= maxPos.row - minPos.row; ++y)
			{
				if (terrain->is_wall(minPos.row + y, minPos.col + x))
				{
					hasWall = true;
				}
			}
		}

		//do not remove anything, move all 3 pointers
		if (hasWall)
		{
			--mid;
			--end;
		}
		//remove midpoint, move str and mid, do not move end
		else
		{
			auto exMid = std::find(gridPath.begin(), gridPath.end(), gridPath[mid]);
			gridPath.erase(exMid);

			mid = str;
			--end;
		}

		if (str != 0)
			--str;
	}

	request.path.clear();
	for (const auto& g : gridPath)
		request.path.emplace_front(terrain->get_world_position(g));

}

void AStarPather::addPointsBackIn(PathRequest& request)
{
	auto& rPath = request.path;
	const float distance = (terrain->get_world_position({ gridSize.row - 1 ,0 }) - terrain->get_world_position({ 0 ,0 })).x * 0.075f;

	for (std::list<Vec3>::iterator it = rPath.begin(); it != rPath.end();)
	{
		std::list<Vec3>::iterator start, end;
		start = it;
		end = ++it;

		if (end == rPath.end())
			return;

		if (Vec3::Distance(*start, *end) > distance)
		{
			Vec3 halfSum = (*start + *end) / 2;
			rPath.insert(end, halfSum);
			it = start;
		}
	}
}

void AStarPather::smoothing(PathRequest& request)
{
	auto& pPath = request.path;

	if (pPath.size() < 2)
		return;

	WaypointList::iterator start = pPath.begin();
	WaypointList::iterator end = pPath.end(); --end;
	WaypointList::iterator p1, p2, p3, p4;
	p1 = p2 = p3 = p4 = start;

	if (pPath.size() != 2)
	{
		++p3;
		++p4;
		++p4;
	}
	else
	{
		++p3;
		++p4;
	}

	while (true)
	{
		Vec3 newPt;
		for (int i = 1; i < 4; ++i)
		{
			newPt = Vec3::CatmullRom(*p1, *p2, *p3, *p4, i * 0.25f);
			pPath.insert(p3, newPt);
		}

		if (p3 == p4)
			break;

		if (p1 != p2)
			p1 = p2;
		p2 = p3;
		p3 = p4;

		if (p4 != end)
			++p4;
	}
}


