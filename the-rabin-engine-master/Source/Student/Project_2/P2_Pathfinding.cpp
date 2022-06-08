#include <pch.h>
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

	Messenger::listen_for_message(Messages::MAP_CHANGE, [this]()
								  {
									  gridSize = GridPos{ terrain->get_map_height(), terrain->get_map_width() };

									  openList.clear();
									  closedList.clear();

									  for (int x = 0; x < gridSize.col; ++x)
										  for (int y = 0; y < gridSize.row; ++y)
											  nodeArr[SingleIndexConverter(GridPos{ y, x })].pos = GridPos{ y, x };

								  });

	sqrtTwo = static_cast<float>(sqrt(2));

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
		openList.clear();
		closedList.clear();

		for (int x = 0; x < gridSize.col; ++x)
			for (int y = 0; y < gridSize.row; ++y)
			{
				nodeArr[SingleIndexConverter(GridPos{ y, x })].fCost = 0;
				nodeArr[SingleIndexConverter(GridPos{ y, x })].gCost = 0;
				nodeArr[SingleIndexConverter(GridPos{ y, x })].onList = ONLIST::NONE;
				nodeArr[SingleIndexConverter(GridPos{ y, x })].parentNodePos = GridPos();
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

		for (const Node* o : openList)
			terrain->set_color(o->pos, openColor);

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

	if (request.newRequest)
	{
		Node* startNode = &nodeArr[SingleIndexConverter(terrain->get_grid_position(request.start))];
		openList.emplace_back(startNode);
		startNode->onList = ONLIST::OPEN;
	}

	while (!openList.empty())
	{
		Node* pNode = PopCheapestOpenListNode();

		if (pNode->pos == terrain->get_grid_position(request.goal))
		{
			closedList.emplace_back(pNode);
			pNode->onList = ONLIST::CLOSED;

			pathResult = PathResult::COMPLETE;
			return;
		}

		//get all 8 neighbours
		GridPos tNode = pNode->pos;
		std::unordered_map<DIRECTION, GridPos> neighbourPos;
		neighbourPos.insert({ DIRECTION::UP, GridPos{ tNode.row + 1, tNode.col } });
		neighbourPos.insert({ DIRECTION::DOWN, GridPos{ tNode.row - 1, tNode.col } });
		neighbourPos.insert({ DIRECTION::LEFT, GridPos{ tNode.row, tNode.col - 1 } });
		neighbourPos.insert({ DIRECTION::RIGHT, GridPos{ tNode.row, tNode.col + 1 } });
		neighbourPos.insert({ DIRECTION::UP_LEFT, GridPos{ tNode.row + 1, tNode.col - 1 } });
		neighbourPos.insert({ DIRECTION::UP_RIGHT, GridPos{ tNode.row + 1, tNode.col + 1 } });
		neighbourPos.insert({ DIRECTION::DOWN_LEFT, GridPos{ tNode.row - 1, tNode.col - 1 } });
		neighbourPos.insert({ DIRECTION::DOWN_RIGHT, GridPos{ tNode.row - 1, tNode.col + 1 } });

		std::unordered_map<DIRECTION, Node*> neighbours;
		for (const auto& np : neighbourPos)
		{
			GridPos pos = np.second;
			if (pos.col < 0 || pos.col >= gridSize.col
				|| pos.row < 0 || pos.row >= gridSize.row)
				continue;

			DIRECTION dir = np.first;
			switch (dir)
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

			neighbours.insert({ np.first, &nodeArr[SingleIndexConverter(np.second)] });
		}

		//find path
		for (const auto& [dir, n] : neighbours)
		{
			//check if it is a wall
			if (terrain->is_wall(n->pos.row, n->pos.col))
				continue;

			//calculate cost
			float g = static_cast<int>(dir) < 4 ? pNode->gCost + 1 : pNode->gCost + sqrtTwo;
			float h = CalculateHeuristicCost(n->pos, terrain->get_grid_position(request.goal), request.settings.heuristic);
			float f = g + h * request.settings.weight;

			if (n->onList == ONLIST::NONE)
			{
				openList.emplace_back(n);
				n->onList = ONLIST::OPEN;
				n->fCost = f;
				n->gCost = g;
				n->parentNodePos = pNode->pos;
			}
			else
			{
				UpdateCost(n, pNode, f, g);
			}
		}

		closedList.emplace_back(pNode);
		pNode->onList = ONLIST::CLOSED;

		if (request.settings.singleStep) // || engine->get_timer().GetElapsedSeconds() >= 1.0f)
		{
			pathResult = PathResult::PROCESSING;
			return;
		}

	}
	if (openList.empty())
	{
		pathResult = PathResult::IMPOSSIBLE;
		return;
	}

}

AStarPather::Node::Node(GridPos p, GridPos par, float f, float g, ONLIST ol)
	: pos{ p }, parentNodePos{ par }, fCost{ f }, gCost{ g }, onList{ ol } { }

float AStarPather::CalculateHeuristicCost(const GridPos& start, const GridPos& end, const Heuristic& h)
{
	//Grid heuristic distance calculations
	float diffX = static_cast<float>(std::abs(start.col - end.col));
	float diffY = static_cast<float>(std::abs(start.row - end.row));

	switch (h)
	{
	case Heuristic::OCTILE:
	{
		return std::min(diffX, diffY) * sqrtTwo + std::max(diffX, diffY) - std::min(diffX, diffY);
	}
	case Heuristic::CHEBYSHEV:
	{
		return std::max(diffX, diffY);
	}
	case Heuristic::MANHATTAN:
	{
		return diffX + diffY;
	}
	case Heuristic::EUCLIDEAN:
	{
		return sqrt(diffX * diffX + diffY * diffY);
	}
	default:
		return 0;
	}
}

AStarPather::Node* AStarPather::PopCheapestOpenListNode()
{
	std::sort(openList.begin(), openList.end(), [](Node* a, Node* b) { return a->fCost > b->fCost; });

	auto retNode = openList.back();

	auto temp = openList.end() - 1;
	openList.erase(temp);

	return retNode;
}

int AStarPather::SingleIndexConverter(const GridPos& pos)
{
	return pos.row * gridSize.col + pos.col;
}

void AStarPather::UpdateCost(Node* child, Node* parent, float newF, float newG)
{
	if (newF >= child->fCost)
		return;

	auto exNode = std::find(closedList.begin(), closedList.end(), child);

	if (exNode != closedList.end())
	{
		openList.emplace_back(*exNode);
		(*exNode)->onList = ONLIST::OPEN;

		closedList.erase(exNode);
	}
	else
	{
		openList.emplace_back(child);
		child->onList = ONLIST::OPEN;
	}

	child->fCost = newF;
	child->gCost = newG;
	child->parentNodePos = parent->pos;

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
		GridPos minPos = GridPos{ std::min(gridPath[str].row, gridPath[end].row), std::min(gridPath[str].col, gridPath[end].col) };
		GridPos maxPos = GridPos{ std::max(gridPath[str].row, gridPath[end].row), std::max(gridPath[str].col, gridPath[end].col) };

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
	for (std::list<Vec3>::iterator it = rPath.begin(); it != rPath.end();)
	{
		std::list<Vec3>::iterator start, end;
		start = it;
		end = ++it;

		if (end == rPath.end())
			return;

		if (Vec3::Distance(*start, *end) > 1.5f)
		{
			Vec3 halfSum = (*start + *end) / 2;
			rPath.insert(it, halfSum);
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


