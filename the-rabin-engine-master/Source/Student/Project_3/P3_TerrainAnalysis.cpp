#include <pch.h>
#include "Terrain/TerrainAnalysis.h"
#include "Terrain/MapMath.h"
#include "Agent/AStarAgent.h"
#include "Terrain/MapLayer.h"
#include "Projects/ProjectThree.h"

#include <iostream>

bool ProjectThree::implemented_fog_of_war() const // extra credit
{
	return false;
}

float distance_to_closest_wall(int row, int col)
{
	/*
		Check the euclidean distance from the given cell to every other wall cell,
		with cells outside the map bounds treated as walls, and return the smallest
		distance.  Make use of the is_valid_grid_position and is_wall member
		functions in the global terrain to determine if a cell is within map bounds
		and a wall, respectively.
	*/

	float shortestDist = INFINITY;
	for (int r = -1; r < terrain->get_map_height() + 1; ++r)
	{
		for (int c = -1; c < terrain->get_map_width() + 1; ++c)
		{
			float distToCurrCell = 0.0f;
			if (r == -1 || c == -1
				|| r == terrain->get_map_height() || c == terrain->get_map_width()
				|| (terrain->is_valid_grid_position(r, c) && terrain->is_wall(r, c)))
			{
				distToCurrCell = static_cast<float>(sqrt((c - col) * (c - col) + (r - row) * (r - row)));
				if (distToCurrCell < shortestDist)
					shortestDist = distToCurrCell;
			}
		}
	}
	return shortestDist;
}

bool is_clear_path(int row0, int col0, int row1, int col1)
{
	/*
		Two cells (row0, col0) and (row1, col1) are visible to each other if a line
		between their centerpoints doesn't intersect the four boundary lines of every
		wall cell.  You should puff out the four boundary lines by a very tiny amount
		so that a diagonal line passing by the corner will intersect it.  Make use of the
		line_intersect helper function for the intersection test and the is_wall member
		function in the global terrain to determine if a cell is a wall or not.
	*/

	//GridPos lowerBound = GridPos{ std::min(row0, row1), std::min(col0,col1) };
	//GridPos upperBound = GridPos{ std::max(row0, row1), std::max(col0,col1) };
	Vec2 p01 = { terrain->get_world_position(GridPos{row0, col0}).x, terrain->get_world_position(GridPos{row0, col0}).z };
	Vec2 p02 = { terrain->get_world_position(GridPos{row1, col1}).x, terrain->get_world_position(GridPos{row1, col1}).z };
	float halfCellOffset = terrain->mapSizeInWorld / terrain->get_map_width() / 2 + 0.05f;

	//go through bounding box
	for (int r = 0; r < terrain->get_map_height(); ++r)
	{
		for (int c = 0; c < terrain->get_map_width(); ++c)
		{
			GridPos currCell = GridPos{ r, c };
			if (!terrain->is_wall(currCell) || !terrain->is_valid_grid_position(currCell))
				continue;

			Vec2 cellMid = { terrain->get_world_position(currCell).x, terrain->get_world_position(currCell).z };
			std::array<Vec2, 4> cellPoints
			{
				Vec2{cellMid.x + halfCellOffset, cellMid.y + halfCellOffset},
				Vec2{cellMid.x + halfCellOffset, cellMid.y - halfCellOffset},
				Vec2{cellMid.x - halfCellOffset, cellMid.y - halfCellOffset},
				Vec2{cellMid.x - halfCellOffset, cellMid.y + halfCellOffset}
			};

			//go through each cell side
			if (line_intersect(p01, p02, cellPoints[0], cellPoints[1])
				|| line_intersect(p01, p02, cellPoints[1], cellPoints[2])
				|| line_intersect(p01, p02, cellPoints[2], cellPoints[3])
				|| line_intersect(p01, p02, cellPoints[3], cellPoints[0]))
				return false;
		}
	}

	return true;
}

void analyze_openness(MapLayer<float>& layer)
{
	/*
		Mark every cell in the given layer with the value 1 / (d * d),
		where d is the distance to the closest wall or edge.  Make use of the
		distance_to_closest_wall helper function.  Walls should not be marked.
	*/

	for (int r = 0; r < terrain->get_map_height(); ++r)
	{
		for (int c = 0; c < terrain->get_map_width(); ++c)
		{
			float dist = distance_to_closest_wall(r, c);
			layer.set_value(GridPos{ r, c }, 1 / (dist * dist));
		}
	}

}

void analyze_visibility(MapLayer<float>& layer)
{
	/*
		Mark every cell in the given layer with the number of cells that
		are visible to it, divided by 160 (a magic number that looks good).  Make sure
		to cap the value at 1.0 as well.

		Two cells are visible to each other if a line between their centerpoints doesn't
		intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
		helper function.
	*/

	for (int r = 0; r < terrain->get_map_height(); ++r)
	{
		for (int c = 0; c < terrain->get_map_width(); ++c)
		{
			float visibleTiles = 0.0f;
			for (int y = 0; y < terrain->get_map_height(); ++y)
			{
				for (int x = 0; x < terrain->get_map_width(); ++x)
				{
					if (is_clear_path(r, c, y, x))
						++visibleTiles;
				}
			}

			visibleTiles /= 160.0f;
			if (visibleTiles > 1.0f)
				visibleTiles = 1.0f;

			layer.set_value(r, c, visibleTiles);

		}
	}

}

void analyze_visible_to_cell(MapLayer<float>& layer, int row, int col)
{
	/*
		For every cell in the given layer mark it with 1.0
		if it is visible to the given cell, 0.5 if it isn't visible but is next to a visible cell,
		or 0.0 otherwise.

		Two cells are visible to each other if a line between their centerpoints doesn't
		intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
		helper function.
	*/

	for (int r = 0; r < terrain->get_map_height(); ++r)
	{
		for (int c = 0; c < terrain->get_map_width(); ++c)
		{
			if (is_clear_path(row, col, r, c))
				layer.set_value(r, c, 1.0f);
			else
				layer.set_value(r, c, 0.0f);
		}
	}

	for (int r = 0; r < terrain->get_map_height(); ++r)
	{
		for (int c = 0; c < terrain->get_map_width(); ++c)
		{
			if (layer.get_value(r, c) == 1.0f)
			{
				std::array<GridPos, 4> dirs
				{
					GridPos{r + 1, c},
					GridPos{r, c + 1},
					GridPos{r - 1, c},
					GridPos{r, c - 1}
				};

				for (int i = 0; i < dirs.size(); ++i)
				{
					if (!terrain->is_valid_grid_position(dirs[i]) || terrain->is_wall(dirs[i]))
						continue;

					if (layer.get_value(dirs[i]) != 1.0f)
						layer.set_value(dirs[i], 0.5f);
				}

				if (terrain->is_valid_grid_position(r + 1, c + 1) && !terrain->is_wall(dirs[0]) && !terrain->is_wall(dirs[1]))
					if (layer.get_value(r + 1, c + 1) != 1.0f && !terrain->is_wall(r + 1, c + 1))
						layer.set_value(r + 1, c + 1, 0.5f);
				if (terrain->is_valid_grid_position(r - 1, c + 1) && !terrain->is_wall(dirs[1]) && !terrain->is_wall(dirs[2]))
					if (layer.get_value(r - 1, c + 1) != 1.0f && !terrain->is_wall(r - 1, c + 1))
						layer.set_value(r - 1, c + 1, 0.5f);
				if (terrain->is_valid_grid_position(r - 1, c - 1) && !terrain->is_wall(dirs[2]) && !terrain->is_wall(dirs[3]))
					if (layer.get_value(r - 1, c - 1) != 1.0f && !terrain->is_wall(r - 1, c - 1))
						layer.set_value(r - 1, c - 1, 0.5f);
				if (terrain->is_valid_grid_position(r + 1, c - 1) && !terrain->is_wall(dirs[3]) && !terrain->is_wall(dirs[0]))
					if (layer.get_value(r + 1, c - 1) != 1.0f && !terrain->is_wall(r + 1, c - 1))
						layer.set_value(r + 1, c - 1, 0.5f);

			}
		}
	}
}

void analyze_agent_vision(MapLayer<float>& layer, const Agent* agent)
{
	/*
		For every cell in the given layer that is visible to the given agent,
		mark it as 1.0, otherwise don't change the cell's current value.

		You must consider the direction the agent is facing.  All of the agent data is
		in three dimensions, but to simplify you should operate in two dimensions, the XZ plane.

		Take the dot product between the view vector and the vector from the agent to the cell,
		both normalized, and compare the cosines directly instead of taking the arccosine to
		avoid introducing floating-point inaccuracy (larger cosine means smaller angle).

		Give the agent a field of view slighter larger than 180 degrees.

		Two cells are visible to each other if a line between their centerpoints doesn't
		intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
		helper function.
	*/

	Vec2 agentView{ agent->get_forward_vector().x,agent->get_forward_vector().z };
	agentView.Normalize();

	for (int r = 0; r < terrain->get_map_height(); ++r)
	{
		for (int c = 0; c < terrain->get_map_width(); ++c)
		{
			Vec2 currCell{ terrain->get_world_position(r,c).x, terrain->get_world_position(r,c).z };
			Vec2 cellToAgent{ currCell.x - agent->get_position().x, currCell.y - agent->get_position().z };
			cellToAgent.Normalize();

			GridPos agentPosWP = terrain->get_grid_position(agent->get_position());

			float dotProd = agentView.Dot(cellToAgent);
			float fov = (float)cos(PI * 1.01);

			if (dotProd > fov && dotProd > 0)
				if (is_clear_path(agentPosWP.row, agentPosWP.col, r, c))
					layer.set_value(r, c, 1);
		}
	}
}

void propagate_solo_occupancy(MapLayer<float>& layer, float decay, float growth)
{
	/*
		For every cell in the given layer:

			1) Get the value of each neighbor and apply decay factor
			2) Keep the highest value from step 1
			3) Linearly interpolate from the cell's current value to the value from step 2
			   with the growing factor as a coefficient.  Make use of the lerp helper function.
			4) Store the value from step 3 in a temporary layer.
			   A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

		After every cell has been processed into the temporary layer, write the temporary layer into
		the given layer;
	*/

	float maxNeighbourInfluence;
	std::array<	std::array<float, 40>, 40> tempLayer;

	for (int r = 0; r < terrain->get_map_height(); ++r)
	{
		for (int c = 0; c < terrain->get_map_width(); ++c)
		{
			maxNeighbourInfluence = 0.0f;

			std::array<GridPos, 8> neighbours
			{
				GridPos{r + 1, c},
				GridPos{r, c + 1},
				GridPos{r - 1, c},
				GridPos{r, c - 1},

				GridPos{r + 1, c + 1},
				GridPos{r - 1, c + 1},
				GridPos{r - 1, c - 1},
				GridPos{r + 1, c - 1}
			};

			for (int i = 0; i < neighbours.size(); ++i)
			{
				if (!terrain->is_valid_grid_position(neighbours[i])
					|| terrain->is_wall(neighbours[i])
					|| !is_clear_path(r, c, neighbours[i].row, neighbours[i].col))
					continue;

				float newInfluence = (i < 4)
					? static_cast<float>(layer.get_value(neighbours[i]) * exp(-1 * 0.1 * decay))
					: static_cast<float>(layer.get_value(neighbours[i]) * exp(-1 * 0.141 * decay));

				layer.set_value(neighbours[i], newInfluence);

				if (layer.get_value(neighbours[i]) > maxNeighbourInfluence)
					maxNeighbourInfluence = layer.get_value(neighbours[i]);
			}

			float newVal = lerp(layer.get_value(r, c), maxNeighbourInfluence, growth);

			tempLayer[r][c] = (1.0f - growth) * newVal + growth * maxNeighbourInfluence;

			if (terrain->is_wall(r, c))
				tempLayer[r][c] = 0.0f;

		}
	}

	for (int r = 0; r < terrain->get_map_height(); ++r)
		for (int c = 0; c < terrain->get_map_width(); ++c)
			layer.set_value(r, c, tempLayer[r][c]);

}

void propagate_dual_occupancy(MapLayer<float>& layer, float decay, float growth)
{
	/*
		Similar to the solo version, but the values range from -1.0 to 1.0, instead of 0.0 to 1.0

		For every cell in the given layer:

		1) Get the value of each neighbor and apply decay factor
		2) Keep the highest ABSOLUTE value from step 1
		3) Linearly interpolate from the cell's current value to the value from step 2
		   with the growing factor as a coefficient.  Make use of the lerp helper function.
		4) Store the value from step 3 in a temporary layer.
		   A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

		After every cell has been processed into the temporary layer, write the temporary layer into
		the given layer;
	*/

	//no need to implement

}

void normalize_solo_occupancy(MapLayer<float>& layer)
{
	/*
		Determine the maximum value in the given layer, and then divide the value
		for every cell in the layer by that amount.  This will keep the values in the
		range of [0, 1].  Negative values should be left unmodified.
	*/

	float maxNeighbourInfluence = 0.0f;
	std::array<	std::array<float, 40>, 40> tempLayer;

	for (int r = 0; r < terrain->get_map_height(); ++r)
		for (int c = 0; c < terrain->get_map_width(); ++c)
			if (maxNeighbourInfluence < layer.get_value(r, c))
				maxNeighbourInfluence = layer.get_value(r, c);

	std::cout << maxNeighbourInfluence << std::endl;

	for (int r = 0; r < terrain->get_map_height(); ++r)
	{
		for (int c = 0; c < terrain->get_map_width(); ++c)
		{
			std::array<GridPos, 8> neighbours
			{
				GridPos{r + 1, c},
				GridPos{r, c + 1},
				GridPos{r - 1, c},
				GridPos{r, c - 1},

				GridPos{r + 1, c + 1},
				GridPos{r - 1, c + 1},
				GridPos{r - 1, c - 1},
				GridPos{r + 1, c - 1}
			};

			for (int i = 0; i < neighbours.size(); ++i)
			{
				if (!terrain->is_valid_grid_position(neighbours[i])
					|| terrain->is_wall(neighbours[i])
					|| !is_clear_path(r, c, neighbours[i].row, neighbours[i].col))
					continue;

				float newInfluence = (i < 4)
					? static_cast<float>(layer.get_value(neighbours[i]) * exp(0.1 * 1 / maxNeighbourInfluence))
					: static_cast<float>(layer.get_value(neighbours[i]) * exp(0.141 * 1 / maxNeighbourInfluence));

				tempLayer[neighbours[i].row][neighbours[i].col] = newInfluence;

			}

			if (terrain->is_wall(r, c))
				tempLayer[r][c] = 0.0f;
		}
	}

	for (int r = 0; r < terrain->get_map_height(); ++r)
	{
		for (int c = 0; c < terrain->get_map_width(); ++c)
		{
			if (tempLayer[r][c] <= 0.0f)
				tempLayer[r][c] = 0.0f;
			else if (tempLayer[r][c] >= 1.0f)
				tempLayer[r][c] = 1.0f;

			layer.set_value(r, c, tempLayer[r][c]);
		}
	}

}

void normalize_dual_occupancy(MapLayer<float>& layer)
{
	/*
		Similar to the solo version, but you need to track greatest positive value AND
		the least (furthest from 0) negative value.

		For every cell in the given layer, if the value is currently positive divide it by the
		greatest positive value, or if the value is negative divide it by -1.0 * the least negative value
		(so that it remains a negative number).  This will keep the values in the range of [-1, 1].
	*/

	// no need to implement
}

void enemy_field_of_view(MapLayer<float>& layer, float fovAngle, float closeDistance, float occupancyValue, AStarAgent* enemy)
{
	/*
		First, clear out the old values in the map layer by setting any negative value to 0.
		Then, for every cell in the layer that is within the field of view cone, from the
		enemy agent, mark it with the occupancy value.  Take the dot product between the view
		vector and the vector from the agent to the cell, both normalized, and compare the
		cosines directly instead of taking the arccosine to avoid introducing floating-point
		inaccuracy (larger cosine means smaller angle).

		If the tile is close enough to the enemy (less than closeDistance),
		you only check if it's visible to enemy.  Make use of the is_clear_path
		helper function.  Otherwise, you must consider the direction the enemy is facing too.
		This creates a radius around the enemy that the player can be detected within, as well
		as a fov cone.
	*/

	for (int r = 0; r < terrain->get_map_height(); ++r)
		for (int c = 0; c < terrain->get_map_width(); ++c)
			if (layer.get_value(r, c) < 0.0f)
				layer.set_value(r, c, 0.0f);

	Vec2 enemyView = { enemy->get_forward_vector().x, enemy->get_forward_vector().z };
	enemyView.Normalize();

	for (int r = 0; r < terrain->get_map_height(); ++r)
	{
		for (int c = 0; c < terrain->get_map_width(); ++c)
		{
			Vec2 currCell{ terrain->get_world_position(r,c).x, terrain->get_world_position(r,c).z };
			Vec2 cellToEnemy{ currCell.x - enemy->get_position().x, currCell.y - enemy->get_position().z };
			cellToEnemy.Normalize();

			float dotProd = enemyView.Dot(cellToEnemy);

			//if(dotProd >= 0.0f)
			//TODO check dot prod against angle dot value (can't remember how to do this) 
			//set value to occuancy value if is within fov angle
		}
	}

	for (int r = 0; r < terrain->get_map_height(); ++r)
	{
		for (int c = 0; c < terrain->get_map_width(); ++c)
		{
			Vec2 currCell{ terrain->get_world_position(r,c).x, terrain->get_world_position(r,c).z };
			Vec2 cellToEnemy{ currCell.x - enemy->get_position().x, currCell.y - enemy->get_position().z };
			cellToEnemy.Normalize();

			GridPos enemyPosWP = terrain->get_grid_position(enemy->get_position());

			float dotProd = enemyView.Dot(cellToEnemy);
			float fov = (float)cos(PI * 1.01);

			if (dotProd > fov && dotProd > 0)
				if (is_clear_path(enemyPosWP.row, enemyPosWP.col, r, c))
					layer.set_value(r, c, 1);
		}
	}

}

bool enemy_find_player(MapLayer<float>& layer, AStarAgent* enemy, Agent* player)
{
	/*
		Check if the player's current tile has a negative value, ie in the fov cone
		or within a detection radius.
	*/

	const auto& playerWorldPos = player->get_position();

	const auto playerGridPos = terrain->get_grid_position(playerWorldPos);

	// verify a valid position was returned
	if (terrain->is_valid_grid_position(playerGridPos) == true)
	{
		if (layer.get_value(playerGridPos) < 0.0f)
		{
			return true;
		}
	}

	// player isn't in the detection radius or fov cone, OR somehow off the map
	return false;
}

bool enemy_seek_player(MapLayer<float>& layer, AStarAgent* enemy)
{
	/*
		Attempt to find a cell with the highest nonzero value (normalization may
		not produce exactly 1.0 due to floating point error), and then set it as
		the new target, using enemy->path_to.

		If there are multiple cells with the same highest value, then pick the
		cell closest to the enemy.

		Return whether a target cell was found.
	*/

	// WRITE YOUR CODE HERE

	return false; // REPLACE THIS
}
