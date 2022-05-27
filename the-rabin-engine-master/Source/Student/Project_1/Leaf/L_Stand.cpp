#include "pch.h"
#include "L_Stand.h"

void L_Stand::on_enter()
{
	targetPoint = agent->get_position();
	targetPoint.y = height;

	BehaviorNode::on_leaf_enter();
}

void L_Stand::on_update(float dt)
{
	const auto result = agent->move_toward_point(targetPoint, dt);

	if (result == true)
	{
		on_success();
	}

	display_leaf_text();
}
