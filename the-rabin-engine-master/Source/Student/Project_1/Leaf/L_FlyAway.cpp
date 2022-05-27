#include "pch.h"
#include "L_FlyAway.h"

void L_FlyAway::on_enter()
{
	targetPoint = RNG::world_position();
	targetHeight = static_cast<float>(RNG::d10() * 20.0f);
	targetPoint.y = targetHeight;

	BehaviorNode::on_leaf_enter();

}

void L_FlyAway::on_update(float dt)
{
	const auto result = agent->move_toward_point(targetPoint, dt);

	if (result == true)
	{
		on_success();
	}

	display_leaf_text();
}

