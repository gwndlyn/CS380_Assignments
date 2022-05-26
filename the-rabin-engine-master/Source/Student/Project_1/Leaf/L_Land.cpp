#include "pch.h"
#include "L_Land.h"

void L_Land::on_enter()
{
	targetPoint = agent->get_position();
	targetPoint.y = targetHeight;

	BehaviorNode::on_leaf_enter();

}

void L_Land::on_update(float dt)
{
	const auto result = agent->move_toward_point(targetPoint, dt);

	if (result == true)
	{
		std::cout << "L_Land" << std::endl;
		on_success();
	}

	display_leaf_text();
}
