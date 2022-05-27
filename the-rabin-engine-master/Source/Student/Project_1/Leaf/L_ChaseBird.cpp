#include "pch.h"
#include "L_ChaseBird.h"

void L_ChaseBird::on_enter()
{
	const char* birdStr = "BirdAgent";
	currOwnerPos = agent->get_blackboard().get_value<Vec3>(birdStr);
	currOwnerPos.y = 0.0f;

	agent->set_movement_speed(40.0f);

	BehaviorNode::on_leaf_enter();

}

void L_ChaseBird::on_update(float dt)
{
	const auto result = agent->move_toward_point(currOwnerPos, dt);

	if (result == true)
	{
		on_success();
	}

	display_leaf_text();
}

void L_ChaseBird::on_exit()
{
	agent->set_movement_speed(30.0f);

	BehaviorNode::on_exit();
}
