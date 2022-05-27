#include "pch.h"
#include "L_FollowOwner.h"

void L_FollowOwner::on_enter()
{
	agent->set_movement_speed(20.0f);

	BehaviorNode::on_leaf_enter();
}

void L_FollowOwner::on_update(float dt)
{
	const char* ownerStr = "OwnerAgent";
	currOwnerPos = agent->get_blackboard().get_value<Vec3>(ownerStr);
	currOwnerPos.y = 0.0f;

	//const auto result = 
	agent->move_toward_point(currOwnerPos, dt);
	followTimer += dt;

	//if (result == true)
	if (followTimer >= maxFollowTime)
	{
		on_success();
	}

	display_leaf_text();
}

void L_FollowOwner::on_exit()
{
	agent->set_movement_speed(30.0f);

	BehaviorNode::on_exit();
}
