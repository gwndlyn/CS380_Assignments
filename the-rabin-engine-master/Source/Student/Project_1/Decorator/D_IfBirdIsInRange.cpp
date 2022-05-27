#include "pch.h"
#include "D_IfBirdIsInRange.h"

void D_IfBirdIsInRange::on_update(float dt)
{
	const char* birdStr = "BirdAgent";
	Vec3 birdPos = agent->get_blackboard().get_value<Vec3>(birdStr);
	Vec3 vecFromBird = (birdPos - agent->get_position());
	float distFromBird = vecFromBird.Distance(birdPos, agent->get_position());

	BehaviorNode* child = children.front();
	child->tick(dt);

	if (distFromBird <= range)
	{
		on_success();
	}
	else
	{
		on_failure();
	}
}
