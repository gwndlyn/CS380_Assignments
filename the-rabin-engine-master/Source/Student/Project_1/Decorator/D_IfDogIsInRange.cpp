#include "pch.h"
#include "D_IfDogIsInRange.h"

void D_IfDogIsInRange::on_update(float dt)
{
	const char* dogStr = "DogAgent";
	Vec3 dogPos = agent->get_blackboard().get_value<Vec3>(dogStr);
	Vec3 vecFromDog = (dogPos - agent->get_position());
	float distFromDog = vecFromDog.Distance(dogPos, agent->get_position());

	BehaviorNode* child = children.front();
	child->tick(dt);

	if (distFromDog <= range)
	{
		on_success();
	}
	else
	{
		on_failure();
	}
}
