#include "pch.h"
#include "D_IfOwnerIsInRange.h"

void D_IfOwnerIsInRange::on_update(float dt)
{
	const char* ownerStr = "OwnerAgent";
	Vec3 ownerPos = agent->get_blackboard().get_value<Vec3>(ownerStr);
	Vec3 vecFromOwner = (ownerPos - agent->get_position());
	float distFromOwner = vecFromOwner.Distance(ownerPos, agent->get_position());

	BehaviorNode* child = children.front();
	child->tick(dt);

	if (distFromOwner <= range)
	{
		on_success();
	}
	else
	{
		on_failure();
	}
}
