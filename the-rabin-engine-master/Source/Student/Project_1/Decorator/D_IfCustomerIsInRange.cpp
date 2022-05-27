#include "pch.h"
#include "D_IfCustomerIsInRange.h"

void D_IfCustomerIsInRange::on_update(float dt)
{
	const char* customerStr = "CustomerAgent";
	Vec3 ownerPos = agent->get_blackboard().get_value<Vec3>(customerStr);
	Vec3 vecFromCustomer = (ownerPos - agent->get_position());
	float distFromCustomer = vecFromCustomer.Distance(ownerPos, agent->get_position());

	BehaviorNode* child = children.front();
	child->tick(dt);

	if (distFromCustomer <= range)
	{
		on_success();
	}
	else
	{
		on_failure();
	}
}
