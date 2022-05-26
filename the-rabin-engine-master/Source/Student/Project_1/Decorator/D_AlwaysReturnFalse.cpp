#include <pch.h>
#include "D_AlwaysReturnFalse.h"

void D_AlwaysReturnFalse::on_update(float dt)
{
	BehaviorNode* child = children.front();
	child->tick(dt);
	set_status(NodeStatus::SUSPENDED);
	set_result(NodeResult::SUCCESS);

	on_failure();
}
