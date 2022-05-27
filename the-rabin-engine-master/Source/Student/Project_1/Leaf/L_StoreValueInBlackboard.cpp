#include "pch.h"
#include "L_StoreValueInBlackboard.h"

void L_StoreValueInBlackboard::on_update(float dt)
{
	const auto& allAgents = agents->get_all_agents();
	for (const auto& a : allAgents)
	{
		if (a != agent)
		{
			//std::cout << "stored " << a->get_type() << " pos in blackboard" << std::endl;
			agent->get_blackboard().set_value(a->get_type(), a->get_position());
		}
	}

	on_success();
}
