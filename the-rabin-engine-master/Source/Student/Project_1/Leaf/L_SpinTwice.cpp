#include "pch.h"
#include "L_SpinTwice.h"

void L_SpinTwice::on_update(float dt)
{
	//get owner pos
	const char* ownerStr = "OwnerAgent";
	currOwnerPos = agent->get_blackboard().get_value<Vec3>(ownerStr);
	currOwnerPos.y = 0.0f;

	//calculate dest while spin
	float offsetAngle = timer * spinAnglePerMin;
	Vec3 targetPoint = Vec3(currOwnerPos.x * cos(offsetAngle) * 100.0f, 0.0f, currOwnerPos.z * sin(offsetAngle) * 100.0f);

	//move to dest
	agent->set_position(targetPoint);
	timer += dt;

	if (timer >= 1.0f)
	{
		on_success();
	}

	display_leaf_text();
}

