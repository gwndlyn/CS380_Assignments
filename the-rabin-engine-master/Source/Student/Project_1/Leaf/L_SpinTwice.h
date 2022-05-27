#pragma once
#include "BehaviorNode.h"
#include "Misc/NiceTypes.h"

class L_SpinTwice : public BaseNode<L_SpinTwice>
{
protected:
	virtual void on_update(float dt) override;

private:
	Vec3 currOwnerPos;
	const float spinAnglePerMin = 720.0f;
	float timer;
};

