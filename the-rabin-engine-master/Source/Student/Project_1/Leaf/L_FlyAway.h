#pragma once
#include "BehaviorNode.h"
#include "Misc/NiceTypes.h"

class L_FlyAway : public BaseNode<L_FlyAway>
{
protected:
	virtual void on_enter() override;
	virtual void on_update(float dt) override;

private:
	Vec3 targetPoint;
	float targetHeight = 10.0f;
};

