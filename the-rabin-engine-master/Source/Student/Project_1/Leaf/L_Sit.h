#pragma once
#include "BehaviorNode.h"
#include "Misc/NiceTypes.h"

class L_Sit : public BaseNode<L_Sit>
{
protected:
	virtual void on_enter() override;
	virtual void on_update(float dt) override;

private:
	Vec3 targetPoint;
	const float height = -30.0f;
};
