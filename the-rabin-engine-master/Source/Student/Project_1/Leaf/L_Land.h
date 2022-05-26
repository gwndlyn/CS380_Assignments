#pragma once
#include "BehaviorNode.h"
#include "Misc/NiceTypes.h"

class L_Land : public BaseNode<L_Land>
{
protected:
	virtual void on_enter() override;
	virtual void on_update(float dt) override;

private:
	Vec3 targetPoint;
	float targetHeight = 0.0f;
};
