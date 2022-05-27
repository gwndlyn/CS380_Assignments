#pragma once
#include "BehaviorNode.h"
#include "Misc/NiceTypes.h"

class L_Stand : public BaseNode<L_Stand>
{
protected:
	virtual void on_enter() override;
	virtual void on_update(float dt) override;

private:
	Vec3 targetPoint;
	const float height = 0.0f;
};
