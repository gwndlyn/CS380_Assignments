#pragma once
#include "BehaviorNode.h"

class D_IfBirdIsInRange : public BaseNode<D_IfBirdIsInRange>
{
protected:
	virtual void on_update(float dt) override;

private:
	float range = 30.0f;
};

