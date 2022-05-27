#pragma once
#include "BehaviorNode.h"

class D_IfOwnerIsInRange : public BaseNode<D_IfOwnerIsInRange>
{
protected:
	virtual void on_update(float dt) override;

private:
	float range = 30.0f;
};

