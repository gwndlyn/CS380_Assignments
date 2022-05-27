#pragma once
#include "BehaviorNode.h"

class D_IfCustomerIsInRange : public BaseNode<D_IfCustomerIsInRange>
{
protected:
	virtual void on_update(float dt) override;

private:
	float range = 20.0f;
};

