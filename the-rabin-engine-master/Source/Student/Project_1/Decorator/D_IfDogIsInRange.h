#pragma once
#include "BehaviorNode.h"

class D_IfDogIsInRange : public BaseNode<D_IfDogIsInRange>
{
protected:
	virtual void on_update(float dt) override;

private:
	float range = 20.0f;
};

