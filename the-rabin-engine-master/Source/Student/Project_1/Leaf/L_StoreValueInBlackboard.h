#pragma once
#include "BehaviorNode.h"

class L_StoreValueInBlackboard : public BaseNode<L_StoreValueInBlackboard>
{
protected:
	virtual void on_update(float dt) override;

};
