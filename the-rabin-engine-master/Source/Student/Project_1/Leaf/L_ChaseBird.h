#pragma once
#include "BehaviorNode.h"
#include "Misc/NiceTypes.h"

class L_ChaseBird : public BaseNode<L_ChaseBird>
{
protected:
	virtual void on_enter() override;
	virtual void on_update(float dt) override;
	virtual void on_exit() override;

private:
	Vec3 currOwnerPos;
};

