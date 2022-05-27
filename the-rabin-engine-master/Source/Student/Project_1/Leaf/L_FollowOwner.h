#pragma once
#include "BehaviorNode.h"
#include "Misc/NiceTypes.h"

class L_FollowOwner : public BaseNode<L_FollowOwner>
{
protected:
	virtual void on_enter() override;
	virtual void on_update(float dt) override;
	virtual void on_exit() override;

private:
	Vec3 currOwnerPos;

	float followTimer;
	const float maxFollowTime = 3.0f;
};

