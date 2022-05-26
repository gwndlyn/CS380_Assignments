#pragma once
#include "BehaviorNode.h"

class D_AlwaysReturnFalse : public BaseNode<D_AlwaysReturnFalse>
{
protected:
    virtual void on_update(float dt) override;

};