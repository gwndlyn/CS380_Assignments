#pragma once

// Include all node headers in this file

// Example Control Flow Nodes
#include "ControlFlow/C_ParallelSequencer.h"
#include "ControlFlow/C_RandomSelector.h"
#include "ControlFlow/C_Selector.h"
#include "ControlFlow/C_Sequencer.h"

// Student Control Flow Nodes


// Example Decorator Nodes
#include "Decorator/D_Delay.h"
#include "Decorator/D_InvertedRepeater.h"
#include "Decorator/D_RepeatFourTimes.h"

// Student Decorator Nodes
#include "Decorator/D_AlwaysReturnFalse.h"
#include "Decorator/D_IfDogIsInRange.h" //fly away from dog
//#include "Decorator/D_IfBirdIsInRange.h" //chase bird

// Example Leaf Nodes
#include "Leaf/L_CheckMouseClick.h"
#include "Leaf/L_Idle.h"
#include "Leaf/L_MoveToFurthestAgent.h"
#include "Leaf/L_MoveToMouseClick.h"
#include "Leaf/L_MoveToRandomPosition.h"

// Student Leaf Nodes
#include "Leaf/L_StoreValueInBlackboard.h"
#include "Leaf/L_FlyAway.h"
#include "Leaf/L_Land.h"
//#include "Leaf/L_SpinTwice.h"
//#include "Leaf/L_Sit.h"
//#include "Leaf/L_Stand.h"
