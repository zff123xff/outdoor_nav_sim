//
// Created by libo on 2024/05/11.
//

#ifndef UAV_CTL_STATE_H
#define UAV_CTL_STATE_H

#include <cstdint>

enum CtlState {
    TakeOff = 2,
    Hover = 3,
    Land = 4,
    FollowCancel = 12,
    AttackGoal = 14,
    AlgoPointAttack = 28,
    Armed = 1000,
    DisArmed = 1001,
    Return = 1002  
};

#endif //UAV_CTL_STATE_H