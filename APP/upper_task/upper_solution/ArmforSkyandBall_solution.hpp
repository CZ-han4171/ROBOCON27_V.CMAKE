#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <type_traits>

#include "Axis.hpp"
#include "ControlOptimize.hpp"
#include "SLM_solution.hpp"
#include "stm32h723xx.h"
#include "topic_pool.h"
#include "tool.hpp"


class ArmforSkyandBall
{
public:
    enum MotorIndex : size_t {EVP = 0, EClaw = 1, Claw = 2, kCount = 3};
    enum SLMIndex : size_t {DL = 0, ML = 1, kSLMCount = 2};
    enum ControlMode : size_t {SPEEDPLAN = 0, INTERPOLATION = 1};
    ArmforSkyandBall(LinearAxis &DL_L, LinearAxis &DL_R, LinearAxis &ML_L,LinearAxis &ML_R, 
        LinearAxis &EVP, LinearAxis &EClaw, LinearAxis &Claw, 
        ControlMode controlmode = SPEEDPLAN,ControlOptimizer::Control_Improvement control_improvement = ControlOptimizer::Control_Improvement::COSPLAN)
    :SLMs_{SLM{DL_L, DL_R}, SLM{ML_L, ML_R}},motors_{&EVP, &EClaw, &Claw}{
        controlmode_ = controlmode;
    };
private:
    std::array<SLM, kSLMCount> SLMs_;
    std::array<LinearAxis*, kCount>motors_{};
    

    ControlMode controlmode_;
};
