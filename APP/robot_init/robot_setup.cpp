#include "chassis_task.h"
#include "Axis.hpp"
#include "Motor.hpp"
#include "chassis_solution.hpp"
#include "com_config.h"
#include "pid_controller.hpp"
#include "tool.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include <array>

PidParam M2006_P38_SpeedPid(3000.0f, 8600.0f, 0.0f, 10000.0f, 0.0f, IMCREATEMENT_OF_OUT);
PidParam M2006_P38_PosPid(3.0f,0.4f,0.01f,5*M_PI,0.5f*kDegToRad,NONE);
const PidParam M3508_P19_SpeedPid(4000.0f, 8600.0f, 0.0f, 16000.0f, 0.0f, IMCREATEMENT_OF_OUT);
const PidParam M3508_P19_PosPid(3.0f,0.4f,0.01f,2*M_PI,0.5f*kDegToRad,NONE);
const PidParam U8Lite(0.0f,0.0f,0.0f,0.0f,0.0f,NONE);