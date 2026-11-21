/**
 * @file fsm_task.cpp
 * @author ErenH
 * @brief 全局比赛流程状态机
 * @version 0.1
 * @date 2026-11-07
 *
 * 
 * @note 状态树：
 *
 *      Root
 *       ├─ Disabled        急停 / 遥控失联 / 未就绪：输出全零
 *       ├─ Idle            待命：底盘不动，等比赛阶段
 *       └─ Running         比赛中
 *           ├─ Auto        自动阶段
 *         
 *          
 */

#define HFSM2_DISABLE_TYPEINDEX
#include <hfsm2/machine.hpp>

#include "fsm_task.h"
#include "rm_pocket.hpp"
#include "topic_pool.h"
#include "topics.hpp"

osThreadId_t FsmTaskHandle;

#include <cmath>

namespace {



//  Context：状态之间、状态与任务之间共享的数据
struct FsmContext {
   
    bool  rc_alive   = false;
    bool  estop      = false;
    int   stage      = 0;        // 0=待命 1=自动
   
    pub_chassis_cmd cmd{};
};


// 结构声明：编译期就固定下来，运行时不能增删状态
//    PeerRoot：根下面的几个并列分支
//    Composite：子状态机，同一时刻只有一个孩子活着

using M = hfsm2::MachineT<hfsm2::Config::ContextT<FsmContext>>;

struct Disabled;
struct Idle;
struct Running;
struct Auto;

using FSM = M::PeerRoot<
    Disabled,
    Idle,
    M::Composite<Running,Auto>
>;

// 事件：遥控掉线，边沿事件适合用 react()
struct RcLostEvent {};

struct Disabled : FSM::State {
    void enter(Control& control) {
        control.context().cmd = pub_chassis_cmd{};   
    }
    void update(FullControl& control) {
        const auto& x = control.context();
        if (x.estop || !x.rc_alive) return;
        control.changeTo<Idle>();
    }
};



struct Idle : FSM::State {
    void enter(Control& control) {
        control.context().cmd = pub_chassis_cmd{};
    }
    void update(FullControl& control) {
        auto& x = control.context();
        x.cmd = pub_chassis_cmd{};
        if (x.estop) { control.changeTo<Disabled>(); return; }
        if (x.stage != 0) control.changeTo<Running>();
    }
    void react(const RcLostEvent&, EventControl& control) {
        control.changeTo<Disabled>();
    }
    using FSM::State::react; 
};


struct Running : FSM::State {
    void update(FullControl& control) {
        const auto& x = control.context();

        if (x.estop)
        {
            control.changeTo<Disabled>();
            return;
        }
        if (x.stage == 0)
        {
           control.changeTo<Idle>();
           return;
        }
        if (x.stage == 1)
        {
            control.changeTo<Auto>();
            return;
        }   
    }
    void react(const RcLostEvent&, EventControl& control) {
        control.changeTo<Disabled>();
    }
    using FSM::State::react;
};


struct Auto : FSM::State {
    void update(FullControl& control) {
        // 示例：自动阶段先给个恒定前进
        control.context().cmd = pub_chassis_cmd{0.5f, 0.0f, 0.0f};
    }
};


//实例与通信
FsmContext g_context{};
FSM::Instance g_fsm{g_context};  

TypedTopicSubscriber<pub_RM_Data>     g_rm_sub("rm", 8);
TypedTopicPublisher<pub_chassis_cmd>  fsm_chassis_cmd("fsm_chassis_cmd");
pub_RM_Data g_rm{};

}  // namespace

// 调试器 watch 窗口里看状态用
volatile float fsm_watch[4] = {0};


void fsmTask(void *argument) {
    (void)argument;

    constexpr TickType_t kRcTimeout = pdMS_TO_TICKS(500);
    TickType_t currentTime = xTaskGetTickCount();
    TickType_t rcUpdateTime = currentTime - kRcTimeout - 1;
    
    for (;;) {
        const TickType_t now = xTaskGetTickCount();

        
        if (g_rm_sub.TryGet(&g_rm)) {
            rcUpdateTime = now;
            g_context.estop  = (g_rm.swA == RM_2_POS_SW_State_t::DOWN);
           
            switch (g_rm.swD) {
                case RM_2_POS_SW_State_t::UP:     g_context.stage = 0; break;
                case RM_2_POS_SW_State_t::DOWN:   g_context.stage = 1; break;
            }
        }

        const bool alive_now = (now - rcUpdateTime) <= kRcTimeout;
        if (g_context.rc_alive && !alive_now) {
            g_fsm.react(RcLostEvent{});  
        }
        g_context.rc_alive = alive_now;

        
        g_fsm.update();


        fsm_watch[0] = g_fsm.isActive<Disabled>() ? 1.0f : 0.0f;
        fsm_watch[1] = g_fsm.isActive<Idle>()     ? 1.0f : 0.0f;
        fsm_watch[2] = g_fsm.isActive<Running>()  ? 1.0f : 0.0f;
        fsm_watch[3] = g_fsm.isActive<Auto>()     ? 1.0f : 0.0f;

        vTaskDelayUntil(&currentTime, pdMS_TO_TICKS(5));
    }
}