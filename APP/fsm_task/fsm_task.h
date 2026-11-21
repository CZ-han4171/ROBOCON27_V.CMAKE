/**
 * @file fsm_task.h
 * @author ErenH
 * @brief 全局比赛流程状态机任务（HFSM2 示例）
 * @version 0.1
 * @date 2026-11-07
 *
 * @note 启用：在 robot_task.cpp 的 osTaskInit() 里新建一个任务跑 fsmTask() 即可。
 */

#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

void fsmTask(void *argument);