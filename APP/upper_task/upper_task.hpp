#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "task.h"
#include "topics.hpp"
#include "topic_pool.hpp"

void VPTask(void *argument);
