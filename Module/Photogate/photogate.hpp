/**
 * @file photogate.hpp
 * @author CZhan
 * @brief
 * @version 1.0
 * @date 2026-11-26
 *
 * @copyright
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include "task.h"
#include "topics.hpp"
#include "topic_pool.hpp"
#include <cstdint>

class Photogate {
public:
    Photogate(const GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,
              float confirm_pass_time, float detect_direction)
            :GPIOx_(GPIOx),GPIO_Pin_(GPIO_Pin),confirm_pass_time_(confirm_pass_time),detect_direction_(detect_direction){
            }

    void Update() {
        dt = DWT_GetDeltaT(&DWT_CNT);
        last_detected_ = detected_;
        detected_ = HAL_GPIO_ReadPin(GPIOx_, GPIO_Pin_) ==
                    (detect_direction_ ? GPIO_PIN_RESET : GPIO_PIN_SET);

        if (!is_init || dt >= confirm_pass_time_ || !detected_) {
            is_init = true;
            confirm_time_ = 0.0f;
            Potogate_judgement = false;
            return;
        }

        if (last_detected_ && !Potogate_judgement) {
            confirm_time_ += dt;
            Potogate_judgement = confirm_time_ >= confirm_pass_time_;
        }
    }
    
    bool getPotogate(){
        return Potogate_judgement;
    }


private:
    const GPIO_TypeDef* GPIOx_;
    uint16_t GPIO_Pin_;
    float confirm_pass_time_;
    bool detect_direction_;

    bool detected_{false};
    bool last_detected_{false};
    bool Potogate_judgement{0};

    float confirm_time_{0};
    float dt{0};
    uint32_t DWT_CNT{0};

    bool is_init{0};
};
