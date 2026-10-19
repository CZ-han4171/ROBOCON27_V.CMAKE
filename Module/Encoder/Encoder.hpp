
#pragma once

#include "Canbus.hpp"
#include "main.h"
#include <cmath>
#include <cstdint>
#include <math.h>
#include <stdint.h>
#include "bsp_dwt.h"

class BRT38B : public CanDevice {
public:
    BRT38B(CanBus *manager, uint32_t id, bool is_extid, uint32_t tx_id,
            bool tx_is_extid)
      : CanDevice(manager, id, is_extid, tx_id, tx_is_extid) {}

    void init(uint16_t turn_bits = 4096, uint8_t native_max_turns = 16) {
        setTurnBits(turn_bits);
        setNativeMaxTurns(native_max_turns);
        setEncoderRadRatio(turn_bits/(2.0f*M_PI));
    }

    void onRx(const uint8_t *data, uint8_t len) override {
        watch1 = data[1];
        watch2 = raw_total_encoder_;
        if(data[1] != id_ || len < 7)return;

        raw_total_encoder_ = static_cast<int32_t>((static_cast<uint32_t>(data[6]) << 24) |
                                                  (static_cast<uint32_t>(data[5]) << 16) |
                                                  (static_cast<uint32_t>(data[4]) << 8)  |
                                                   static_cast<uint32_t>(data[3]));
        raw_round_cnt_ = raw_total_encoder_/turn_bits_;
        
        if(raw_round_cnt_last_ - raw_round_cnt_ > static_cast<int16_t>(native_max_turns_/2)) round_max_turn_cnt_++;
        else if(raw_round_cnt_last_ - raw_round_cnt_ < static_cast<int16_t>(-native_max_turns_/2))round_max_turn_cnt_--;
        round_cnt_ = round_max_turn_cnt_*native_max_turns_ + raw_round_cnt_;

        single_encoder_  = raw_total_encoder_%turn_bits_;
        total_encoder_ = round_cnt_*turn_bits_ + single_encoder_;

        SinglePos_rad = single_encoder_/encoder_rad_ratio_;
        SumPos_rad = total_encoder_/encoder_rad_ratio_;

        raw_round_cnt_last_ = raw_round_cnt_;
    }

    void setTurnBits(const uint16_t turn_bits){ turn_bits_ = turn_bits; }
    void setNativeMaxTurns(const uint8_t native_max_turns){ native_max_turns_ = native_max_turns; }
    void setEncoderRadRatio(const float encoder_rad_ratio){ encoder_rad_ratio_ = encoder_rad_ratio; }

    float getSumPos_rad(){ return SumPos_rad; }
    float getSinglePos_rad(){ return SinglePos_rad; }

    float watch1;
    float watch2;

private:
    uint16_t turn_bits_;
    uint8_t native_max_turns_;
    uint32_t single_encoder_;
    int32_t total_encoder_;
    int16_t round_cnt_{0};
    int16_t raw_round_cnt_{0};
    int16_t raw_round_cnt_last_{0};
    int16_t round_max_turn_cnt_{0};
    uint32_t raw_total_encoder_;

    float encoder_rad_ratio_;

    float SumPos_rad;
    float SinglePos_rad;
};