#include "HWT101CT.hpp"

#include <cstring>

uint8_t HWT101CT::init() {
  resetState();
  data_ = Data{};
  frame_ready_ = false;
  return 1;
}

void HWT101CT::resetState() {
  rx_state_ = RxState::WaitingHeader0;
  frame_index_ = 0;
  std::memset(frame_, 0, sizeof(frame_));
}

uint8_t HWT101CT::processByte(uint8_t byte) {
  switch (rx_state_) {
  case RxState::WaitingHeader0:
    if (byte == kFrameHeader0) {
      frame_[0] = byte;
      frame_index_ = 1;
      rx_state_ = RxState::WaitingHeader1;
    }
    break;

  case RxState::WaitingHeader1:
    if (isSupportedFrameId(byte)) {
      frame_[1] = byte;
      frame_index_ = 2;
      rx_state_ = RxState::ReceivingFrame;
    } else if (byte == kFrameHeader0) {
      frame_[0] = byte;
      frame_index_ = 1;
    } else {
      resetState();
    }
    break;

  case RxState::ReceivingFrame:
    frame_[frame_index_++] = byte;
    if (frame_index_ >= kFrameLength) {
      if (checkSum(frame_)) {
        parseFrame();
        uint8_t frame_id = frame_[1];
        resetState();
        frame_ready_ = true;
        return frame_id;
      }

      if (byte == kFrameHeader0) {
        frame_[0] = byte;
        frame_index_ = 1;
        rx_state_ = RxState::WaitingHeader1;
      } else {
        resetState();
      }
    }
    break;

  default:
    resetState();
    break;
  }

  return 0;
}

bool HWT101CT::checkSum(const uint8_t *frame) {
  if (frame == nullptr) {
    return false;
  }

  uint8_t sum = 0;
  for (uint8_t i = 0; i < kFrameLength - 1; ++i) {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }

  return sum == frame[kFrameLength - 1];
}

bool HWT101CT::isSupportedFrameId(uint8_t frame_id) {
  return frame_id == kFrameIdGyro || frame_id == kFrameIdAngle;
}

int16_t HWT101CT::readInt16LE(uint8_t low, uint8_t high) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

void HWT101CT::parseFrame() {
  data_.last_frame_id = frame_[1];
  ++data_.frame_count;

  if (frame_[1] == kFrameIdGyro) {
    data_.rwz_raw = readInt16LE(frame_[4], frame_[5]);
    data_.wz_raw = readInt16LE(frame_[6], frame_[7]);

    // HWT101CT follows the common WitMotion 16-bit full-scale mapping.
    data_.rwz = static_cast<float>(data_.rwz_raw) / 32768.0f * 180.0f;
    data_.wz_dps = static_cast<float>(data_.wz_raw) / 32768.0f * 2000.0f;
  } else if (frame_[1] == kFrameIdAngle) {
    data_.roll_raw = readInt16LE(frame_[2], frame_[3]);
    data_.pitch_raw = readInt16LE(frame_[4], frame_[5]);
    data_.yaw_raw = readInt16LE(frame_[6], frame_[7]);

    data_.roll_deg = static_cast<float>(data_.roll_raw) / 32768.0f * 180.0f;
    data_.pitch_deg = static_cast<float>(data_.pitch_raw) / 32768.0f * 180.0f;
    data_.yaw_deg = static_cast<float>(data_.yaw_raw) / 32768.0f * 180.0f;
  }
}
