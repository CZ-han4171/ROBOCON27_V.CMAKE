#pragma once

#include <cstdint>

class HWT101CT {
public:
  static constexpr uint8_t kFrameHeader0 = 0x55;
  static constexpr uint8_t kFrameIdGyro = 0x52;
  static constexpr uint8_t kFrameIdAngle = 0x53;
  static constexpr uint8_t kFrameLength = 11;

  struct Data {
    int16_t rwz_raw{0};
    int16_t wz_raw{0};
    float rwz{0.0f};
    float wz_dps{0.0f};

    int16_t roll_raw{0};
    int16_t pitch_raw{0};
    int16_t yaw_raw{0};
    float roll_deg{0.0f};
    float pitch_deg{0.0f};
    float yaw_deg{0.0f};

    uint8_t last_frame_id{0};
    uint32_t frame_count{0};
  };

  uint8_t init();

  // Feed one UART byte. Returns the decoded frame id, or 0 when no frame is ready.
  uint8_t processByte(uint8_t byte);

  const Data &getData() const { return data_; }
  bool frameReady() const { return frame_ready_; }
  void clearFrameReady() { frame_ready_ = false; }
  void resetState();

private:
  enum class RxState {
    WaitingHeader0,
    WaitingHeader1,
    ReceivingFrame,
  };

  static bool checkSum(const uint8_t *frame);
  static bool isSupportedFrameId(uint8_t frame_id);
  static int16_t readInt16LE(uint8_t low, uint8_t high);

  void parseFrame();

  RxState rx_state_{RxState::WaitingHeader0};
  uint8_t frame_[kFrameLength]{};
  uint8_t frame_index_{0};
  Data data_{};
  bool frame_ready_{false};
};
