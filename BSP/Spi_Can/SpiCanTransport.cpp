/**
 * @file SpiCanTransport.cpp
 * @brief ICanTransport 的 SPI 实现
 */
#include "SpiCanTransport.hpp"

namespace BspCan {

namespace {
constexpr uint32_t kSpiTimeoutMs = 10;
}

SpiCanTransport::SpiCanTransport(SPI_HandleTypeDef &hspi, const SpiCanPins &pins,
                                 const Options &opt)
    : hspi_(hspi), pins_(pins), opt_(opt),
      chip_(SpiPort{this, &SpiCanTransport::spiXfer}) {}

void SpiCanTransport::select(bool active) {
  if (pins_.csPort == nullptr) {
    return;
  }
  HAL_GPIO_WritePin(pins_.csPort, pins_.csPin,
                    active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

bool SpiCanTransport::spiXfer(void *ctx, const uint8_t *tx, uint8_t *rx,
                              uint16_t len) {
  auto *self = static_cast<SpiCanTransport *>(ctx);
  if (self == nullptr || tx == nullptr || rx == nullptr || len == 0) {
    return false;
  }

  self->select(true);
  // nCS 拉低到第一个 SCK 上升沿要留建立时间，这里用几次循环兜一下
  for (volatile uint32_t i = 0; i < 4; ++i) {
    __NOP();
  }

  const HAL_StatusTypeDef st =
      HAL_SPI_TransmitReceive(&self->hspi_, const_cast<uint8_t *>(tx), rx, len,
                              kSpiTimeoutMs);

  self->select(false);

  if (st != HAL_OK) {
    // 超时后 SPI 可能停在忙状态，abort 一次让后面的事务还能继续
    HAL_SPI_Abort(&self->hspi_);
    return false;
  }
  return true;
}

bool SpiCanTransport::start() {
  if (pins_.rstPort != nullptr) {
    HAL_GPIO_WritePin(pins_.rstPort, pins_.rstPin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(pins_.rstPort, pins_.rstPin, GPIO_PIN_SET);
    HAL_Delay(2);
  }

  Mcp2518fd::Config cfg;
  cfg.sysclkHz = opt_.sysclkHz;
  cfg.usePll = opt_.usePll;
  cfg.isoCanFd = opt_.isoCanFd;

  // 仲裁段和数据段都要先算出合法时序，算不出来就别往下配了
  if (!Mcp2518fd::calcTiming(opt_.sysclkHz, opt_.nominalBitrate,
                             opt_.nominalSamplePoint, cfg.timing.nominal) ||
      !Mcp2518fd::calcTiming(opt_.sysclkHz, opt_.dataBitrate,
                             opt_.dataSamplePoint, cfg.timing.data)) {
    return false;
  }

  return chip_.init(cfg);
}

bool SpiCanTransport::send(const FdFrame &frame) { return chip_.pushTx(frame); }

bool SpiCanTransport::recv(FdFrame &frame) { return chip_.popRx(frame); }

uint32_t SpiCanTransport::txFreeLevel() {
  // TXQ 只有一级缓冲，塞得进就是 1，满了就是 0
  return chip_.txQueueFull() ? 0U : 1U;
}

void SpiCanTransport::onInterrupt() {
  // 中断里不碰 SPI：8.59MHz 下搬一帧 72 字节约 70us，放在中断里会把控制任务顶歪
  ++irqCount_;
}

bool SpiCanTransport::busHealthy() const { return chip_.busHealthy(); }

void SpiCanTransport::errorCounters(uint8_t &tec, uint8_t &rec) const {
  chip_.errorCounters(tec, rec);
}

} // namespace BspCan
