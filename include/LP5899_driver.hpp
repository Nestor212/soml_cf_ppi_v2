#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "led_panel_types.hpp"

class LP5899_PanelDriver
{
public:
  LP5899_PanelDriver(uint8_t csPin,
                     uint8_t drdyPin,
                     uint8_t faultPin,
                     SPIClass& spi);

  void begin(uint32_t spiHz);
  bool init();

  bool debugReadDeviceId(Stream* dbg = nullptr);
  bool debugReadStatus(Stream* dbg = nullptr);

  bool isFaultActive() const;
  bool isReadyToUpdate() const;

  void clear();
  void loadFrame(const ScanFrame& frame);

  void dumpScanFrame(Stream& s) const;
  void printPins(Stream& s) const;

  bool debugRawAllOn(uint16_t level = 0xFFFF);
  bool debugRawAllOff();
  bool debugVsyncOnly();
  bool debugRawSingle(uint8_t line,
                      uint8_t channel,
                      uint8_t color,
                      uint16_t level = 0x0FFF);
  bool debugReadRegister(uint8_t address, uint16_t& value);
  bool debugWriteRegister(uint8_t address, uint16_t value);

  bool update(bool requireDrdy = false);

private:
  static constexpr uint8_t kLp5890ScanLines = 16;
  static constexpr uint8_t kLp5890RgbChannels = 16;

  SPIClass& _spi;
  uint8_t _cs, _drdy, _fault;
  uint32_t _spiHz = 1000000;
  uint8_t _activeSpiMode = SPI_MODE1;
  bool _spiAckDisabled = false;
  bool _initialized = false;

  ScanFrame _frame;

  void csLow_();
  void csHigh_();

  uint16_t spiTransferWord_(uint16_t word);
  void spiTransferWords_(const uint16_t* tx, uint16_t* rx, size_t count);
  bool runTransferWithMode_(uint8_t spiMode,
                            const uint16_t* tx,
                            uint16_t* rx,
                            uint16_t count);
  static void debugDumpWords_(const __FlashStringHelper* tag,
                              const uint16_t* words,
                              uint16_t count);

  uint16_t crc16CcittFalse_(const uint16_t* words, size_t count) const;

  bool lp5899Transfer_(uint16_t command,
                       const uint16_t* dataWords,
                       uint16_t dataLength,
                       uint8_t address,
                       bool includeData,
                       bool captureResponse,
                       bool checkResponseCrc,
                       uint16_t* responseWords,
                       uint16_t responseCapacity,
                       uint16_t* receivedCrc = nullptr);

  bool lp5899WriteRegisters_(uint8_t startAddress, const uint16_t* values, uint16_t count);
  bool lp5899ReadRegisters_(uint8_t startAddress, uint16_t* values, uint16_t count);
  bool lp5899WriteRegisterVerified_(uint8_t address, uint16_t value, uint8_t retries = 3);
  bool lp5899SoftReset_();

  bool ccsiWrite_(uint16_t headWord, const uint16_t* dataWords, uint16_t wordCount);
  bool lp5890Init_();
  bool lp5890Vsync_();

  bool getDeviceId_(uint16_t& devid);
  bool getStatus_(uint16_t& status);

  bool writeSRAM_();
};