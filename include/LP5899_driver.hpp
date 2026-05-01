#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "led_panel_types.hpp"

/// LP5890 LED driver controller for 8-line × 15-channel panel.
/// Manages SPI communication, frame buffering, and hardware synchronization.
class LP5899_PanelDriver
{
public:
  /// @param csPin   Chip select GPIO
  /// @param drdyPin Data ready interrupt GPIO
  /// @param faultPin Fault status GPIO
  /// @param spi     SPI interface reference
  LP5899_PanelDriver(uint8_t csPin,
                     uint8_t drdyPin,
                     uint8_t faultPin,
                     SPIClass& spi);

  /// Configure SPI clock speed.
  void begin(uint32_t spiHz);
  /// Initialize LP5890 device (chip reset, configuration).
  bool init();

  /// Read and optionally print device ID register.
  bool debugReadDeviceId(Stream* dbg = nullptr);
  /// Read and optionally print status register.
  bool debugReadStatus(Stream* dbg = nullptr);

  /// Check fault pin state.
  bool isFaultActive() const;
  /// Check if DRDY signal indicates ready for new frame.
  bool isReadyToUpdate() const;

  /// Clear internal frame buffer (all LEDs off).
  void clear();
  /// Buffer new frame data (no hardware update yet).
  void loadFrame(const ScanFrame& frame);

  /// Print frame buffer contents to serial.
  void dumpScanFrame(Stream& s) const;
  /// Print GPIO pin assignments.
  void printPins(Stream& s) const;

  /// Debug: Set all LEDs to specified brightness.
  bool debugRawAllOn(uint16_t level = 0xFFFF);
  /// Debug: Turn off all LEDs.
  bool debugRawAllOff();
  /// Debug: Send VSYNC pulse only.
  bool debugVsyncOnly();
  /// Debug: Set single LED intensity.
  bool debugRawSingle(uint8_t line,
                      uint8_t channel,
                      uint8_t color,
                      uint16_t level = 0x0FFF);
  /// Debug: Read register from chip.
  bool debugReadRegister(uint8_t address, uint16_t& value);
  /// Debug: Write register to chip.
  bool debugWriteRegister(uint8_t address, uint16_t value);

  /// Push buffered frame to hardware (via SPI). Optionally wait for DRDY.
  bool update(bool requireDrdy = false);

private:
  static constexpr uint8_t kLp5890ScanLines = 16;      ///< LP5890 scan line count
  static constexpr uint8_t kLp5890RgbChannels = 16;    ///< RGB channels per line

  SPIClass& _spi;                                       ///< SPI bus reference
  uint8_t _cs, _drdy, _fault;                          ///< Chip select, data ready, fault pins
  uint32_t _spiHz = 1000000;                           ///< SPI clock speed
  uint8_t _activeSpiMode = SPI_MODE1;                 ///< Current SPI mode
  bool _spiAckDisabled = false;                        ///< ACK feature status
  bool _initialized = false;                           ///< Init completion flag

  ScanFrame _frame;                                     ///< Buffered frame data

  /// Drive chip select low.
  void csLow_();
  /// Drive chip select high.
  void csHigh_();

  /// Send/receive single 16-bit word via SPI.
  uint16_t spiTransferWord_(uint16_t word);
  /// Send/receive multiple 16-bit words via SPI.
  void spiTransferWords_(const uint16_t* tx, uint16_t* rx, size_t count);
  /// SPI transfer with mode switching.
  bool runTransferWithMode_(uint8_t spiMode,
                            const uint16_t* tx,
                            uint16_t* rx,
                            uint16_t count);
  /// Print word array for debugging.
  static void debugDumpWords_(const __FlashStringHelper* tag,
                              const uint16_t* words,
                              uint16_t count);

  /// Compute CRC16-CCITT-FALSE for data validation.
  uint16_t crc16CcittFalse_(const uint16_t* words, size_t count) const;

  /// Low-level CCSI protocol transfer (command + optional data/response).
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

  /// Write consecutive registers.
  bool lp5899WriteRegisters_(uint8_t startAddress, const uint16_t* values, uint16_t count);
  /// Read consecutive registers.
  bool lp5899ReadRegisters_(uint8_t startAddress, uint16_t* values, uint16_t count);
  /// Write register and verify success (with retry).
  bool lp5899WriteRegisterVerified_(uint8_t address, uint16_t value, uint8_t retries = 3);
  /// Issue soft reset to device.
  bool lp5899SoftReset_();

  /// Send CCSI write transaction.
  bool ccsiWrite_(uint16_t headWord, const uint16_t* dataWords, uint16_t wordCount);
  /// Setup LP5890 (config, SRAM load, enable).
  bool lp5890Init_();
  /// Trigger vertical sync (frame update).
  bool lp5890Vsync_();

  /// Query device ID register.
  bool getDeviceId_(uint16_t& devid);
  /// Query status register.
  bool getStatus_(uint16_t& status);

  /// Load scan frame data to device SRAM.
  bool writeSRAM_();
};