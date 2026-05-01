#include "LP5899_driver.hpp"

/**
 * @file LP5899_driver.cpp
 * @brief LP5890 LED driver interface for 8-line × 15-channel panel.
 * 
 * Derived from TI LP5899 reference driver (CCSI protocol stack).
 * Conversion: extracted core register/SRAM/VSYNC operations, removed GPIO
 * complexity, added frame buffering, and wrapped SPI transactions for
 * reliable CRC-validated read/write with configurable SPI mode fallback.
 * 
 * Key adaptations:
 * - Simplified GPIO mapping to PCA9698 (offboard in powerPanel.cpp)
 * - Frame buffer (_frame) decouples state -> hardware updates
 * - CRC validation on all transfers with MODE0 fallback retry
 * - Status polling (DRDYST) for frame synchronization
 */

namespace
{
  // File-local constants and decode helpers.
  // Kept in anonymous namespace to avoid symbol export outside this translation unit.

  // Debug and transport behavior.
  constexpr bool kVerboseLp5899ReadDebug = true;
  constexpr uint8_t kForcedSpiMode = SPI_MODE0;

  // LP5890 forwarded CCSI command words (written through LP5899 bridge).
  constexpr uint16_t W_FC0 = 0xAA00;
  constexpr uint16_t W_FC1 = 0xAA01;
  constexpr uint16_t W_FC2 = 0xAA02;
  constexpr uint16_t W_FC3 = 0xAA03;
  constexpr uint16_t W_FC4 = 0xAA04;
  constexpr uint16_t W_CHIP_INDEX = 0xAA10;
  constexpr uint16_t W_VSYNC = 0xAAF0;
  constexpr uint16_t W_SRAM = 0xAA30;

  // LP5899 CCSI transaction opcodes.
  // These form the header word in lp5899Transfer_(), with length/address packed in.
  constexpr uint16_t FWD_WR_CRC = 0x2000;
  constexpr uint16_t DATA_RD_CRC = 0x8000;
  constexpr uint16_t REG_WR_CRC = 0xA000;
  constexpr uint16_t REG_RD_CRC = 0xC000;
  constexpr uint16_t SOFTRESET_CRC = 0xE1E1;

  // LP5899 register addresses used by this driver.
  constexpr uint8_t REG_DEVID = 0x00;
  constexpr uint8_t REG_SPICTRL = 0x01;
  constexpr uint8_t REG_TXFFLVL = 0x03;
  constexpr uint8_t REG_RXFFLVL = 0x04;
  constexpr uint8_t REG_DEVCTRL = 0x05;
  constexpr uint8_t REG_STATUS = 0x07;
  constexpr uint8_t REG_IFST = 0x08;
  constexpr uint8_t REG_TXFFST = 0x09;
  constexpr uint8_t REG_RXFFST = 0x0A;

  // REG_STATUS bit masks.
  // Used for readiness checks, fault detection, and init-state transitions.
  constexpr uint16_t STATUS_FLAG_CCSI = 0x4000;
  constexpr uint16_t STATUS_FLAG_TXFF = 0x1000;
  constexpr uint16_t STATUS_FLAG_RXFF = 0x0800;
  constexpr uint16_t STATUS_FLAG_SRST = 0x0200;
  constexpr uint16_t STATUS_FLAG_SPI = 0x0100;
  constexpr uint16_t STATUS_FLAG_SPI_REG_WRITE = 0x0080;
  constexpr uint16_t STATUS_FLAG_SPI_CRC = 0x0040;
  constexpr uint16_t STATUS_DEV_STATE_MASK = 0x0030;
  constexpr uint16_t STATUS_DEV_STATE_NORMAL = 0x0000;
  constexpr uint16_t STATUS_FLAG_OTP_CRC = 0x0008;
  constexpr uint16_t STATUS_FLAG_OSC = 0x0004;
  constexpr uint16_t STATUS_FLAG_POR = 0x0002;
  constexpr uint16_t STATUS_FLAG_ERR = 0x0001;

  // REG_IFST interface error/status flags.
  constexpr uint16_t IFST_FLAG_SPI_CS = 0x0200;
  constexpr uint16_t IFST_FLAG_SPI_TIMEOUT = 0x0100;
  constexpr uint16_t IFST_FLAG_CCSI_CMD_QUEUE_OVF = 0x0010;
  constexpr uint16_t IFST_FLAG_CCSI_CHECK_BIT = 0x0008;
  constexpr uint16_t IFST_FLAG_CCSI_CRC = 0x0002;
  constexpr uint16_t IFST_FLAG_CCSI_SIN = 0x0001;

  // REG_TXFFST transmit FIFO flags and fill level mask.
  constexpr uint16_t TXFFST_FLAG_TXFFOVF = 0x8000;
  constexpr uint16_t TXFFST_FLAG_TXFFUVF = 0x4000;
  constexpr uint16_t TXFFST_FLAG_TXFFSED = 0x2000;
  constexpr uint16_t TXFFST_LEVEL_MASK = 0x01FF;

  // REG_RXFFST receive FIFO flags and fill level mask.
  constexpr uint16_t RXFFST_FLAG_RXFFOVF = 0x8000;
  constexpr uint16_t RXFFST_FLAG_RXFFUVF = 0x4000;
  constexpr uint16_t RXFFST_FLAG_RXFFSED = 0x2000;
  constexpr uint16_t RXFFST_LEVEL_MASK = 0x00FF;

  // Common control values used in init/update paths.
  constexpr uint16_t STATUS_CLR_FLAG = 0x8000;
  constexpr uint16_t STATUS_DRDYST = 0x0400;
  constexpr uint16_t LP5899_DEVID_EXPECTED = 0xED99;
  constexpr uint16_t CCSI_CMD_GAP_US = 2;

  // LP5890 FCx initialization payloads (sent via W_FC0..W_FC4).
  constexpr uint16_t kLp5890Fc0[] = {0x4000, 0x78EF, 0x0100};
  constexpr uint16_t kLp5890Fc1[] = {0x0065, 0xE252, 0x95FF};
  constexpr uint16_t kLp5890Fc2[] = {0x000C, 0xD033, 0x0000};
  constexpr uint16_t kLp5890Fc3[] = {0x007A, 0x4055, 0x8F80};
  constexpr uint16_t kLp5890Fc4[] = {0x0007, 0x003F, 0x0008};

  // LP5899 register initialization image starting at REG_SPICTRL.
  constexpr uint16_t kLp5899Regs[] = {
    0x3001,
    0x0004,
    0x0000,
    0x00FF,
    0x0000,
    0x0000,
  };

  // CRC16-CCITT-FALSE lookup table used by CCSI command and response validation.
  constexpr uint16_t crcCcittTable[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
  };

  void printStatusDecoded(Stream& s, uint16_t status)
  {
    s.println(F("STATUS decode:"));
    s.print(F("  DRDYST=")); s.println((status & STATUS_DRDYST) ? 1 : 0);
    s.print(F("  FLAG_ERR=")); s.println((status & STATUS_FLAG_ERR) ? 1 : 0);
    s.print(F("  FLAG_POR=")); s.println((status & STATUS_FLAG_POR) ? 1 : 0);
    s.print(F("  FLAG_OSC=")); s.println((status & STATUS_FLAG_OSC) ? 1 : 0);
    s.print(F("  FLAG_OTP_CRC=")); s.println((status & STATUS_FLAG_OTP_CRC) ? 1 : 0);
    s.print(F("  FLAG_SPI_CRC=")); s.println((status & STATUS_FLAG_SPI_CRC) ? 1 : 0);
    s.print(F("  FLAG_SPI_REG_WRITE=")); s.println((status & STATUS_FLAG_SPI_REG_WRITE) ? 1 : 0);
    s.print(F("  FLAG_SPI=")); s.println((status & STATUS_FLAG_SPI) ? 1 : 0);
    s.print(F("  FLAG_SRST=")); s.println((status & STATUS_FLAG_SRST) ? 1 : 0);
    s.print(F("  FLAG_RXFF=")); s.println((status & STATUS_FLAG_RXFF) ? 1 : 0);
    s.print(F("  FLAG_TXFF=")); s.println((status & STATUS_FLAG_TXFF) ? 1 : 0);
    s.print(F("  FLAG_CCSI=")); s.println((status & STATUS_FLAG_CCSI) ? 1 : 0);

    const uint8_t devState = static_cast<uint8_t>((status & STATUS_DEV_STATE_MASK) >> 4);
    s.print(F("  DEV_STATE="));
    switch (devState)
    {
      case 0: s.println(F("0 (NORMAL)")); break;
      case 1: s.println(F("1 (INIT)")); break;
      case 2: s.println(F("2 (INIT)")); break;
      default: s.println(F("3 (FAILSAFE)")); break;
    }
  }

  void printIfstDecoded(Stream& s, uint16_t ifst)
  {
    s.print(F("IFST read: 0x"));
    s.println(ifst, HEX);
    s.println(F("IFST decode:"));
    s.print(F("  FLAG_SPI_CS=")); s.println((ifst & IFST_FLAG_SPI_CS) ? 1 : 0);
    s.print(F("  FLAG_SPI_TIMEOUT=")); s.println((ifst & IFST_FLAG_SPI_TIMEOUT) ? 1 : 0);
    s.print(F("  FLAG_CCSI_CMD_QUEUE_OVF=")); s.println((ifst & IFST_FLAG_CCSI_CMD_QUEUE_OVF) ? 1 : 0);
    s.print(F("  FLAG_CCSI_CHECK_BIT=")); s.println((ifst & IFST_FLAG_CCSI_CHECK_BIT) ? 1 : 0);
    s.print(F("  FLAG_CCSI_CRC=")); s.println((ifst & IFST_FLAG_CCSI_CRC) ? 1 : 0);
    s.print(F("  FLAG_CCSI_SIN=")); s.println((ifst & IFST_FLAG_CCSI_SIN) ? 1 : 0);
  }

  void printTxffstDecoded(Stream& s, uint16_t txffst)
  {
    s.print(F("TXFFST read: 0x"));
    s.println(txffst, HEX);
    s.println(F("TXFFST decode:"));
    s.print(F("  FLAG_TXFFOVF=")); s.println((txffst & TXFFST_FLAG_TXFFOVF) ? 1 : 0);
    s.print(F("  FLAG_TXFFUVF=")); s.println((txffst & TXFFST_FLAG_TXFFUVF) ? 1 : 0);
    s.print(F("  FLAG_TXFFSED=")); s.println((txffst & TXFFST_FLAG_TXFFSED) ? 1 : 0);
    s.print(F("  TXFFST_LEVEL=")); s.println(txffst & TXFFST_LEVEL_MASK);
  }

  void printRxffstDecoded(Stream& s, uint16_t rxffst)
  {
    s.print(F("RXFFST read: 0x"));
    s.println(rxffst, HEX);
    s.println(F("RXFFST decode:"));
    s.print(F("  FLAG_RXFFOVF=")); s.println((rxffst & RXFFST_FLAG_RXFFOVF) ? 1 : 0);
    s.print(F("  FLAG_RXFFUVF=")); s.println((rxffst & RXFFST_FLAG_RXFFUVF) ? 1 : 0);
    s.print(F("  FLAG_RXFFSED=")); s.println((rxffst & RXFFST_FLAG_RXFFSED) ? 1 : 0);
    s.print(F("  RXFFST_LEVEL=")); s.println(rxffst & RXFFST_LEVEL_MASK);
  }
}

LP5899_PanelDriver::LP5899_PanelDriver(uint8_t csPin,
                                       uint8_t drdyPin,
                                       uint8_t faultPin,
                                       SPIClass& spi):
  _spi{spi},
  _cs{csPin},
  _drdy{drdyPin},
  _fault{faultPin}
{}

void LP5899_PanelDriver::begin(uint32_t spiHz)
{
  _spiHz = spiHz;

  pinMode(_cs, OUTPUT);
  csHigh_();

  pinMode(_drdy, INPUT);
  pinMode(_fault, INPUT_PULLUP);

  _spi.begin();
  _activeSpiMode = SPI_MODE1;
  _spiAckDisabled = false;
  _initialized = false;
}

bool LP5899_PanelDriver::init()
{
  delay(2);
  // Soft reset device and validate by reading device ID.
  if (!lp5899SoftReset_())
    return false;

  // Validate device ID (locks SPI mode for subsequent transfers).
  uint16_t devid = 0;
  if (!getDeviceId_(devid))
    return false;

  // Wait for NORMAL state before forwarding commands (datasheet programming flow).
  for (uint8_t attempt = 0; attempt < 20; ++attempt)
  {
    uint16_t status = 0;
    if (getStatus_(status) && ((status & STATUS_DEV_STATE_MASK) == STATUS_DEV_STATE_NORMAL))
      break;
    delay(1);
  }

  // Request exit from FAILSAFE in case LP5899 latched there previously.
  const uint16_t exitFs = 0x0001;
  (void)lp5899WriteRegisters_(REG_DEVCTRL, &exitFs, 1);

  delay(2);
  bool regVerifyOk = true;
  for (uint8_t i = 0; i < static_cast<uint8_t>(sizeof(kLp5899Regs) / sizeof(kLp5899Regs[0])); ++i)
  {
    const uint8_t addr = static_cast<uint8_t>(REG_SPICTRL + i);
    if (!lp5899WriteRegisterVerified_(addr, kLp5899Regs[i]))
    {
      regVerifyOk = false;
      Serial.print(F("LP5899 reg verify mismatch addr 0x"));
      Serial.println(addr, HEX);
    }
  }

  // Clear FIFOs once after configuration; W1C bits self-clear.
  (void)lp5899WriteRegisters_(REG_TXFFLVL, &kLp5899Regs[2], 1);
  (void)lp5899WriteRegisters_(REG_RXFFLVL, &kLp5899Regs[3], 1);
  const uint16_t txFifoClear = 0x8000;
  const uint16_t rxFifoClear = 0x8000;
  (void)lp5899WriteRegisters_(REG_TXFFLVL, &txFifoClear, 1);
  (void)lp5899WriteRegisters_(REG_RXFFLVL, &rxFifoClear, 1);

  uint16_t clearFlags = STATUS_CLR_FLAG;
  if (!lp5899WriteRegisterVerified_(REG_STATUS, clearFlags))
  {
    regVerifyOk = false;
    Serial.println(F("LP5899 STATUS clear verify mismatch"));
  }

  // Datasheet note: clear FLAG_POR before forwarding LP589x commands.
  bool porCleared = false;
  for (uint8_t attempt = 0; attempt < 5; ++attempt)
  {
    uint16_t clear = STATUS_CLR_FLAG;
    (void)lp5899WriteRegisters_(REG_STATUS, &clear, 1);

    uint16_t status = 0;
    if (getStatus_(status) && ((status & STATUS_FLAG_POR) == 0))
    {
      porCleared = true;
      break;
    }
    delay(1);
  }
  if (!porCleared)
  {
    Serial.println(F("LP5899 FLAG_POR remains set after CLR_FLAG"));
  }

  if (!regVerifyOk)
  {
    Serial.println(F("LP5899 init continuing with best-effort register writes"));
  }

  // Capture effective ACK mode from hardware in case SPICTRL write did not stick.
  uint16_t spictrlReadback = 0;
  if (lp5899ReadRegisters_(REG_SPICTRL, &spictrlReadback, 1))
  {
    _spiAckDisabled = (spictrlReadback & 0x0001) != 0;
  }

  // Try again to exit FAILSAFE after register programming.
  (void)lp5899WriteRegisters_(REG_DEVCTRL, &exitFs, 1);

  if (!lp5890Init_())
    return false;

  clear();
  if (!writeSRAM_())
    return false;

  if (!lp5890Vsync_())
    return false;

  _initialized = true;
  return true;
}

bool LP5899_PanelDriver::debugReadDeviceId(Stream* dbg)
{
  uint16_t devid = 0xFFFF;
  const bool ok = getDeviceId_(devid);

  if (dbg)
  {
    dbg->print(F("DEVID read: "));
    if (ok)
    {
      dbg->print(F("0x"));
      dbg->println(devid, HEX);
      if (devid != LP5899_DEVID_EXPECTED)
        dbg->println(F("Unexpected LP5899 device ID"));
    }
    else
    {
      dbg->println(F("FAIL"));
    }
  }

  return ok;
}

bool LP5899_PanelDriver::debugReadStatus(Stream* dbg)
{
  uint16_t status = 0xFFFF;
  uint16_t ifst = 0xFFFF;
  uint16_t txffst = 0xFFFF;
  uint16_t rxffst = 0xFFFF;
  const bool ok = getStatus_(status);
  const bool ifstOk = lp5899ReadRegisters_(REG_IFST, &ifst, 1);
  const bool txffstOk = lp5899ReadRegisters_(REG_TXFFST, &txffst, 1);
  const bool rxffstOk = lp5899ReadRegisters_(REG_RXFFST, &rxffst, 1);

  if (dbg)
  {
    dbg->print(F("STATUS read: "));
    if (ok)
    {
      dbg->print(F("0x"));
      dbg->println(status, HEX);
      printStatusDecoded(*dbg, status);
      if (ifstOk)
      {
        printIfstDecoded(*dbg, ifst);
      }
      else
      {
        dbg->println(F("IFST read: FAIL"));
      }

      if (txffstOk)
      {
        printTxffstDecoded(*dbg, txffst);
      }
      else
      {
        dbg->println(F("TXFFST read: FAIL"));
      }

      if (rxffstOk)
      {
        printRxffstDecoded(*dbg, rxffst);
      }
      else
      {
        dbg->println(F("RXFFST read: FAIL"));
      }
    }
    else
    {
      dbg->println(F("FAIL"));
    }
  }

  return ok;
}

bool LP5899_PanelDriver::isFaultActive() const
{
  // LP5899 FAULT is open-drain and active low.
  return digitalRead(_fault) == LOW;
}

bool LP5899_PanelDriver::isReadyToUpdate() const
{
  uint16_t status = 0;
  if (!const_cast<LP5899_PanelDriver*>(this)->getStatus_(status))
    return false;

  return (status & STATUS_DRDYST) != 0;
}

void LP5899_PanelDriver::clear()
{
  _frame = {};
}

void LP5899_PanelDriver::loadFrame(const ScanFrame& frame)
{
  _frame = frame;
}

void LP5899_PanelDriver::csLow_()  { digitalWrite(_cs, LOW); }
void LP5899_PanelDriver::csHigh_() { digitalWrite(_cs, HIGH); }

uint16_t LP5899_PanelDriver::spiTransferWord_(uint16_t word)
{
  const uint8_t txHi = static_cast<uint8_t>((word >> 8) & 0xFF);
  const uint8_t txLo = static_cast<uint8_t>(word & 0xFF);
  const uint8_t rxHi = _spi.transfer(txHi);
  const uint8_t rxLo = _spi.transfer(txLo);
  return static_cast<uint16_t>((static_cast<uint16_t>(rxHi) << 8) | rxLo);
}

void LP5899_PanelDriver::spiTransferWords_(const uint16_t* tx, uint16_t* rx, size_t count)
{
  csLow_();
  for (size_t i = 0; i < count; ++i)
  {
    const uint16_t received = spiTransferWord_(tx[i]);
    if (rx)
      rx[i] = received;
  }
  // Keep CS low briefly after the final clock edge to satisfy LP5899 command framing.
  delayMicroseconds(1);
  csHigh_();
  // Guard time before next command.
  delayMicroseconds(1);
}

bool LP5899_PanelDriver::runTransferWithMode_(uint8_t spiMode,
                                              const uint16_t* tx,
                                              uint16_t* rx,
                                              uint16_t count)
{
  _spi.beginTransaction(SPISettings(_spiHz, MSBFIRST, spiMode));
  spiTransferWords_(tx, rx, count);
  _spi.endTransaction();
  return true;
}

void LP5899_PanelDriver::debugDumpWords_(const __FlashStringHelper* tag,
                                         const uint16_t* words,
                                         uint16_t count)
{
  Serial.print(tag);
  Serial.print(F(" ["));
  Serial.print(count);
  Serial.println(F("]"));

  for (uint16_t i = 0; i < count; ++i)
  {
    Serial.print(F("  "));
    Serial.print(i);
    Serial.print(F(": 0x"));
    if (words[i] < 0x1000) Serial.print('0');
    if (words[i] < 0x100)  Serial.print('0');
    if (words[i] < 0x10)   Serial.print('0');
    Serial.println(words[i], HEX);
  }
}

uint16_t LP5899_PanelDriver::crc16CcittFalse_(const uint16_t* words, size_t count) const
{
  uint32_t crc = 0xFFFF;

  for (size_t i = 0; i < count; ++i)
  {
    const uint16_t word = words[i];

    uint16_t tmp = static_cast<uint16_t>((crc >> 8) ^ ((word >> 8) & 0xFF));
    crc = static_cast<uint16_t>((crc << 8) ^ crcCcittTable[tmp]);

    tmp = static_cast<uint16_t>((crc >> 8) ^ (word & 0xFF));
    crc = static_cast<uint16_t>((crc << 8) ^ crcCcittTable[tmp]);
  }

  return static_cast<uint16_t>(crc);
}

bool LP5899_PanelDriver::lp5899Transfer_(uint16_t command,
                                         const uint16_t* dataWords,
                                         uint16_t dataLength,
                                         uint8_t address,
                                         bool includeData,
                                         bool captureResponse,
                                         bool checkResponseCrc,
                                         uint16_t* responseWords,
                                         uint16_t responseCapacity,
                                         uint16_t* receivedCrc)
{
  // CCSI protocol: assemble command header + optional data, compute CRC, handle response
  uint16_t tx[64] = {0};
  uint16_t rx[64] = {0};
  uint16_t rxMode0[64] = {0};
  uint16_t offset = 0;
  uint16_t crcIndex = 1;
  uint16_t responseLength = 2;
  uint16_t commandWord = 0xFFFF;

  switch (command)
  {
    case FWD_WR_CRC:
      if (dataLength == 0) return false;
      commandWord = static_cast<uint16_t>(command + (dataLength - 1));
      crcIndex = static_cast<uint16_t>(dataLength + 1);
      break;
    case DATA_RD_CRC:
      if (dataLength == 0) return false;
      commandWord = static_cast<uint16_t>(command + (dataLength - 1));
      crcIndex = 1;
      responseLength = static_cast<uint16_t>(dataLength + 1);
      break;
    case REG_WR_CRC:
      if (dataLength == 0) return false;
      commandWord = static_cast<uint16_t>(command + (address << 6) + (dataLength - 1));
      crcIndex = static_cast<uint16_t>(dataLength + 1);
      break;
    case REG_RD_CRC:
      if (dataLength == 0) return false;
      commandWord = static_cast<uint16_t>(command + (address << 6) + (dataLength - 1));
      crcIndex = 1;
      responseLength = static_cast<uint16_t>(dataLength + 1);
      break;
    case SOFTRESET_CRC:
      commandWord = command;
      crcIndex = 1;
      break;
    default:
      return false;
  }

  tx[offset++] = commandWord;

  if (includeData)
  {
    for (uint16_t i = 0; i < dataLength; ++i)
      tx[offset++] = dataWords[i];
  }

  tx[crcIndex] = crc16CcittFalse_(tx, crcIndex);
  offset = static_cast<uint16_t>(crcIndex + 1);

  // TI reference framing: always clock command trailer words.
  // For write commands responseLength stays 2; for read it is dataLength + 1.
  for (uint16_t i = 0; i < responseLength; ++i)
    tx[offset++] = 0xFFFF;

  runTransferWithMode_(kForcedSpiMode, tx, rx, offset);

  if (!captureResponse)
    return true;

  const uint16_t responseStart = static_cast<uint16_t>(crcIndex + 1);
  const uint16_t dataWordsAvailable = static_cast<uint16_t>(responseLength - 1);
  const uint16_t wordsToCopy = (dataWordsAvailable < responseCapacity) ? dataWordsAvailable : responseCapacity;

  if (checkResponseCrc)
  {

    auto parseResponse = [this, responseStart, dataWordsAvailable, wordsToCopy,
                          responseWords, receivedCrc](const uint16_t* frameRx) -> bool
    {
      for (uint16_t shift = 0; shift <= 1; ++shift)
      {
        const uint16_t candidateStart = static_cast<uint16_t>(responseStart + shift);
        const uint16_t reportedCrc = frameRx[candidateStart + dataWordsAvailable];
        const uint16_t calculated = crc16CcittFalse_(&frameRx[candidateStart], dataWordsAvailable);
        if (calculated != reportedCrc)
          continue;

        for (uint16_t i = 0; i < wordsToCopy; ++i)
          responseWords[i] = frameRx[candidateStart + i];

        if (receivedCrc)
          *receivedCrc = reportedCrc;

        return true;
      }

      return false;
    };

    if (parseResponse(rx))
    {
      _activeSpiMode = kForcedSpiMode;
      return true;
    }

    // One additional retry in forced mode to tolerate occasional transient corruption.
    runTransferWithMode_(kForcedSpiMode, tx, rxMode0, offset);
    if (parseResponse(rxMode0))
    {
      _activeSpiMode = kForcedSpiMode;
      return true;
    }

    if (kVerboseLp5899ReadDebug && command == REG_RD_CRC)
    {
      Serial.println(F("LP5899 read CRC failed in forced MODE0"));
      debugDumpWords_(F("TX words"), tx, offset);
      debugDumpWords_(F("RX MODE0 words (try1)"), rx, offset);
      debugDumpWords_(F("RX MODE0 words (try2)"), rxMode0, offset);
    }

    return false;

    // unreachable
    // Some LP5899 variants/boards return read data one word later.
  }

  for (uint16_t i = 0; i < wordsToCopy; ++i)
    responseWords[i] = rx[responseStart + i];

  const uint16_t reportedCrc = rx[responseStart + dataWordsAvailable];
  if (receivedCrc)
    *receivedCrc = reportedCrc;

  return true;
}

bool LP5899_PanelDriver::lp5899WriteRegisters_(uint8_t startAddress, const uint16_t* values, uint16_t count)
{
  return lp5899Transfer_(REG_WR_CRC, values, count, startAddress, true, false, false, nullptr, 0, nullptr);
}

bool LP5899_PanelDriver::lp5899ReadRegisters_(uint8_t startAddress, uint16_t* values, uint16_t count)
{
  return lp5899Transfer_(REG_RD_CRC, nullptr, count, startAddress, false, true, true, values, count, nullptr);
}

bool LP5899_PanelDriver::lp5899WriteRegisterVerified_(uint8_t address, uint16_t value, uint8_t retries)
{
  if (address == REG_STATUS)
  {
    // STATUS is W1C; readback is not expected to match write data.
    return lp5899WriteRegisters_(address, &value, 1);
  }

  uint16_t verifyMask = 0xFFFF;
  if (address == REG_TXFFLVL || address == REG_RXFFLVL)
  {
    // Ignore W1C clear bit during verify.
    verifyMask = 0x7FFF;
  }

  for (uint8_t attempt = 0; attempt < retries; ++attempt)
  {
    if (!lp5899WriteRegisters_(address, &value, 1))
      continue;

    uint16_t readback = 0;
    if (!lp5899ReadRegisters_(address, &readback, 1))
      continue;

    if ((readback & verifyMask) == (value & verifyMask))
      return true;
  }

  return false;
}

bool LP5899_PanelDriver::lp5899SoftReset_()
{
  return lp5899Transfer_(SOFTRESET_CRC, nullptr, 0, 0, false, false, false, nullptr, 0, nullptr);
}

bool LP5899_PanelDriver::ccsiWrite_(uint16_t headWord, const uint16_t* dataWords, uint16_t wordCount)
{
  delayMicroseconds(CCSI_CMD_GAP_US);

  uint16_t payload[20] = {0};
  payload[0] = headWord;

  for (uint16_t i = 0; i < wordCount; ++i)
    payload[i + 1] = dataWords[i];

  return lp5899Transfer_(FWD_WR_CRC, payload, static_cast<uint16_t>(wordCount + 1), 0,
                        true, false, false, nullptr, 0, nullptr);
}

bool LP5899_PanelDriver::lp5890Init_()
{
  if (!ccsiWrite_(W_CHIP_INDEX, nullptr, 0)) return false;
  if (!ccsiWrite_(W_FC0, kLp5890Fc0, 3)) return false;
  if (!ccsiWrite_(W_FC1, kLp5890Fc1, 3)) return false;
  if (!ccsiWrite_(W_FC2, kLp5890Fc2, 3)) return false;
  if (!ccsiWrite_(W_FC3, kLp5890Fc3, 3)) return false;
  return ccsiWrite_(W_FC4, kLp5890Fc4, 3);
}

bool LP5899_PanelDriver::lp5890Vsync_()
{
  return ccsiWrite_(W_VSYNC, nullptr, 0);
}

bool LP5899_PanelDriver::getDeviceId_(uint16_t& devid)
{
  return lp5899ReadRegisters_(REG_DEVID, &devid, 1);
}

bool LP5899_PanelDriver::getStatus_(uint16_t& status)
{
  return lp5899ReadRegisters_(REG_STATUS, &status, 1);
}

bool LP5899_PanelDriver::debugReadRegister(uint8_t address, uint16_t& value)
{
  return lp5899ReadRegisters_(address, &value, 1);
}

bool LP5899_PanelDriver::debugWriteRegister(uint8_t address, uint16_t value)
{
  return lp5899WriteRegisters_(address, &value, 1);
}

void LP5899_PanelDriver::printPins(Stream& s) const
{
  s.print(F("FAULT="));
  s.print(digitalRead(_fault));
  s.print(F(" DRDY="));
  s.print(digitalRead(_drdy));
  s.print(F(" READ_SPI_MODE="));
  s.println((_activeSpiMode == SPI_MODE0) ? F("MODE0") : F("MODE1"));
}

void LP5899_PanelDriver::dumpScanFrame(Stream& s) const
{
  s.println();
  s.println(F("===== SCAN FRAME DUMP ====="));

  for (uint8_t line = 0; line < PANEL_LINE_COUNT; ++line)
  {
    switch (line)
    {
      case 0: s.println(F("Line 0  (bank0: fault=R, relay=G)")); break;
      case 1: s.println(F("Line 1  (bank1: fault=R, relay=G)")); break;
      case 2: s.println(F("Line 2  (bank0: localV on G0..G14)")); break;
      case 3: s.println(F("Line 3  (bank1: localV on G0..G14)")); break;
      case 4: s.println(F("Line 4  (bank0: localI on G0..G14)")); break;
      case 5: s.println(F("Line 5  (bank1: localI on G0..G14)")); break;
      case 6: s.println(F("Line 6  (bank0: remoteI on G0..G14)")); break;
      case 7: s.println(F("Line 7  (bank1: remoteI on G0..G14)")); break;
      default:
        s.print(F("Line "));
        s.println(line);
        break;
    }

    if (line < 2)
    { 
      s.print(F("  R: "));
      for (uint8_t col = 0; col < PANEL_COL_COUNT; ++col)
      {
        s.print(_frame.r[line][col]);
        if (col + 1 != PANEL_COL_COUNT) s.print(' ');
      }
      s.println();
    }

    s.print(F("  G: "));
    for (uint8_t col = 0; col < PANEL_COL_COUNT; ++col)
    {
      s.print(_frame.g[line][col]);
      if (col + 1 != PANEL_COL_COUNT) s.print(' ');
    }
    s.println();
  }

  s.println(F("==========================="));
  s.println();
}

bool LP5899_PanelDriver::debugRawAllOn(uint16_t level)
{
  if (!_initialized && !init())
    return false;

  if (isFaultActive())
    return false;

  for (uint8_t line = 0; line < kLp5890ScanLines; ++line)
  {
    for (uint8_t ch = 0; ch < kLp5890RgbChannels; ++ch)
    {
      const uint16_t sramWords[3] = {level, level, level};
      if (!ccsiWrite_(W_SRAM, sramWords, 3))
        return false;
    }
  }

  return lp5890Vsync_();
}

bool LP5899_PanelDriver::debugRawAllOff()
{
  if (!_initialized && !init())
    return false;

  for (uint8_t line = 0; line < kLp5890ScanLines; ++line)
  {
    for (uint8_t ch = 0; ch < kLp5890RgbChannels; ++ch)
    {
      const uint16_t sramWords[3] = {0, 0, 0};
      if (!ccsiWrite_(W_SRAM, sramWords, 3))
        return false;
    }
  }

  return lp5890Vsync_();
}

bool LP5899_PanelDriver::debugVsyncOnly()
{
  if (!_initialized && !init())
    return false;

  if (isFaultActive())
    return false;

  return lp5890Vsync_();
}

bool LP5899_PanelDriver::debugRawSingle(uint8_t line,
                                        uint8_t channel,
                                        uint8_t color,
                                        uint16_t level)
{
  if (line >= kLp5890ScanLines || channel >= kLp5890RgbChannels || color > 2)
    return false;

  if (!_initialized && !init())
    return false;

  if (isFaultActive())
    return false;

  for (uint8_t l = 0; l < kLp5890ScanLines; ++l)
  {
    for (uint8_t ch = 0; ch < kLp5890RgbChannels; ++ch)
    {
      uint16_t sramWords[3] = {0, 0, 0};
      if (l == line && ch == channel)
      {
        // LP5890 SRAM write order is B, G, R.
        sramWords[color] = level;
      }

      if (!ccsiWrite_(W_SRAM, sramWords, 3))
        return false;
    }
  }

  return lp5890Vsync_();
}

/// Populate LP5890 SRAM from frame buffer.
/// Channel order: B,G,R for each of 16 columns (we only use R,G; B always 0).
bool LP5899_PanelDriver::writeSRAM_()
{
  for (uint8_t line = 0; line < kLp5890ScanLines; ++line)
  {
    for (uint8_t ch = 0; ch < kLp5890RgbChannels; ++ch)
    {
      uint16_t r = 0;
      uint16_t g = 0;
      uint16_t b = 0;

      if (line < PANEL_LINE_COUNT && ch < PANEL_COL_COUNT)
      {
        r = _frame.r[line][ch];
        g = _frame.g[line][ch];
      }

      const uint16_t sramWords[3] = {b, g, r};
      if (!ccsiWrite_(W_SRAM, sramWords, 3))
        return false;
    }
  }

  return true;
}

bool LP5899_PanelDriver::update(bool requireDrdy)
{
  if (!_initialized)
  {
    if (!init())
    {
      Serial.println(F("Cannot update: LP chain init failed"));
      return false;
    }
  }

  if (isFaultActive())
  {
    Serial.println(F("Cannot update: FAULT active"));
    return false;
  }

  if (requireDrdy && !isReadyToUpdate())
  {
    Serial.println(F("Cannot update: DRDY not active"));
    return false;
  }

  if (!writeSRAM_())
  {
    Serial.println(F("Cannot update: SRAM write failed"));
    return false;
  }

  if (!lp5890Vsync_())
  {
    Serial.println(F("Cannot update: VSYNC failed"));
    return false;
  }

  return true;
}