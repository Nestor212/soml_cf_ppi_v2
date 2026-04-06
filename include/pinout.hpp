#pragma once
#include <Arduino.h>


// ============================================================
// PPI Address Pins
// ============================================================
#define JUMPER_PIN_1 37
#define JUMPER_PIN_2 38
#define JUMPER_PIN_3 39
#define JUMPER_PIN_4 40
#define JUMPER_PIN_5 41

// ============================================================
// SPI bus usage (LP5899 LED driver)
// ============================================================
static constexpr uint32_t PANEL_SPI_HZ = 7500000;
static constexpr uint32_t PANEL_SPI_BRINGUP_HZ = 500000;

static constexpr uint8_t PIN_PANEL_CS    = 10;
static constexpr uint8_t PIN_PANEL_DRDY  = 9;
static constexpr uint8_t PIN_PANEL_FAULT = 8;

// ============================================================
// I2C bus usage (PCA9698 wiring)
// ============================================================
// i2c0: Wire  (Teensy 4.1 default SDA/SCL pins you already use)
// i2c1: Wire1 (Teensy 4.1 SDA1/SCL1 pins you already use)
static constexpr uint32_t I2C_HZ = 400000;       // 400 kHz 

static constexpr uint8_t PCA_ADDR_LOCAL_IO_0 = 0x10;  // localI/volt heaters 0..14 0x20
static constexpr uint8_t PCA_RST_LOCAL_IO_0 = 0;
static constexpr uint8_t PCA_INT_LOCAL_IO_0 = 1;  

static constexpr uint8_t PCA_ADDR_LOCAL_IO_1 = 0x11;  // localI/volt heaters 15..29 0x22
static constexpr uint8_t PCA_RST_LOCAL_IO_1 = 14;
static constexpr uint8_t PCA_INT_LOCAL_IO_1 = 15;  

static constexpr uint8_t PCA_ADDR_RELAY_CTRL = 0x12;  // relay control for heaters 0..29 0x24
static constexpr uint8_t PCA_RST_RELAY_CTRL = 2;
static constexpr uint8_t PCA_INT_RELAY_CTRL = 3;

static constexpr uint8_t PCA_ADDR_REMOTE_I   = 0x13;  // remote current for heaters 0..29 0x26
static constexpr uint8_t PCA_RST_REMOTE_I   = 33;
static constexpr uint8_t PCA_INT_REMOTE_I   = 34;

// ============================================================
// ADG732 temperature mux wiring (from your table)
// ============================================================
// ADG732BSUZ MUX -> Teensy ADC input
static constexpr uint8_t PIN_T_ADC    = 25;  // (A11) ADC input (t_data)

// Control
static constexpr uint8_t PIN_T_MUX_EN = 24;  // 
static constexpr uint8_t PIN_T_MUX_WR = 26;   // wr_t_mux: D26
static constexpr uint8_t PIN_T_MUX_CS = 27;   // cs_t_mux: D27

// Address lines
static constexpr uint8_t PIN_T_MUX_A0 = 32;   // a0:D32
static constexpr uint8_t PIN_T_MUX_A1 = 31;   // a1:D31
static constexpr uint8_t PIN_T_MUX_A2 = 30;   // a2:D30
static constexpr uint8_t PIN_T_MUX_A3 = 29;   // a3:D29
static constexpr uint8_t PIN_T_MUX_A4 = 28;   // a4:D28

// ============================================================
// ADC configuration (pick one and stay consistent)
// ============================================================
// If you call analogReadResolution(12), counts will be 0..4095.
// If you use default (often 10-bit), counts will be 0..1023.
static constexpr uint8_t  ADC_RES_BITS   = 12;
static constexpr uint16_t ADC_MAX_COUNTS = (1u << ADC_RES_BITS) - 1u; // 4095
static constexpr float    ADC_VREF       = 3.3f;

// ============================================================
// Timing knobs
// ============================================================
// Mux settle time after channel switch (depends on source impedance)
static constexpr uint16_t TEMP_MUX_SETTLE_US = 10;

// Polling intervals
// I/O is fast (PCA reads), so poll frequently
static constexpr uint32_t IO_POLL_INTERVAL_US = 1000;   // 1 ms

// Temperature is slow (mux + ADC), so poll less often
static constexpr uint32_t TEMP_POLL_INTERVAL_MS = 30; // 30 ms per channel => 900 ms for full panel 