# pp_controller_v1
# Power Panel Controller Firmware (Teensy 4.1)

## Overview

Firmware for the SOML casting furnace power panel controller. Runs on a Teensy 4.1 and manages
30 heater channels: relay control, sensor monitoring, temperature acquisition, NTP-synchronized
scheduling, LP5899/LP5890 LED status panel, and Sparkplug B MQTT telemetry.

**Version:** 2.0
**Comms version:** 6  
**Platform:** Teensy 4.1, Arduino framework (PlatformIO)

---

## High-level data flow

```
PCA9698 (I2C)  relay outputs
                 local voltage/current inputs   > PowerPanel > PowerPanelInterface > MQTT (Sparkplug B)
ADG732 + ADC  temperature inputs                      
                                                           > LedPanelMapper > LP5899_PanelDriver > LP5890 > LEDs
```

---

## Hardware

| Peripheral | Interface | Purpose |
|---|---|---|
| PCA9698 x2 (`0x10`, `0x11`) | Wire (i2c0) | Local voltage + local current sense (heaters 029) |
| PCA9698 (`0x12`) | Wire1 (i2c1) | Relay control outputs (heaters 029) |
| PCA9698 (`0x13`) | Wire1 (i2c1) | Remote current sense (heaters 029) |
| ADG732BSUZ | GPIO | 32-channel analog mux for temperature sensors |
| MCP9700  30 | ADC (A11) | Per-heater temperature (500 mV + 10 mV/C) |
| LP5899 + LP5890 | SPI | Status LED panel (8 lines  15 columns) |
| Teensy NIC | Ethernet | MQTT broker connection |

I2C buses run at 400 kHz. SPI to LP5899 runs at 7.5 MHz in normal operation (500 kHz during bring-up).

---

## File tree

```
include/
    ppi_global.hpp           Pin assignments, I2C/SPI addresses, timing constants, version string
    heaterState.hpp          HeaterState struct, SignalValidity enum, HEATER_COUNT (30)
    powerPanel.hpp           PowerPanel class declaration
    LP5899_driver.hpp        LP5899 SPI driver class declaration
    led_mapper.hpp           LedPanelMapper class declaration
    scanFrame.hpp            ScanFrame struct (8 lines  15 cols, R + G channels)
    og_ppi.hpp               PowerPanelInterface (MQTT / Sparkplug B) class declaration
    og_ppi_metrics.hpp       Sparkplug B metric alias enum + metric name table
    og_ntp_thread.hpp        NTP thread public API

src/
    main.cpp                 setup() / loop(), global object instances, timing schedulers
    powerPanel.cpp           PowerPanel: PCA I/O, ADC temp, relay control, NTP scheduler
    LP5899_driver.cpp        LP5899/LP5890 SPI + CCSI framing, CRC, frame upload
    led_mapper.cpp           HeaterState[]  ScanFrame pixel mapping
    og_ppi.cpp               MQTT connect/publish, Sparkplug B encode/decode, error-lamp logic
    og_ntp_thread.cpp        Background NTP sync thread

tests/
    led_panel_test.cpp/.hpp  LED panel unit tests

led_client_gui/
    pp_led_panel_sim.py      LED panel simulator GUI
    ppi_mqtt_monitor.py      MQTT/Sparkplug B monitor tool
```

---

## Module responsibilities

### `ppi_global.hpp`
Central pin and configuration constants:
- Hardware ID jumper pins (JUMPER_PIN_1..5)
- SPI pins: CS (10), DRDY (9), FAULT (8)
- PCA I2C addresses and RST/INT pins
- ADG732 mux pins (A0A4, WR, CS, EN) and ADC pin (A11)
- I2C speed (400 kHz), SPI speeds, ADC resolution (12-bit), Vref (3.3 V)
- Polling intervals: I/O = 1 ms, temperature = 30 ms per channel

---

### `heaterState.hpp`
Per-channel state snapshot used throughout:
- `relayCommand`  commanded relay output
- `localCurrent`, `localVoltage`, `remoteCurrent`  digital sensor readings
- `ioValid`  `SignalValidity::VALID/INVALID` (set INVALID until first successful read)
- `moduleTempC`  temperature in C
- `tempValid`, `ioUpdatedMs`  measurement freshness

---

### `powerPanel.hpp` / `powerPanel.cpp`
Owns all heater hardware and the NTP-synchronized scheduler.

**I/O:**
- Reads 4 PCA9698 expanders via I2C (bank reads, not bit-by-bit)
- `refreshIo()`  reads local and remote current/voltage banks in one call
- `refreshTemp(n)` / `refreshNextTemp()`  round-robin ADC reads via ADG732 mux
- `setRelaysFromMask(uint32_t)`  sets all 30 relay outputs from a bitmask

**Scheduler:**
- `writeNextList(list, size, applyStart, startNtpMs)`  loads a pending schedule
- `schedulerTick(nowNtpMs)`  call every loop; advances through schedule entries using NTP timestamps
- Default cycle period: 250 ms; configurable via `cycle_period`
- `getScheduleSnapshot()`  returns current scheduler state for MQTT publish

---

### `LP5899_driver.hpp` / `LP5899_driver.cpp`
Chip-level driver for TI LP5899 (SPICCSI bridge) + LP5890 (LED current source).
Derived from TI reference code.
- `init()`  configures LP5899 registers, sets up LP5890 via CCSI
- `loadFrame(ScanFrame&)`  stages a full 815 panel frame
- `update(requireDrdy)`  pushes staged frame to hardware (DRDY-gated)
- Internal CRC16-CCITT-FALSE validation on all CCSI responses

---

### `scanFrame.hpp` / `led_mapper.hpp` + `led_mapper.cpp`
Maps heater state to the physical LED panel.

**ScanFrame layout** (8 lines  15 columns, R + G PWM channels):

| Line | R channel | G channel |
|---|---|---|
| 0 | fault, heaters 0-14 | relay, heaters 0-14 |
| 1 | fault, heaters 15-29 | relay, heaters 15-29 |
| 2 |  | localV, heaters 0-14 |
| 3 |  | localV, heaters 15-29 |
| 4 |  | localI, heaters 0-14 |
| 5 |  | localI, heaters 15-29 |
| 6 |  | remoteI, heaters 0-14 |
| 7 |  | remoteI, heaters 15-29 |

Fault (red) = `ioValid == INVALID`, or `relayCommand` on with any sensor reading off, or `relayCommand` off with any sensor reading on.

---

### `og_ppi.hpp` / `og_ppi.cpp`
Sparkplug B MQTT interface. Connects to one of two redundant brokers and publishes all panel metrics.

**Published node metrics (read-only from host):**

| Metric | Type | Description |
|---|---|---|
| FirmwareVersion | string | Full version string |
| PanelStatus | string | `Idle` / `Active` / `I/O Invalid` |
| HeaterOutput | uint32 | 30-bit relay output bitmask |
| VMon / ILMon / IRMon | uint32 | 30-bit voltage / local-I / remote-I bitmasks |
| ErrorLamps | uint32 | 30-bit error indicator bitmask |
| ActiveSchedule / ActiveScheduleStart | dataset / uint64 | Current running schedule |
| NextSchedule / NextScheduleStart | dataset / uint64 | Pending schedule |
| CommandCounter / CommandIndex | uint32 / int32 | Scheduler progress |
| CyclePeriod / CycleOffset | uint32 / int32 | Scheduler timing |
| NTPSynchronized / NTPLastUpdate / NTPServer | bool/uint64/string | NTP health |

**Writable metrics (host  node):**

| Metric | Type | Description |
|---|---|---|
| NextSchedule + NextScheduleStart | dataset + uint64 | Load next heater output schedule |
| VDown / ILDown / IRDown | uint32 | Bitmask of known-bad sensors to ignore in error logic |
| Reboot / Rebirth / NextServer | bool | Control commands |

**Error lamp logic:**
An error bit is set for heater `n` when heater output (HO) disagrees with any *healthy* sensor:
- `HO=0` and any healthy sensor reads ON  error (failed-on)
- `HO=1` and any healthy sensor reads OFF  error (failed-off)

Healthy sensor = sensor not flagged in the `VDown`/`ILDown`/`IRDown` masks.  
When `panel_status == IO_INVALID`, all error lamps are set.

---

### `og_ntp_thread.cpp`
Background thread that periodically syncs with NTP servers (same IP list as MQTT brokers).
Provides `ntp_get_current_time_millis()` used by the scheduler and MQTT timestamps.

---

## Initialization sequence (`setup()`)

1. Serial at 115200 baud
2. Read hardware ID jumpers (sets IP address last octet, MQTT node ID suffix)
3. LP5899 SPI driver init
4. PCA9698 RST/INT pins driven/pulled
5. Wire + Wire1 at 400 kHz, device presence check printed
6. ADG732 mux GPIO init
7. `powerPanel.begin(Wire, Wire1)`  attaches I2C buses
8. All 30 relays set OFF; temperature and I/O primed
9. Initial LED frame uploaded
10. `ppi.attach_panel(&powerPanel)`  `ppi.network_init()`  Ethernet + MQTT configured
11. `ntp_start()`  begins background NTP sync

---

## Main loop scheduling

| Task | Interval | Mechanism |
|---|---|---|
| Heater scheduler tick | every loop | `powerPanel.schedulerTick(ntp_ms)` |
| I/O poll (PCA reads) | 1 ms | `elapsedMicros ioPollUs` |
| Temperature poll (round-robin) | 30 ms | `elapsedMillis tempPollMs` |
| Network/MQTT poll | 100 ms | `elapsedMillis networkPollMs` |
| LED frame upload | 50 ms | `elapsedMillis panelMs` |
| LP5890 VSYNC keepalive | 2 ms | `elapsedMicros vsyncUs` |

All scheduling is non-blocking (no `delay()` in loop).

---

## Network / MQTT configuration

Defined in `og_ppi.hpp`. Broker IPs and panel IP base address differ between
`DEVELOPMENT_SYSTEM` and production builds (set via `build_flags` in `platformio.ini`).

| Setting | Dev | Production |
|---|---|---|
| MQTT brokers | 192.168.1.91 / .92 | 192.168.1.51 / .52 |
| Panel IP base | 192.168.1.150 | 192.168.1.150 |
| Panel IP range | .150.158 | .150.158 (set by jumpers) |
| Sparkplug group | PS | PS |
| Sparkplug node | PPIx (x = jumper ID) | PPIx |

---

## Build environments (`platformio.ini`)

| Environment | Use |
|---|---|
| `teensy41` | Production build |
| `teensy41_debug` | Debug build (`-D DEBUG` enables `Serial.println` trace output) |

**Dependencies:**
- ArduinoJson 6.x
- NativeEthernet (Steward Observatory fork)
- NTPClient_Generic (Steward Observatory fork)
- PubSubClient (Steward Observatory fork)
- so_sparkplugb (Steward Observatory Sparkplug B library)

---

## Hardware ID jumpers

Jumper pins 14 set the panel index (08), which selects the last IP octet offset and the MQTT node ID character.
Jumper pin 5 selects the UART comms channel (1 or 2)  currently unused by the direct I/O backend.

Jumpers are active-low (internal pull-ups; jumper installed = logic 0, read as 1 after inversion).
