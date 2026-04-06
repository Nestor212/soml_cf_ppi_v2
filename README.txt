# Heater Controller + LP5899/LP5890 LED Panel (Teensy 4.1)

This project implements a structured heater-monitor/control system on a Teensy 4.1 and visualizes heater status on an LP5899→LP5890 LED driver chain.

High-level flow:
Hardware (PCA9698 + ADG732 + ADC) ──> HeaterIO ──> HeaterController ──> HeaterSnapshot
├──> LedMapper ──> LP5899_Driver ──> LP5890_LED_Driver ──> LEDs
└──> MQTT


The design goal is to keep:
- **chip drivers** separate from
- **heater-specific I/O mapping**, separate from
- **system state + control policy**, separate from
- **application presentation logic** (LEDs, MQTT)

---

## File tree (PlatformIO)
include/
    config/
        pinout.h
    model/
        heaters.h
    io/
        heater_io.h
    control/
        heater_controller.h
    app/
        led_mapper.h    
    drivers/
    LP5899_driver.h

src/
    main.cpp
    io/
        heater_io.cpp
    control/
        heater_controller.cpp
    app/
        led_mapper.cpp
    drivers/
        LP5899_driver.cpp

---

## Naming conventions
- “driver” = chip/transport level (PCA9698, LP5899)
- “io” = heater-specific mapping + reads/writes
- “controller” = state + policy + scheduling
- “app” = presentation + serialization (LED mapping, MQTT)

---

## Module responsibilities

### `include/config/pinout.h`
Central location for constants that should never be duplicated:
- Heater count
- PCA9698 I2C addresses
- ADG732 pins (A0..A4, WR, CS, ADC pin)
- ADC parameters (Vref, resolution)
- Timing knobs (settle delays, scan periods)

---

### `include/model/heaters.h`
The **canonical data model** for the system.

Defines:
- `HeaterState` (per-heater state)
- `HeaterSnapshot` (array of 30 heater states)

Typical fields:
- relay command + ctrl source (USER/SYSTEM)
- local voltage / local current / remote current boolean status
- module temperature (and validity/timestamps)

This model is intentionally hardware-agnostic so it can be used by:
- LED display logic
- MQTT publish logic
- debug/telemetry

---

### `include/drivers/*`
Chip-level drivers (no heater knowledge):

- `LP5899_driver.*`
  - Owns a framebuffer and pushes scanline data over SPI.
  - Provides a stable API: set values by logical heater/row, then `update()`.
  - Contains a single “transport seam” to implement the exact LP5899→LP5890 framing.

---

### `include/io/heater_io.h` + `src/io/heater_io.cpp`
The **heater hardware boundary**.

Responsibilities:
- Knows heater-specific mapping tables:
  - local current input bits
  - voltage input bits
  - remote current input bits
  - relay control output bits
- Knows which I2C bus each PCA address uses (Wire vs Wire1)
- Reads power-module temperature via ADG732 mux → Teensy ADC (raw counts)

Does NOT:
- store system state
- implement fault policy
- publish MQTT
- control LED behavior

---

### `include/control/heater_controller.h` + `src/control/heater_controller.cpp`
The **system brain**.

Responsibilities:
- Owns the current `HeaterSnapshot`
- Polls hardware through `HeaterIO` on a controlled cadence (round-robin)
  - e.g., reads 1 heater’s IO per tick
  - reads 1 heater’s temperature per tick (slower)
- Updates validity + timestamps
- Applies relay commands (USER/SYSTEM ownership tracking)

This keeps the main loop non-blocking and prevents temperature reads from stalling LED refresh.

---

### `include/app/led_mapper.h` + `src/app/led_mapper.cpp`
Pure “presentation logic.”

Maps `HeaterSnapshot` → panel pixels:
- One heater = one column
- Red LED indicates fault
- Green LEDs indicate:
  - relay command
  - voltage status
  - local current status
  - remote current status

No hardware access here except calling the panel driver’s API.

---

### `include/app/mqtt_codec.*`
Optional module (you said MQTT code already exists).
Recommended use:
- encode/decode `HeaterSnapshot` to whatever payload format you use
- keep MQTT logic from creeping into the controller/IO layers

---

## How the runtime works

### Initialization (main.cpp)
1. Initialize serial (optional)
2. Bring up I2C (Wire + Wire1)
3. Initialize HeaterIO
4. Initialize HeaterController 
5. Initialize LP5899 driver and panel config
6. Initialize LedMapper

### Main loop (non-blocking)
The loop should do three fast things repeatedly:

1. `heaterCtl.service()`
   - updates 1 heater’s IO at a time and 1 heater’s temp at a slower cadence
   - maintains fresh timestamps + validity

2. `led.apply(heaterCtl.snapshot(), pushFrame=true, requireDrdy=true)`
   - maps the latest snapshot to LED framebuffer
   - pushes to hardware when allowed (DRDY gating)

3. MQTT publish at your chosen rate

---

## LED semantics (current)
- **Green rows**: direct status indicators
  - RELAY row: commanded relay state (on/off)
  - VOLT row: voltage present status bit
  - LOCAL row: local current present status bit
  - REMOTE row: remote current present status bit

- **Red row**: fault indicator
  - Current default policy often includes:
    - IO invalid => fault
    - relay commanded on but missing voltage/current => fault

---

## TODO:

1. **LP5899→LP5890 framing**
   - Implement `LP5899_Driver::writeLp5890Scanline_()` for your exact SPI framing.
   - Everything else (framebuffer, mapping, update cadence) can remain stable.

2. **Temperature conversion**
   - If you want real °C conversion, define your sensor transfer function and either:
     - store raw ADC counts in `HeaterState`, or
     - convert in controller or codec.

3. **Scan cadence**
   - Tune `IO_SCAN_PERIOD_MS` / `TEMP_SCAN_PERIOD_MS` in `pinout.h`.

---

