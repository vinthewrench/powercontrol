# Power Controller Design Document

## 1. Purpose

The Power Controller is a small, deterministic embedded controller whose sole job is to reliably apply and remove power to a downstream system (typically a Raspberry Pi or similar SBC), while providing:

- Hard power cut and restore via a latching relay
- Manual wake capability
- External wake from an AC-present signal
- Simple visual status via red/green LEDs
- A minimal I2C command interface
- Predictable low-power idle behavior

This board is intentionally boring. No cloud, no OS assumptions, no timing magic. It exists to do one thing well: decide when power flows.

---

## 2. Design Philosophy

- Deterministic: All behavior is explicit and bounded.
- Offline-first: No network dependency.
- Fail-safe bias: Loss of MCU control defaults to relay OFF.
- Serviceable: ISP header, visible LEDs, simple signals.
- Conservative silicon: AVR, ULN2003, latching relay.

---

## 3. Functional Overview

The controller runs continuously from +12 V and selectively switches a separate +12 V switched output feeding the downstream system.

---

## 4. Hardware Architecture

### 4.1 Microcontroller

- ATmega8 / ATmega88 class AVR
- Internal 8 MHz RC oscillator
- No external crystal

### 4.2 Power Regulation

- +12 V input
- Pololu D45V5F5 buck regulator
- +5 V MCU rail

### 4.3 Relay Driver

- Dual-coil latching relay
- ULN2003A driver
- Pulse-only coil drive

### 4.4 LEDs

- Red: fault/timeout
- Green: power ON
- Active-low GPIO drive

### 4.5 Wake Inputs

- INT0: manual wake button
- INT1: AC_OK relay contact

### 4.6 I2C Interface

- Slave address: 0x08
- Watchdog, relay, LED control

---

## 5. Firmware Architecture

- Single main loop
- ISRs set flags only
- IDLE sleep when inactive

### 5.0 Extended I2C Readback

The controller supports an extended read transaction for diagnostics and host verification.

**Command**: `0xF0`  
**Read length**: 4 bytes

**Response format**:

| Byte | Description |
|------|-------------|
| 0 | Firmware version (`0x13`) |
| 1 | Status byte |
| 2 | Timer MSB |
| 3 | Timer LSB |

**Linux example**:

```
i2ctransfer -y 1 w1@0x08 0xF0 r4
```

**Expected response example**:

```
0x13 0x00 0x00 0x00
```

This indicates firmware version 0x13, status = 0, and timer = 0 ms.

This read path is side-effect free and may be called at any time.

---

### 5.1 I2C Command Set

| Command | Function |
|-------|----------|
| 0x00 | NO-OP watchdog ping |
| 0x01 | Relay OFF |
| 0x02 | Relay ON |
| 0x03 | Red LED OFF |
| 0x04 | Red LED ON |
| 0x05 | Green LED OFF |
| 0x06 | Green LED ON |
| 0x10 | Set ping timeout (uint16 ms) |

---

## 6. Safety and Robustness

- Series resistors on ISP and I2C
- No +12 V exposure to MCU pins
- Latching relay prevents thrash

---

## 7. Use Cases

- SBC power watchdog
- Unattended systems
- Farm and off-grid electronics

---

## 8. Non-Goals

- No RTC
- No scheduler
- No network stack

---

## 9. Summary

Power should be deliberate, observable, and recoverable.
