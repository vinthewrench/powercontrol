/**
 * @file powercontrol.c
 * @brief Power Controller Firmware for ATmega88PB – Manages relay, LEDs, sleep, and I2C commands.
 *
 * This firmware controls a latching power relay, red/green LEDs, and implements low-power sleep/wake
 * behavior. It operates as an I2C slave at address 0x08, accepting commands to control the relay
 * and LEDs, and supports a programmable wake-up timer stored in EEPROM.
 *
 * @note Key features:
 * - Deferred relay pulses (set in main loop, never in ISR)
 * - Independent control of active-low red (PD5) and green (PD4) LEDs
 * - Power-down sleep mode with wakeup via:
 *   - User button (INT0, active-low, PD2)
 *   - AC_OK signal rising edge (INT1, PD3)
 *   - Watchdog-based wake timer (minutes)
 * - Software-based rising-edge detection for AC_OK
 * - Persistent wake timer configuration via EEPROM
 * - Hardware watchdog enabled for system reliability
 *
 * @note Assumptions:
 * - Internal 8 MHz RC oscillator (no external crystal)
 * - LEDs are active-low (ON = pin low)
 * - Button is pulled to GND on PD2 with internal pull-up
 * - AC_OK is HIGH when power supply is active (rising edge triggers wakeup)
 * - Relay is dual-coil latching type (set/reset coils pulsed briefly)
 *
 * @note Limitations:
 * - Multi-byte I2C writes supported only for wake timer (command 0x07 + 2 bytes)
 * - Read operations return single status byte or extended status (register pointer)
 * - No I2C error recovery beyond basic reset
 * - Timer accuracy approximate (~1 min resolution, subject to RC oscillator drift)
 *
 * @note Build & Flash:
 * - Compile: `avr-gcc -mmcu=atmega88pb -Os -o powercontrol.elf powercontrol.c`
 * - Flash: `avrdude -c usbasp -p m8 -U flash:w:powercontrol.elf`
 * - Recommended fuses (internal 8 MHz, BOD 2.7V): `-U lfuse:w:0xE4:m -U hfuse:w:0xD9:m`
 *
 * @note Stack usage: ~150 bytes maximum (estimated; flat structure, shallow ISRs)
 */
/* test
 *
 * # Detect device
 sudo i2cdetect -y 1

 * # Status byte only
 i2cget -y 1 0x08

 * # Extended read example (FW version, status byte, timer MSB, timer LSB)
 * # Expect: 0x13 status_byte timer_MSB timer_LSB (e.g., 0x13 0x00 0x00 0x00 if timer=0 and status=0)
 i2ctransfer -y 1 w1@0x08 0xF0 r4

 # LED tests
 i2cset -y 1 0x08 0x04 # RED ON
 i2cset -y 1 0x08 0x06 # GREEN ON (RED stays ON)
 i2cset -y 1 0x08 0x05 # GREEN OFF (RED stays ON)
 i2cset -y 1 0x08 0x03 # RED OFF
# Relay tests
 i2cset -y 1 0x08 0x01 # Relay OFF
 i2cset -y 1 0x08 0x02 # Relay ON

 * # Timer test (set 1 minute wake-up)
 i2ctransfer -y 1 w3@0x08 0x07 0x00 0x01 # MSB LSB for 1 min

 */
#include <avr/io.h>         // AVR I/O definitions (ports, bits, etc.)
#include <avr/interrupt.h>  // Interrupt handling
#include <util/delay.h>     // Delay functions (_delay_ms)
#include <util/twi.h>       // Two-Wire Interface (I2C) utilities
#include <avr/wdt.h>        // Watchdog timer
#include <avr/eeprom.h>     // EEPROM read/write
#include <avr/sleep.h>      // Sleep modes
#include <stdbool.h>        // Boolean type
#include <stdint.h>         // Standard integer types (uint8_t, etc.)
#include <stddef.h>         // offsetof for struct checksum

/* =========================================================
 * MCU REGISTER COMPATIBILITY (atmega88pb vs ATmega88/88PB)
 * =========================================================
 *
 * ATmega8 uses:
 *   MCUCR, GICR, GIFR, TIMSK, TIFR
 *
 * ATmega88/88PB uses:
 *   EICRA, EIMSK, EIFR, TIMSK1, TIFR1
 *
 * Bit names (ISC01, ISC00, ISC11, ISC10, INT0, INT1, INTF0, INTF1, OCIE1A, OCF1A)
 * are consistent, but the registers holding them differ.
 */
#if defined(__AVR_ATmega8__)
#define EXTINT_CTRL_REG MCUCR
#define EXTINT_MASK_REG GICR
#define EXTINT_FLAG_REG GIFR
#define TIMER1_TIMSK_REG TIMSK
#define TIMER1_TIFR_REG TIFR
#else
#define EXTINT_CTRL_REG EICRA
#define EXTINT_MASK_REG EIMSK
#define EXTINT_FLAG_REG EIFR
#define TIMER1_TIMSK_REG TIMSK1
#define TIMER1_TIFR_REG TIFR1
#endif

/**
 * @brief Forward declarations for functions.
 */
static void i2c_init(void);
static void extint_config_run_edges(void);
static void timer1_init_1s(void);
static inline void timer1_stop(void);

/* ---------------------------------------------------------
 * Clock Configuration
 * --------------------------------------------------------- */
/** @brief CPU frequency (internal RC oscillator). */
#ifndef F_CPU
#define F_CPU 8000000UL
#endif

/** @brief Adjusted Timer1 compare value for closer ~1s ticks (8MHz / 1024 prescaler). */
#define TIMER1_OCR1A 6695

/** @brief Calibrated OSCCAL value for internal oscillator (tune based on measurement; example mid-range). */
#define CALIBRATED_OSCCAL 0x80

/* =========================================================
 * I2C SLAVE ADDRESS
 * ========================================================= */
/** @brief 7-bit I2C slave address for this device. */
#define I2C_ADDR 0x08

/* =========================================================
 * GPIO ASSIGNMENTS (LOCKED)
 * ========================================================= */
/** @brief Green LED pin on PORTD. Active-low (ON when low). */
#define GREEN_LED PD4
/** @brief Red LED pin on PORTD. Active-low (ON when low). */
#define RED_LED PD5
/** @brief Relay set coil pin on PORTD. Pulse to turn relay ON. */
#define RELAY_SET PD6
/** @brief Relay reset coil pin on PORTD. Pulse to turn relay OFF. */
#define RELAY_RESET PD7
/** @brief AC_OK input pin (INT1) on PORTD. High when PSU on. */
#define INT1_PIN PD3 /* AC_OK (INT1) */
/** @brief User button input pin (INT0) on PORTD. Active-low. */
#define INT0_PIN PD2 /* user button (INT0) */

/* =========================================================
 * HARDWARE WATCHDOG
 * ========================================================= */
/** @brief Watchdog timeout period (2 seconds). */
#define HW_WDT_TIMEOUT WDTO_2S

/** @brief Command for extended status read (register start). */
#define CMD_READ_EXT_STATUS 0xF0
/** @brief Firmware version byte (0x13). */
#define FW_VERSION 0x20 /* current version */

/** @brief Command for setting wake-up timer (multi-byte). */
#define CMD_SET_WAKE_TIMER 0x07

/** @brief Max wake timer minutes (1 day) to prevent indefinite sleep. */
#define MAX_WAKE_TIMER_MIN 1440

/* =========================================================
 * EEPROM CONFIG (kept for future use)
 * ========================================================= */
/** @brief EEPROM signature ('P''C'). */
#define EEPROM_SIGNATURE 0x5043 /* 'P''C' */
/** @brief EEPROM config version. */
#define EEPROM_VERSION 0x01

/**
 * @brief EEPROM configuration structure.
 */
typedef struct {
    uint16_t signature; /**< Signature for validity check. */
    uint8_t version; /**< Config version. */
    uint16_t wake_timer_min; /**< Wake-up timer in minutes (persisted). */
    uint16_t checksum; /**< Fletcher-16 checksum over prior bytes. */
} eeprom_cfg_t;

/** @brief EEPROM base address. */
#define EEPROM_BASE_ADDR 0x00

/**
 * @brief Persistent configuration loaded from EEPROM.
 *
 * This structure holds user-configured policy values that are:
 * - Loaded once from EEPROM at boot
 * - Updated explicitly by main-line code (e.g. via I2C commands)
 * - Cached in RAM for all runtime decisions
 *
 * Important:
 * - It is never modified inside an ISR
 * - It does not represent hardware or asynchronous state
 * - Allowing the compiler to optimize access is correct
 *
 * EEPROM is only accessed when explicitly loading or saving this
 * structure; all runtime logic uses this cached copy.
 */
static eeprom_cfg_t cfg;

/**
 * @brief Compute Fletcher-16 checksum.
 * @param data Pointer to data buffer.
 * @param len Length of data.
 * @return Checksum value.
 */
uint16_t config_fletcher16(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    while (len--) {
        sum1 = (sum1 + *p++) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}

/**
 * @brief Compute checksum for config struct (up to but not including checksum field).
 * @param cfg Pointer to config struct.
 * @return Checksum.
 */
static uint16_t cfg_checksum(const eeprom_cfg_t *cfg)
{
    return config_fletcher16(cfg, offsetof(eeprom_cfg_t, checksum));
}

/**
 * @brief Load persistent configuration from EEPROM into global cfg.
 *
 * Reads the EEPROM block into the global cfg structure and validates it.
 *
 * Validation checks:
 * - Signature matches EEPROM_SIGNATURE
 * - Version matches EEPROM_VERSION
 * - Checksum is correct
 *
 * @return 1 if EEPROM contents are valid and cfg is populated,
 *         0 if EEPROM contents are invalid.
 */
static uint8_t eeprom_load_cfg(void)
{
    /* Read entire config block from EEPROM into global cfg */
    eeprom_read_block((void *)&cfg,
                      (const void *)EEPROM_BASE_ADDR,
                      sizeof(cfg));
    /* Validate signature */
    if (cfg.signature != EEPROM_SIGNATURE)
        return 0;
    /* Validate version */
    if (cfg.version != EEPROM_VERSION)
        return 0;
    /* Validate checksum */
    if (cfg.checksum != cfg_checksum(&cfg))
        return 0;
    return 1;
}

/**
 * @brief Initialize global cfg with default values.
 *
 * This is used when EEPROM contents are invalid or uninitialized.
 * The defaults establish a known-good policy state.
 */
static void eeprom_default_cfg(void)
{
    cfg.signature = EEPROM_SIGNATURE;
    cfg.version = EEPROM_VERSION;
    cfg.wake_timer_min = 1; /**< Default: 1 minute wake timer configured */
    /* Compute checksum over initialized fields */
    cfg.checksum = cfg_checksum(&cfg);
}

/**
 * @brief Save global cfg to EEPROM.
 *
 * Updates the checksum and writes the entire configuration block
 * to EEPROM using update semantics to minimize wear.
 *
 * EEPROM is written only when policy changes.
 */
static void eeprom_save_cfg(void)
{
    /* Update checksum before persisting */
    cfg.checksum = cfg_checksum(&cfg);
    /* Write global cfg to EEPROM */
    eeprom_update_block((const void *)&cfg,
                        (void *)EEPROM_BASE_ADDR,
                        sizeof(cfg));
}

/* =========================================================
 * SOFTWARE STATE
 * ========================================================= */
/** @brief I2C register pointer for multi-byte reads. */
static volatile uint8_t i2c_reg_ptr = 0;
/** @brief I2C rx state for multi-byte writes (0 idle, 1 expect MSB, 2 expect LSB). */
static volatile uint8_t rx_state = 0;
/** @brief Temporary buffer for multi-byte I2C data (e.g., timer). */
static volatile uint16_t rx_tmp = 0;
/** @brief Deferred wake timer value (minutes) from I2C. */
static volatile uint16_t deferred_wake_timer = 0;
/** @brief Flag for pending timer update. */
static volatile bool timer_update_pending = false;
/** @brief Remaining wake-up timer in minutes (0 = disabled). */
static volatile uint16_t wake_timer_min = 0;
/** @brief Second counter for timer (decrements every 1s interrupt). */
static volatile uint8_t timer_seconds = 0;
/** @brief Flag for timer expiration wake. */
static volatile bool timer_expired = false;
/** @brief Current relay state (1 = ON). */
static volatile uint8_t relay_state = 0;
/** @brief Current red LED state (1 = ON). */
static volatile uint8_t red_state = 0;
/** @brief Current green LED state (1 = ON). */
static volatile uint8_t green_state = 0;
/** @brief Flag for button press wake event (one-shot). */
static volatile uint8_t wake_flag = 0;
/** @brief Latch for Pi-requested shutdown (1 = requested). */
static volatile uint8_t pi_shutdown_requested = 0;
/** @brief Flag for active shutdown sleep cycle (prevents timer reload on partial wakes). */
static volatile bool shutdown_sleep_active = false;
/** @brief Previous AC_OK state for edge detection. */
static uint8_t ac_ok_prev = 0;

/**
 * @brief Enum for deferred relay requests (to avoid pulses in ISR).
 */
typedef enum {
    RELAY_REQ_NONE = 0, /**< No request pending. */
    RELAY_REQ_ON, /**< Request to turn relay ON. */
    RELAY_REQ_OFF /**< Request to turn relay OFF. */
} relay_req_t;

/** @brief Pending relay request. */
static volatile relay_req_t relay_req = RELAY_REQ_NONE;

/* =========================================================
 * LED HELPERS
 * ========================================================= */
/**
 * @brief Turn red LED on/off.
 * @param on True to turn on (active-low), false to off.
 */
static inline void led_red(bool on)
{
    if (on) { PORTD &= ~(1 << RED_LED); red_state = 1; } // Set pin low for ON
    else { PORTD |= (1 << RED_LED); red_state = 0; } // Set pin high for OFF
}

/**
 * @brief Turn green LED on/off.
 * @param on True to turn on (active-low), false to off.
 */
static inline void led_green(bool on)
{
    if (on) { PORTD &= ~(1 << GREEN_LED); green_state = 1; } // Set pin low for ON
    else { PORTD |= (1 << GREEN_LED); green_state = 0; } // Set pin high for OFF
}

/**
 * @brief Blink an LED in main loop (busy-wait, disables TWI temporarily).
 * @param pin Pin to blink (RED_LED or GREEN_LED).
 * @param count Number of blinks.
 */
static void led_blink(uint8_t pin, uint8_t count)
{
    uint8_t twcr_saved = TWCR; // Save TWI control register
    TWCR &= ~(1 << TWIE); // Disable TWI interrupts to avoid conflicts during blink
    uint8_t red_was = red_state; // Save current states
    uint8_t green_was = green_state;
    for (uint8_t i = 0; i < count; i++) {
        if (pin == RED_LED) led_red(true); // Turn on
        else led_green(true);
        for (uint8_t k = 0; k < 25; k++) { // ~250ms delay (shortened for faster boot)
            _delay_ms(10);
            wdt_reset(); // Reset watchdog during long delays
        }
        if (pin == RED_LED) led_red(false); // Turn off
        else led_green(false);
        for (uint8_t k = 0; k < 25; k++) { // ~250ms delay
            _delay_ms(10);
            wdt_reset();
        }
    }
    TWCR = twcr_saved; // Restore TWI
    led_red(red_was); // Restore original states
    led_green(green_was);
}

/* =========================================================
 * RELAY HELPERS (main only)
 * ========================================================= */
/** @brief Relay coil pulse duration in ms. */
#define RELAY_PULSE_MS 30

/**
 * @brief Pulse a relay coil pin.
 * @param pin Pin to pulse (RELAY_SET or RELAY_RESET).
 */
static inline void relay_pulse(uint8_t pin)
{
    PORTD &= ~((1 << RELAY_SET) | (1 << RELAY_RESET)); // Ensure both coils off
    PORTD |= (1 << pin); // Pulse selected pin
    _delay_ms(RELAY_PULSE_MS); // Pulse duration ~30ms; may vary slightly with clock drift
    PORTD &= ~(1 << pin); // Release
}

/**
 * @brief Apply relay state change.
 * @param on True for ON (set pulse), false for OFF (reset pulse).
 */
static inline void relay_apply(bool on)
{
    if (on) {
        relay_pulse(RELAY_SET); // Pulse set coil
        relay_state = 1; // Update state
    } else {
        relay_pulse(RELAY_RESET);// Pulse reset coil
        relay_state = 0; // Update state
    }
}

/* ===========================
 * POWER_DOWN ADDITIONS
 * =========================== */
#define WDT_TICKS_PER_MIN 30 /* 2s WDT × 30 ≈ 60s */
static volatile uint8_t wdt_ticks = 0;

/* ---------------------------
 * Watchdog interrupt only
 * --------------------------- */
ISR(WDT_vect)
{
    wdt_ticks++;
    if (wdt_ticks >= WDT_TICKS_PER_MIN) {
        wdt_ticks = 0;
        if (wake_timer_min > 0) {
            wake_timer_min--;
            if (wake_timer_min == 0) {
                timer_expired = true;
            }
        }
    }
}

/* ---------------------------
 * Arm watchdog for sleep
 * --------------------------- */
static inline void wdt_arm_interrupt(void)
{
    cli();
    wdt_reset();
    WDTCSR = (1 << WDCE) | (1 << WDE);
    WDTCSR =
        (1 << WDIE) | /* interrupt only */
        (1 << WDP2) | (1 << WDP1) | (1 << WDP0); /* ~2s */
    sei();
}

/* ---------------------------
 * Stop watchdog
 * --------------------------- */
static inline void wdt_stop(void)
{
    cli();
    wdt_reset();
    WDTCSR = (1 << WDCE) | (1 << WDE);
    WDTCSR = 0;
    sei();
}

/* =========================================================
 * SLEEP SUPPORT (POWER_DOWN)
 * ========================================================= */
static void prepare_for_sleep_powerdown(void)
{
    /* HARD stop Timer1 before anything else */
    timer1_stop();
    /* Disable TWI */
    TWCR = 0;
    /* FIX: remove AC_OK pull-up during sleep */
    PORTD &= ~(1 << INT1_PIN);
    /* Disable ADC and analog comparator */
    ADCSRA &= ~(1 << ADEN);
    ACSR |= (1 << ACD);
    /* Shut down peripherals deterministically */
    PRR =
        (1 << PRADC) |
        (1 << PRTWI) |
        (1 << PRTIM0) |
        (1 << PRTIM1) |
        (1 << PRSPI) |
        (1 << PRUSART0);
    /* Clear and enable wake sources */
    EXTINT_FLAG_REG |= (1 << INTF0) | (1 << INTF1);
    EXTINT_MASK_REG |= (1 << INT0) | (1 << INT1);
    /* LEDs physically off */
    PORTD |= (1 << RED_LED) | (1 << GREEN_LED);
    /* Arm watchdog interrupt only */
    wdt_arm_interrupt();
}

static void enter_sleep_powerdown(void)
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleep_enable();
#if defined(sleep_bod_disable)
    sleep_bod_disable(); /* ATmega88PB supports this */
#endif
    sei();
    sleep_cpu();
    sleep_disable();
}

static void restore_after_sleep_powerdown(void)
{
    cli();
    /* Restore peripheral clocks */
    PRR = 0;
    /* FIX: restore AC_OK pull-up for runtime */
    PORTD |= (1 << INT1_PIN);
    /* Restore I2C */
    i2c_init();
    /* Restore external interrupts */
    extint_config_run_edges();
    /* Restore LED outputs */
    led_red(red_state);
    led_green(green_state);
    sei();
}

/* =========================================================
 * EXT INT CONFIG
 * ========================================================= */
/**
 * @brief Configure external interrupts for runtime (edge-triggered).
 */
static void extint_config_run_edges(void)
{
    /* INT0: falling edge (button active-low) */
    EXTINT_CTRL_REG |= (1 << ISC01);
    EXTINT_CTRL_REG &= ~(1 << ISC00);
    /* INT1: rising edge (AC_OK high when on) */
    EXTINT_CTRL_REG |= (1 << ISC11) | (1 << ISC10);
    EXTINT_MASK_REG |= (1 << INT0) | (1 << INT1); // Enable interrupts
    EXTINT_FLAG_REG |= (1 << INTF0) | (1 << INTF1); // Clear pending flags
}

/* =========================================================
 * TIMER1 FOR WAKE-UP (1-second ticks)
 * ========================================================= */
/**
 * @brief Timer1 ISR (every 1 second).
 */
ISR(TIMER1_COMPA_vect)
{
    if (timer_seconds > 0) {
        timer_seconds--;
    } else if (wake_timer_min > 0) {
        wake_timer_min--;
        if (wake_timer_min > 0) {
            timer_seconds = 59; // Only reset if still minutes left
        } else {
            timer_expired = true; // Flag expiration
            timer1_stop(); // Stop timer to avoid further ISRs (safe in ISR)
        }
    }
    // If wake_timer_min == 0 at entry, do nothing (defensive)
}

/* ----------------------------------------------------------
 * TIMER ARM/DISARM HELPERS
 * ---------------------------------------------------------- */
/**
 * @brief Arm wake timer for the NEXT sleep cycle.
 *
 * This must be called ONLY immediately before entering sleep.
 * It does NOT stop Timer1.
 */
static inline void arm_wake_timer(void)
{
    wake_timer_min = cfg.wake_timer_min;
    if (wake_timer_min > 0) {
        timer_seconds = 59;
        timer1_init_1s();
    }
}

/**
 * @brief Stop Timer1 cleanly.
 *
 * Prevents stray compare interrupts and clears pending compare flag.
 */
static inline void timer1_stop(void)
{
    TCCR1B = 0; /* stop clock */
    TIMER1_TIMSK_REG &= ~(1 << OCIE1A); /* disable interrupt */
    TIMER1_TIFR_REG |= (1 << OCF1A); /* clear pending flag */
}

/**
 * @brief Arm runtime countdown from configured policy and start Timer1 if needed.
 *
 * This is called:
 * - before entering sleep (arming a new sleep cycle)
 * - after any wake event (preparing for a future sleep cycle)
 *
 * Uses cfg.wake_timer_min (policy) and loads wake_timer_min (remaining).
 */
static inline void reload_wake_timer(void)
{
    timer1_stop();
    wake_timer_min = cfg.wake_timer_min;
    if (wake_timer_min > 0) {
        timer_seconds = 59;
        timer1_init_1s();
    }
}

/**
 * @brief Initialize Timer1 for 1-second CTC interrupts.
 */
static void timer1_init_1s(void)
{
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= (1 << WGM12); // CTC mode
    OCR1A = TIMER1_OCR1A; // Adjusted for ~1s at 8MHz / prescaler 1024 (adjust if clock drifts)
    TIMER1_TIMSK_REG |= (1 << OCIE1A); // Enable compare A interrupt
    TCCR1B |= (1 << CS10) | (1 << CS12); // Prescaler 1024
}

/**
 * @brief Software debounce for AC_OK rising edge (sample 3 times with ~1ms delay).
 * @param prev Previous state.
 * @return True if confirmed rising edge (stable high).
 */
static inline bool debounce_ac_ok(uint8_t prev, uint8_t now)
{
    return (!prev && now);
}

/* =========================================================
 * I2C SLAVE
 * ========================================================= */
/**
 * @brief Compute status byte for reads.
 * @return Status byte.
 */
static uint8_t get_status_byte(void)
{
    uint8_t s = 0;
    s |= (relay_state ? 1 : 0) << 0; // Bit 0: relay
    s |= (red_state ? 1 : 0) << 1; // Bit 1: red LED
    s |= (green_state ? 1 : 0) << 2; // Bit 2: green LED
    s |= ((PIND & (1 << INT1_PIN)) ? 1 : 0) << 3; // Bit 3: AC_OK
    s |= (wake_timer_min > 0 ? 1 : 0) << 4; // Bit 4: timer active
    return s;
}

/**
 * @brief Read from a register (for multi-byte reads).
 * @param reg Register address.
 * @return Byte value.
 */
static uint8_t read_reg(uint8_t reg)
{
    if (reg > 0xF3) {
        return 0xFF; // Error: beyond valid range
    }
    switch (reg) {
        case 0x00:
            return get_status_byte(); // Default status
        case 0xF0:
            return FW_VERSION; // Firmware version
        case 0xF1:
            return get_status_byte(); // Default status
        case 0xF2:
            return (uint8_t)(cfg.wake_timer_min >> 8); /* config timer MSB */
        case 0xF3:
            return (uint8_t)(cfg.wake_timer_min & 0xFF); /* config timer LSB */
        default:
            return 0xFF; // Error: unknown register
    }
}

/**
 * @brief Initialize I2C as slave.
 */
static void i2c_init(void)
{
    TWAR = (I2C_ADDR << 1); // Set slave address (shifted for AVR, no general call enabled)
    TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE) | (1 << TWINT); // Enable TWI, ACK, interrupt
}

/**
 * @brief TWI (I2C) interrupt service routine.
 */
ISR(TWI_vect)
{
    uint8_t status = TW_STATUS;
    switch (status) {
    case TW_SR_SLA_ACK:
        rx_state = 0; // Reset for new transaction
        rx_tmp = 0;
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE); // Clear flag, ACK next
        break;
    case TW_SR_DATA_ACK: {
        uint8_t b = TWDR; // Read received byte
        if (rx_state == 1) { // MSB for multi-byte
            rx_tmp = ((uint16_t)b << 8);
            rx_state = 2;
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
            break;
        }
        if (rx_state == 2) { // LSB for multi-byte (timer set)
            rx_tmp |= b;
            deferred_wake_timer = (rx_tmp > MAX_WAKE_TIMER_MIN) ? MAX_WAKE_TIMER_MIN : rx_tmp; // Clamp to max
            timer_update_pending = true;
            rx_state = 0;
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
            break;
        }
        /* High bytes: set register pointer for reads or timer command */
        if (b > 0x06) {
            if (b == CMD_SET_WAKE_TIMER) {
                rx_state = 1; // Start multi-byte for timer
                rx_tmp = 0;
            } else {
                i2c_reg_ptr = b; // Read pointer
            }
        } else {
            switch (b) {
            case 0x00:
                break;
            case 0x01:
                relay_req = RELAY_REQ_OFF; // Defer off
                pi_shutdown_requested = 1; // Set latch
                break;
            case 0x02:
                relay_req = RELAY_REQ_ON; // Defer on
                pi_shutdown_requested = 0; // Clear latch
                break;
            case 0x03: led_red(false); break;
            case 0x04: led_red(true); break;
            case 0x05: led_green(false); break;
            case 0x06: led_green(true); break;
            default:
                break;
            }
        }
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE); // ACK next
    } break;
    case TW_ST_SLA_ACK: {
        /* Start of read: send first register byte */
        TWDR = read_reg(i2c_reg_ptr);
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE); // ACK next
        break;
    }
    case TW_ST_DATA_ACK: {
        /* Multi-byte read: increment and send next */
        i2c_reg_ptr++;
        TWDR = read_reg(i2c_reg_ptr);
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE); // ACK next
        break;
    }
    case TW_ST_DATA_NACK:
        /* Read end (no ACK) */
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE); // Ready for next
        break;
    case TW_ST_LAST_DATA:
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE); // Ready for next
        break;
    case TW_SR_STOP:
        rx_state = 0; // Reset on stop
        rx_tmp = 0;
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE); // Ready for next
        break;

    default:
    /* Handle TWI errors: bus error, arbitration lost in slave mode, illegal status, etc. */
        rx_state = 0;               // Reset multi-byte receive state
        rx_tmp = 0;                 // Clear any partial data

        if (status == TW_BUS_ERROR || status == 0xB0) {  // TW_BUS_ERROR = 0x00, 0xB0 = arbitration lost as slave
            // Most serious errors - perform full TWI hardware reset + re-init
            TWCR = 0;                           // Disable TWI completely
            _delay_us(10);                      // Short delay recommended for some AVR families
            i2c_init();                         // Re-initialize slave address, enable, ACK, interrupts
            led_blink(RED_LED, 5);              // Visual feedback: 5 blinks = serious error (shorter than 10)
        }
        else {
            // Minor / recoverable errors (illegal status, START in slave mode, etc.)
            // Just release bus and prepare for next transaction
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
            // Optional: single blink or LED toggle for minor issue
            // led_red(true); _delay_ms(100); led_red(false);  // very subtle feedback
        }
        break;
    }
}

/* =========================================================
 * INT0 and INT1
 * ========================================================= */
/**
 * @brief INT0 ISR (button press).
 */
ISR(INT0_vect)
{
    /* Mask INT0 until main re-arms it */
    EXTINT_MASK_REG &= ~(1 << INT0);
    /* Just latch the event */
    wake_flag = 1;
}

/**
 * @brief INT1 ISR (AC_OK change, wake-only).
 */
ISR(INT1_vect)
{
    /* Policy handled in main (software edge detect). */
}

/* =========================================================
 * INIT
 * ========================================================= */
/**
 * @brief Re-arm INT0 (clear flag and enable).
 */
static inline void rearm_int0(void)
{
    EXTINT_FLAG_REG |= (1 << INTF0); // Clear pending flag
    EXTINT_MASK_REG |= (1 << INT0); // Enable INT0
}

/**
 * @brief Force relay ON on boot/reset.
 */
static void boot_force_relay_on(void)
{
    /*
     * Deterministic power-up behavior:
     * - Never trust the unknown latch state.
     * - Force relay ON exactly once on boot.
     *
     * Preconditions:
     * - DDRD for RELAY_SET/RELAY_RESET already configured as outputs.
     */
    PORTD &= ~((1 << RELAY_SET) | (1 << RELAY_RESET)); /* coils off */
    _delay_ms(2);
    relay_pulse(RELAY_SET); /* ON */
    relay_state = 1;
    /* Clear policy latches */
    pi_shutdown_requested = 0;
    shutdown_sleep_active = false;
    wake_flag = 0;
    timer_expired = false;
}

/**
 * @brief Initialize I/O and peripherals.
 */
static void io_init(void)
{
    /* ======================================================
     * PORTD (LEDs, Relay, INT0, INT1, and unused PD0/PD1)
     * ====================================================== */
    /* Outputs: LEDs + Relay coils + unused PD0/PD1 */
    DDRD |= (1 << GREEN_LED) | (1 << RED_LED)
          | (1 << RELAY_SET) | (1 << RELAY_RESET)
          | (1 << PD0) | (1 << PD1);
    /* Ensure relay coils OFF and unused pins LOW */
    PORTD &= ~((1 << RELAY_SET) | (1 << RELAY_RESET)
             | (1 << PD0) | (1 << PD1));
    /* Inputs: only the used ones (INT0 button + INT1 AC_OK) */
    /* (PD2 and PD3 are already inputs by default after reset) */
    /* Pull-ups on used inputs */
    PORTD |= (1 << INT0_PIN); /* button: active-low, needs pull-up */
    PORTD |= (1 << INT1_PIN); /* AC_OK: prevent float, even if externally pulled */
    /* LEDs OFF (active-low = high on pin) */
    PORTD |= (1 << RED_LED);
    PORTD |= (1 << GREEN_LED);
    /* ======================================================
     * PORTB (unused PB0/PB1/PB2 + XTAL pins PB6/PB7)
     * ====================================================== */
    /* Set all unused PB pins as outputs driven low */
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2)
           | (1 << PB6) | (1 << PB7);
    PORTB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2)
             | (1 << PB6) | (1 << PB7));
    /* ======================================================
     * PORTC (ADC pins PC0–PC5; PC4/PC5 also SDA/SCL but apparently not used here?)
     * ====================================================== */
    /* Set unused ADC pins as outputs driven low (PC0–PC3 as requested) */
    DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3);
    PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3));
    /* Keep digital input buffers disabled on all ADC pins (good practice) */
    DIDR0 = 0x3F; /* ADC0–ADC5 = PC0–PC5 */
    /* Disable analog comparator input buffers */
    DIDR1 |= (1 << AIN0D) | (1 << AIN1D);
    /* ======================================================
     * Peripheral shutdown (static) — already excellent
     * ====================================================== */
    ADCSRA &= ~(1 << ADEN); /* ADC off */
    ACSR |= (1 << ACD); /* Analog comparator off */
    /* Configure external interrupts (leave as-is) */
    extint_config_run_edges();
}

/**
 * @brief Main firmware entry point.
 *
 * Responsibilities:
 * - Initialize hardware and load persistent configuration
 * - Maintain main event loop
 * - Enforce sleep / wake policy
 * - Apply I2C commands and wake timer behavior
 */
int main(void)
{
    /* Disable watchdog during early init */
    wdt_disable();
    cli(); /* No interrupts during hardware init */
    /* OSCCAL is a fixed, calibrated value.
     * Must be set before any timing-sensitive init and never changed at runtime.
     */
    OSCCAL = CALIBRATED_OSCCAL;
    io_init(); /* GPIO, LEDs, relay pins, ext interrupts */
    /* Force relay ON on power-up/reset */
    boot_force_relay_on();
    i2c_init(); /* I2C slave setup */
    sei(); /* Interrupts enabled */
    /* ------------------------------------------------------
     * Load persistent configuration (policy)
     * ------------------------------------------------------ */
    if (eeprom_load_cfg()) {
        led_blink(GREEN_LED, 5); /* EEPROM valid */
    } else {
        eeprom_default_cfg(); /* EEPROM invalid or blank */
        eeprom_save_cfg();
        led_blink(RED_LED, 5);
    }
    /* Establish AC_OK baseline for edge detection */
    ac_ok_prev = (PIND & (1 << INT1_PIN)) ? 1 : 0;
    /* Enable watchdog after system is stable */
    wdt_enable(HW_WDT_TIMEOUT);
    /* ======================================================
     * Main loop
     * ====================================================== */
    for (;;) {
        wdt_reset();
        /* ----------------------------------------------
         * Apply deferred relay requests from I2C ISR
         * ---------------------------------------------- */
        {
            relay_req_t r = RELAY_REQ_NONE;
            uint8_t s = SREG;
            cli();
            r = relay_req;
            relay_req = RELAY_REQ_NONE;
            SREG = s;
            if (r == RELAY_REQ_ON) relay_apply(true);
            if (r == RELAY_REQ_OFF) relay_apply(false);
        }
        /* ----------------------------------------------
         * Handle deferred wake timer update from I2C
         * ---------------------------------------------- */
        if (timer_update_pending) {
            uint8_t s = SREG;
            cli();
            cfg.wake_timer_min = deferred_wake_timer; /* update policy */
            timer_update_pending = false;
            SREG = s;
            eeprom_save_cfg(); /* persist policy */
            /* DO NOT arm timer here, only arm immediately before sleep */
        }
        /* ----------------------------------------------
         * Sleep policy
         * ----------------------------------------------
         * Enter sleep only when:
         * - Pi has requested shutdown
         * - Relay is OFF
         * ---------------------------------------------- */
        if (pi_shutdown_requested && !relay_state) {
            wdt_stop();
            rearm_int0();
            if (!shutdown_sleep_active) {
                reload_wake_timer();
                wdt_ticks = 0;
                shutdown_sleep_active = true;
            }
            prepare_for_sleep_powerdown();
            enter_sleep_powerdown();
            bool full_wake = false;
            bool button_wake = (wake_flag != 0);
            if (button_wake) {
                wake_flag = 0;
                /* Post-wake debounce (safe here, not in ISR) */
                _delay_ms(10);
                if (!(PIND & (1 << INT0_PIN))) {
                    full_wake = true;
                }
            }
            if (timer_expired) {
                timer_expired = 0;
                full_wake = true;
            }
            /* Fix: pass both prev and now to debounce/edge detect */
            {
                uint8_t ac_ok_now = (PIND & (1 << INT1_PIN)) ? 1 : 0;
                if (debounce_ac_ok(ac_ok_prev, ac_ok_now)) {
                    full_wake = true;
                }
                ac_ok_prev = ac_ok_now;
            }
            if (full_wake) {
                wdt_stop();
                restore_after_sleep_powerdown();
                relay_apply(true);
                pi_shutdown_requested = 0;
                shutdown_sleep_active = false;
                wdt_enable(HW_WDT_TIMEOUT);
                if (button_wake) {
                    rearm_int0();
                }
            }
        }
        /* ----------------------------------------------
         * AC_OK rising-edge wake (software edge detect)
         * ---------------------------------------------- */
        {
            uint8_t ac_ok_now = (PIND & (1 << INT1_PIN)) ? 1 : 0;
            if (debounce_ac_ok(ac_ok_prev, ac_ok_now)) {
                if (pi_shutdown_requested) {
                    timer1_stop();
                    relay_apply(true);
                    pi_shutdown_requested = 0;
                    shutdown_sleep_active = false;
                }
            }
            ac_ok_prev = ac_ok_now;
        }
    }
}
