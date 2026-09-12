# Multi-board conversion notes

Working notes for the AVR128 / ATmega2560 board-selection refactor of
`muth_coburg_airn2_fw.ino`.

## 1. How to select a target

Edit the two lines at the top of `muth_coburg_airn2_fw.ino`:

```cpp
#define BOARD_AVR128          // or #define BOARD_AT2560
#define LCD_TYPE_JHD           // or #define LCD_TYPE_CLOWMORE
```

Uncomment exactly one board line and exactly one LCD line, comment out the
other. `board_avr128.h` / `board_2560.h` supply the pin numbers, ADC
reference, serial setup, and timer peripheral for whichever board is active;
everything else (button handling, pressure-relay logic, EEPROM layout,
display drawing) is shared, unchanged code.

## 2. Verified vs. unverified combinations

| Board | LCD | Status |
|---|---|---|
| AVR128 | JHD | Verified - currently shipped wiring |
| AVR128 | CLOWMORE | **Blocked at compile time.** The old CLOWMORE pin numbers collide with this board's relay/button/watchdog pins (see finding below). No safe mapping exists yet. |
| AT2560 | CLOWMORE | Verified - matches the original `COBURG_AIR1_HW1_AT2560_J50.ino` wiring |
| AT2560 | JHD | **Blocked at compile time.** No JHD panel has ever been wired to a 2560 board; pin numbers are unknown. |

Only `BOARD_AVR128`+`LCD_TYPE_JHD` and `BOARD_AT2560`+`LCD_TYPE_CLOWMORE` will
actually compile today. The other two combinations deliberately fail with a
`#error` rather than silently building wrong wiring.

## 3. Fixed this session

1. **AVR128+CLOWMORE pin conflict** - 9 of 12 GLCD pins collided with
   `vacummLED`, `vacummPressureLED`, `Config_mode`, `WDI`, `buzzerLED`,
   `Prog_mode`, `abCylinderLED`, `abCylinderLED1`, `pressureLED`. Turned into
   a compile-time `#error` in `board_avr128.h` instead of offering the wrong
   pins.
2. **`delay()`/`EEPROM.update()` called from inside the timer ISR** -
   `Button_Read()` only ever runs from the hardware timer interrupt on both
   boards (`Timer_tick()` <- `ISR(TCA0_OVF_vect)` on AVR128, or <-
   `TimerOne::attachInterrupt` on the 2560). Its calibration-mode branches
   (increment/decrement correction factors, and the mode-button's
   `config_parameter_mode` cycling) used to call `delay(50)` and
   `EEPROM.update()` directly there - unsafe, since `delay()`/`millis()`
   depend on another interrupt firing that can't happen while nested inside
   this one. Fixed by:
   - Queuing the EEPROM write into `eeprom_calib_pending` /
     `eeprom_calib_address` / `eeprom_calib_value`, applied from `loop()`
     (outside ISR context) instead of from `Button_Read()`.
   - Replacing the increment/decrement `delay(50)` pacing with a
     non-blocking tick counter (`calib_inc_repeat_cnt` /
     `calib_dec_repeat_cnt`, gated by `CALIB_REPEAT_TICKS`).
   - Replacing the mode-button's every-tick `delay(50)` with a
     one-shot-per-press guard (`configParamButton_state`), matching how the
     other buttons in this file already debounce.

## 4. Known open items (not fixed yet - flagged, not touched)

1. **`Vacam_mode_Relay_Logic()` / `Vacam_mode_Relay_off_Logic()` still call
   `delay(100)` from inside the ISR** - reachable via `flash()` (the routine
   vacuum-timer countdown, not just calibration mode) and via the
   manual-mode (`operation_mode==2`) decrement button in `Button_Read()`.
   Same root cause as item 3.2 above, but **skipped deliberately**: the
   100ms gap looks like an intentional stagger between the `abCylinderLED1`
   relay and the `vacummPressureLED`/`vacummLED` relays (avoiding
   simultaneous switching - contact arcing, pressure spikes, solenoid
   stress), so removing or restructuring it needs confirmation on the actual
   hardware intent before changing relay-switching timing. If it does need
   fixing later, the same non-blocking pattern used for the calibration
   branches (flag + counter, applied outside the ISR) should work without
   changing the real-world 100ms gap.
2. **`DIS_TYPE` for the ATmega2560 board (`board_2560.h`) is an unverified
   guess (`1`).** This board never shipped the idle-backlight-dim feature
   before, so nobody has measured whether `GLCD_LED=HIGH` turns that board's
   backlight on or off. Leave as-is (flagged in a comment) until confirmed
   on real hardware; if it's backwards, the display will boot with the
   backlight in the wrong state until the first idle-check corrects it.
3. **The 2560's `TIMER_TICK_HZ=100` (10ms via `TimerOne`) is asserted to
   match the AVR128's raw `TCA0` tick rate**, but that's inferred from the
   pre-existing "100 ticks ≈ 1s" comment in the AVR128 firmware, not
   independently measured. If the AVR128's real tick period differs
   noticeably from 10ms, debounce/on-delay timing and the vacuum-release
   countdown will feel slightly different between the two boards.

## 5. Suggested test plan for the first (AT2560) conversion

1. Compile with `BOARD_AT2560` + `LCD_TYPE_CLOWMORE` for `arduino:avr:mega`
   (should build clean - this was confirmed after fixing the `TimerOne.h`
   include-order issue).
2. Flash a real ATmega2560 unit and check:
   - Display draws correctly (confirms the CLOWMORE pin mapping still holds
     on this hardware).
   - Buttons (start/stop, unit, mode, increment, decrement, enter) all
     respond - this exercises the new non-blocking calibration-mode path
     from item 3.2.
   - Enter calibration/programming mode (`Prog_mode`/`Config_mode` jumpers)
     and hold increment/decrement: the correction factor should still step
     at roughly the old ~50ms-per-step feel, and the value should persist
     in EEPROM after a power cycle (confirms the pending-write handoff to
     `loop()` actually lands).
   - Cycle `config_parameter_mode` via the mode button while in config
     mode: confirm it now advances once per press (not continuously while
     held) - this is a deliberate UX change from item 3.2's fix.
   - Run a full inflate/vacuum cycle and watch the ~1s buzzer/timing feel
     against a known-good AVR128 unit, since item 4.3 above means the two
     boards' absolute timing hasn't been cross-verified on real hardware.
   - Watch backlight behavior around the 120s idle mark (item 4.2) - note
     whether it dims correctly or inverted.
