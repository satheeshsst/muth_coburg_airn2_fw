// Hardware layer for the AVR128 (AVR128DA/DB, DxCore) board revision.
// Only pin numbers / peripheral setup live here - all application logic
// stays in the main .ino and is shared with board_2560.h.
#pragma once

// ---- Serial (used for debug prints + the serial "programming mode") ----
#define RS485_serial Serial1
#define RS485_BAUD   19200
inline void Board_Serial_Begin() {
  RS485_serial.swap(1);          // DxCore pin-swap: RS485 transceiver is on the alternate USART1 pins on this board
  RS485_serial.begin(RS485_BAUD);
}

// ---- ADC reference (AVR-Dx internal reference is 2.5V, not 2.56V) ----
#define ADC_REFERENCE INTERNAL2V5

// ---- HX711 load-cell wiring (unused scaffolding, kept for parity) ----
#define LOADCELL_DOUT_PIN 2
#define LOADCELL_SCK_PIN  3

// ---- Buttons ----
#define startStopButton  20 //SW1
#define unitButton       21 //SW2
#define modeButton       24 //SW3
#define enterButton      25 //SW4
#define incrementButton  26 //SW5
#define decrementButton  27 //SW6
#define mode_LMV_HCV     28

// ---- Relays / indicators ----
#define vacummPressureLED 34 //k1
#define vacummLED         35 //k2
#define pressureLED       36 //k3
#define abCylinderLED     38 //k4
#define abCylinderLED1    37 //k5
#define buzzerLED         39 //Buzzer

// ---- Watchdog / mode pins ----
#define WDI          30
#define Prog_mode    29
#define Config_mode  32

// ---- GLCD control pins ----
const int GLCD_LED = 17;
const int GLCD_FS  = 16;
const int GLCD_RD  = 4;

// Backlight drive polarity for the idle dim-down logic in loop() (verified
// value, carried over from COBURG_AIR1_HW1_AVR128_J50.ino on this same board).
#define DIS_TYPE 2

// ---- Pressure sensor inputs (this board has two real ADC channels) ----
#define Pressure_in  A1
#define Pressure_in2 A0

// ---- LCD panel wiring ----
// Only JHD has ever actually shipped on this board. The old CLOWMORE pin
// values (35,34,33,32,31,30,39,29,38,40,37,36) were copied from a legacy
// commented-out constructor that predates this board's current button/relay
// pin assignments above - they collide with vacummLED(35), vacummPressureLED(34),
// Config_mode(32), WDI(30), buzzerLED(39), Prog_mode(29), abCylinderLED(38),
// abCylinderLED1(37) and pressureLED(36). Do not reuse them here; if a
// CLOWMORE panel is ever wired to an AVR128 board, measure its real pins
// against the CURRENT pin list above and fill them in.
#if defined(LCD_TYPE_JHD)
  #define LCD_D0 8
  #define LCD_D1 9
  #define LCD_D2 10
  #define LCD_D3 11
  #define LCD_D4 12
  #define LCD_D5 13
  #define LCD_D6 14
  #define LCD_D7 15
  #define LCD_WR 1
  #define LCD_CS 5
  #define LCD_DC 6
  #define LCD_RESET 7
#elif defined(LCD_TYPE_CLOWMORE)
  #error "AVR128 + LCD_TYPE_CLOWMORE has no verified, conflict-free pin mapping yet - see comment above. Measure real wiring before using this combination."
#else
  #error "Define exactly one of LCD_TYPE_JHD or LCD_TYPE_CLOWMORE above"
#endif

// ---- Timer: raw TCA0 peripheral, ticks Timer_tick() at ~100Hz (10ms) ----
#define TIMER_TICK_HZ 100

inline void Timer_setup() {
  /* enable overflow interrupt */
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
  /* set Normal mode */
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
  /* disable event counting */
  TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);
  /* set the period */
  TCA0.SINGLE.PER = 0xFF;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1024_gc /* set clock source (sys_clk/1024) */
                       | TCA_SINGLE_ENABLE_bm;      /* start timer */

  sei();
}

void Timer_tick();

ISR(TCA0_OVF_vect) {
  Timer_tick();
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}
