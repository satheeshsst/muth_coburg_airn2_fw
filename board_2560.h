// Hardware layer for the ATmega2560 (classic AVR, "Arduino Mega") board revision.
// Only pin numbers / peripheral setup live here - all application logic
// stays in the main .ino and is shared with board_avr128.h.
#pragma once

// ---- Serial (used for debug prints + the serial "programming mode") ----
#define RS485_serial Serial
#define RS485_BAUD   9600
inline void Board_Serial_Begin() {
  RS485_serial.begin(RS485_BAUD);   // classic hardware USART0, no pin-swap on this core
}

// ---- ADC reference (this chip's internal reference is 2.56V, not 2.5V) ----
#define ADC_REFERENCE INTERNAL2V56

// ---- Buttons ----
#define startStopButton  23 //SW1
#define unitButton       24 //SW2
#define modeButton       25 //SW3
#define enterButton      26 //SW4
#define incrementButton  27 //SW5
#define decrementButton  28 //SW6
#define mode_LMV_HCV     15

// ---- Relays / indicators ----
#define vacummPressureLED 10 //k1
#define vacummLED         11 //k2
#define pressureLED       12 //k3
#define abCylinderLED      9 //k4
#define abCylinderLED1    13 //k5
#define buzzerLED          7 //Buzzer

// ---- Watchdog / mode pins ----
#define WDI          22
#define Prog_mode    19
#define Config_mode  18

// ---- GLCD control pins ----
const int GLCD_LED = 4;
const int GLCD_FS  = 14;
const int GLCD_RD  = 41;

// Backlight drive polarity for the idle dim-down logic in loop(). This board
// never shipped that feature, so there is no verified value yet - confirm on
// real hardware (does GLCD_LED=HIGH turn the backlight on or off?) and correct
// if needed.
#define DIS_TYPE 1

// ---- Pressure sensor input (this board only ever had one physical channel) ----
#define Pressure_in  A8
#define Pressure_in2 A8

// ---- LCD panel wiring: pick exactly one ----
#if defined(LCD_TYPE_CLOWMORE)
  // CLOWMORE panel, currently shipped/verified wiring on this board
  #define LCD_D0 35
  #define LCD_D1 34
  #define LCD_D2 33
  #define LCD_D3 32
  #define LCD_D4 31
  #define LCD_D5 30
  #define LCD_D6 39
  #define LCD_D7 29
  #define LCD_WR 38
  #define LCD_CS 40
  #define LCD_DC 37
  #define LCD_RESET 36
#elif defined(LCD_TYPE_JHD)
  #error "AT2560 + LCD_TYPE_JHD pin mapping has not been measured/verified yet - fill in the real wiring in board_2560.h before building this combination"
#else
  #error "Define exactly one of LCD_TYPE_CLOWMORE or LCD_TYPE_JHD above"
#endif

// ---- Timer: TimerOne library, ticks Timer_tick() at ~100Hz (10ms) ----
// (Matches the AVR128 board's TCA0 tick rate so debounce/on-delay constants
//  and the flash() ~1s vac countdown behave identically on both boards.)
#define TIMER_TICK_HZ 100

void Timer_tick();

inline void Timer_setup() {
  Timer1.initialize(1000000UL / TIMER_TICK_HZ); // microseconds per tick
  Timer1.attachInterrupt(Timer_tick);
}
