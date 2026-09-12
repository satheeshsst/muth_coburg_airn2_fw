#include "U8g2lib.h"
#include <stdlib.h>
#include <EEPROM.h>

#define Debugmode 1

// ---- Hardware configuration: define exactly one board and one LCD type ----
// #define BOARD_AVR128          // or
#define BOARD_AT2560
#define LCD_TYPE_CLOWMORE     // or #define LCD_TYPE_JHD (JHD is NOT verified on this board)

#if defined(BOARD_AVR128)
  #include "board_avr128.h"
#elif defined(BOARD_AT2560)
  #include <TimerOne.h>
  #include "board_2560.h"
#else
  #error "Define exactly one of BOARD_AVR128 or BOARD_AT2560 above"
#endif

U8G2_T6963_240X128_F_8080 u8g2(U8G2_R0, LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7,
                                /*enable/wr=*/ LCD_WR, /*cs/ce=*/ LCD_CS, /*dc=*/ LCD_DC, /*reset=*/ LCD_RESET); // Connect RD with +5V, FS0 and FS1 with GND

long offset = -9040.0;

int Display_offcnt = 0; // idle counter (~seconds since last active pressure reading) driving the backlight-off logic in loop()

int priousdissetpoint = 0;
int priousdispressure = 0;
int sensorereaddelay = 60;

unsigned long run_time = 0;
unsigned long run_time_c = 0;
unsigned long run_time_p = 0;
unsigned int j = 6;

unsigned int fuelcalib(float x11,float y11,float z11,float x22,float y22);
#define VOL  54

void hx711_setup();
void hx711_read();

void flash();
void N2Purity();
void buzzer_logic(boolean buzzer_bit );
void end_alram( );
void Button_Read();
void button_buzzer_logic();
void PinModeConfig();
void TyreCount();
void wdi_time_reset();
void Pressure_relay();
void get_ondelay_time () ;
void Pressure_relay_state() ;
void Pressure_relay_st (int state) ;
void Vacam_mode_Relay_Logic(int state);
void Vacam_mode_Relay_off_Logic(int state);
void Vacam_Relay_Logic();
void Timer1_fun();  
void Write_int_eeprom(unsigned int data,unsigned int add);
void writeString(char add,String data);
void Display_referesh();
int sensorRead();
void PID_calculation();
void print_data();
void gain_config ();
void PID_clear();
//
int offdelaytime = 6000;

int sensorRead1 = 0;

//buzzer additional//
int buzzer_stop_count = 0;
int set_bit_buzzer_reset = 0;
int temp_buzzer_reset_bit = 1;
unsigned long pressureSensorRaw = 0;
// Input Variables Go here 


// EEPROM ADDRES
const int TyreCount_address = 8;    
const int setPoint_address = 4; 
const int Caliberation_address = 1;    
unsigned int vac_time_address = 12;
unsigned int pressure_time_address = 14;

unsigned int after_prog_1 = 20;
// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long previousMillis_Dis = 0;
// constants won't change:
const long interval = 60000;           //60000 interval at which to blink (milliseconds)
unsigned long currentMillis_abs = 0;

int count = 0;
unsigned int unit = 1;
unsigned int vacuum = 0;
unsigned long pressureSensorValue = 0;
float pressureBarValue = 0;
float pressureKgfValue = 0;
float setPointBar = 0;
float setPointKgf = 0;
float correctionfactor = 190.0;
unsigned int pressureMIN = 0;
unsigned int pressureMAX = 255;
unsigned int pressureSensor[10];
uint8_t setPoint = 25;
uint8_t setTimer = 25;
uint8_t setTimerOld = 25;
uint8_t setBitTimer = 0;
uint8_t endTimer = 0;
uint8_t vac_timer_bit = 0;
unsigned int Pressure_coil_bit = 0;
unsigned int  vacum_coil_bit = 0;
unsigned int Pressure_SetBit_cnt = 0;
unsigned int Pressure_SetBit = 0;
unsigned int abCylinderLED_SetBit = 0;
unsigned int on_delay = 0;
unsigned int on_delay_cnt = 250; // 300 if inser the mouth take firstime delay 

/*    button logic start   */
unsigned int incrementButton_1 = 0;
unsigned int decrementButton_1 = 0;
unsigned int startStopButton_1 = 0;
unsigned int modeButton_1 = 0;
unsigned int enterButton_1 = 0;
unsigned int unitButton_1 = 0;
unsigned int operation_mode = 0;
unsigned int unit_mode =0;
unsigned int enter_mode =0;
boolean mode_LMV_HCV_1 = 1;
boolean mode_LMV_HCV_1_setbit = 0;


boolean startStopButton_state = false;
boolean startStop_mode = false; //STOP 
boolean startStop_mode_0 = false; //STOP 

boolean unitButton_state = false;   
boolean  modeButton_state = false;
unsigned int debounce_time = 5;
unsigned int debounce_inc_time = 2;
unsigned int debounce_dec_time = 2;
unsigned int temp_cnt = 0 ;
unsigned int temp_cnt_dec = 0 ;
boolean button_release_dec = 1;
boolean button_release_inc = 1;
boolean startStop_mode_bit = 0;
boolean Prog_mode_State = 1;
boolean config_mode_state = 1;

float correctionfactor_pressure_time = 100;
float correctionfactor_vac_time = 100;
unsigned int config_parameter_mode = 0;
boolean configParamButton_state = 0;

/* Button_Read() runs only from inside the timer ISR (see Timer_tick()).
   The calibration branches below used to call delay(50)/EEPROM.update()
   directly from that ISR context, which is unsafe (delay()/millis() need
   another interrupt to advance, and that can't fire while nested inside
   this one). Instead they just update the in-memory value here and hand
   the EEPROM write off to loop(), which runs outside the ISR. */
unsigned int calib_inc_repeat_cnt = 0;
unsigned int calib_dec_repeat_cnt = 0;
#define CALIB_REPEAT_TICKS 5   // ~50ms at the shared 100Hz tick, matches the old delay(50) pacing
boolean eeprom_calib_pending = 0;
unsigned int eeprom_calib_address = 0;
byte eeprom_calib_value = 0;


boolean incrementButton_bz =0 ;
boolean decrementButton_bz =0 ;
boolean modeButton_bz = 0 ;  
boolean unitButton_bz = 0; 
boolean enterButton_bz = 0;  
boolean startStopButton_bz = 0;

/*    button logic  end  */

//    PID Logic
unsigned long On_Time_Presure = 0; //ms
unsigned long Off_Time_Presure = 0;
unsigned long currentMillis ;
unsigned long On_Time_Presure_previousMillis;
unsigned long Off_Time_Presure_previousMillis = 0;
float error_percentage = 0.0;
boolean On_Off_time_setbit = false; //STOP
unsigned int Pressure_relay_logic = 0;
boolean Vacam_bit_state = 1;
boolean Vacam_fast_bit_state = 1;
boolean error_percentage_bit = 0;
boolean ABC_bit = 0;
boolean alarm_bit = 0;
boolean Time_state = 0;
boolean Vac_mode_relay = 0;
boolean Vac_mode_pressure = 0;
unsigned int loop_cnt = 0;
unsigned int pid_error = 0;
unsigned int no_inlet_value_on = 0;
unsigned int Tyre_completed = 0;
unsigned int abCylinderLED1_State;
unsigned int vacummLED_State;
unsigned int vacummPressureLED_State;
unsigned int AB_cylinder = 0;
unsigned int button_buzer_time = 0;

unsigned int END_alram_set = 0;

unsigned long  PresureT1 = 0;
unsigned long  PresureT2 = 0;
unsigned long  PresureT3 = 0;
unsigned long  PresureT4 = 0;
unsigned long  PresureT5 = 0;
unsigned long  PresureT6 = 0;

unsigned long  VacT1 = 0;
unsigned long  VacT2 = 0;
unsigned long  VacT3 = 0;
unsigned long  VacT4 = 0;
unsigned long  VacT5 = 0;
unsigned long  VacT6 = 0;

float Prev_pressure = 0;
float After_pressure = 0;
float defer_pressure = 0;

 long Prev_time = 0;
 long After_time = 0;
 long defer_time = 0;

float Eestimated_gain = 0.0 ;
float gain_output = 0.0;
float Actul_gain = 0.0 ;
float default_gain = 100.0;
float constan_bike = 1000.0;
int first_inc = 0;
int Error_slot = 0;
int Extragainreq =0;
int NUmperofstok =0;
int NUmperofstok_5 =0;
int NUmperofstok_1 = 0;
int NUmperofstok_2 = 0;
int NUmperofstok_3 = 0;
int NUmperofstok_4 = 0;
int NUmperofstok_6 =0;

boolean Display_referesh_bit = 0;
String recivedData;
String recivedData1;
String recivedData2;
// TyreCount
  unsigned int TyreCount_bit = 0;
  unsigned int TyreCount_value = 0;
  unsigned int TyreCount_Total = 0; 
  unsigned int Tyre_connect = 0;
  unsigned int Tyre_connect_vac = 0;

unsigned long vac_timer_cnt =0;
unsigned long vac_timer_state =0;
unsigned long vac_timer_SS =0;


int tyire_mode =1;
int vacumRelease_logic = 1;

void drawTexture(void)
{
  unsigned int i = 0;
  j = 6;
   u8g2.clearBuffer();
  //  unsigned int no_cir = 5; 
    if ( operation_mode == 0){
    u8g2.setFont(u8g2_font_victoriabold8_8u   );
    u8g2.drawStr(0,13,"STD");
    }
    if (mode_LMV_HCV_1 == 0){
   // if ( (setPoint >= 100 || operation_mode == 3) && operation_mode != 4 && operation_mode != 2 && operation_mode != 1 && config_mode_state == 0){
    u8g2.setFont(u8g2_font_victoriabold8_8u   );
  if (tyire_mode == 1)  {u8g2.drawStr(30,13,"LMV/AIR");};
  if (tyire_mode == 2)  {u8g2.drawStr(30,13,"HCV/AIR");};
    } 
   // if ( (setPoint <= 100 || operation_mode == 4) && operation_mode != 3 && operation_mode != 2 && operation_mode != 1 && config_mode_state == 0){
   if (mode_LMV_HCV_1 == 1){
      u8g2.setFont(u8g2_font_victoriabold8_8u   );
      if (tyire_mode == 1)  {u8g2.drawStr(30,13,"LMV/N2");}
      if (tyire_mode == 2)  {u8g2.drawStr(30,13,"HCV/N2");}
    }
    
    if ( operation_mode == 2){
    u8g2.setFont(u8g2_font_victoriabold8_8u   );
    u8g2.drawStr(70,13,"MAN");
    }
    
    if ( Pressure_coil_bit == 1){
    u8g2.setFont(u8g2_font_victoriabold8_8u   );
    u8g2.drawStr(100,13,"INC");
     j = 26;
    }
    else if ( vacum_coil_bit == 1 && Pressure_relay_logic == 2 ){
       u8g2.setFont(u8g2_font_victoriabold8_8u  );
       u8g2.drawStr(100,13,"VAC");
       j = 20;
      
      }
    else{
    u8g2.setFont(u8g2_font_victoriabold8_8u   );
    u8g2.drawStr(180,13,"MEA");
     j = 6;
    }

 if (pid_error > 30 && vac_timer_bit == 0){  
   u8g2.setFont(u8g2_font_courB24_tf  );
   u8g2.drawStr(5,70,"ERR ");
  }
  else if (TyreCount_bit == 1  && vac_timer_bit == 0) {
   u8g2.setFont(u8g2_font_courB24_tf  );
   u8g2.drawStr(5,70,"END ");
    }
 else {
      
   for(i = 5; i < j; i += 5){
    wdi_time_reset(); 
    u8g2.drawCircle(45, 48, i-1, U8G2_DRAW_UPPER_LEFT);
    u8g2.drawCircle(48, 48, i-1, U8G2_DRAW_UPPER_RIGHT);
    u8g2.drawCircle(45, 51, i-1, U8G2_DRAW_LOWER_LEFT);
    u8g2.drawCircle(48, 51, i-1, U8G2_DRAW_LOWER_RIGHT);
         }
    for(i = 5; i < j; i += 5){
    wdi_time_reset();   
    u8g2.drawCircle(45, 48, i, U8G2_DRAW_UPPER_LEFT);
    u8g2.drawCircle(48, 48, i, U8G2_DRAW_UPPER_RIGHT);
    u8g2.drawCircle(45, 51, i, U8G2_DRAW_LOWER_LEFT);
    u8g2.drawCircle(48, 51, i, U8G2_DRAW_LOWER_RIGHT);
       }
 }
 //   U8G2_DRAW_UPPER_RIGHT, U8G2_DRAW_UPPER_LEFT, U8G2_DRAW_LOWER_LEFT, U8G2_DRAW_LOWER_RIGHT
//    u8g2.setFont(u8g2_font_victoriabold8_8u  );
//    u8g2.drawStr(20,85,"N");
//    u8g2.setFont(u8g_font_4x6);
//    u8g2.drawStr(27,85,"2");
//    u8g2.setFont(u8g2_font_victoriabold8_8u  );
//    u8g2.drawStr(35,85,"Purity");
 
 if (enter_mode == 1 && operation_mode == 1){
    u8g2.setFont(u8g2_font_victoriabold8_8u  );
    u8g2.drawStr(48,98,"VAC:");
    }
    
  //  u8g2.setFont(u8g2_font_victoriabold8_8u  );
  //  u8g2.drawStr(25,98,"99.2%");

    if ( unit_mode == 1){
    u8g2.setFont(u8g2_font_victoriabold8_8u   );
    u8g2.drawStr(217,70,"BAR");
    u8g2.setFont(u8g2_font_fub42_tf );
  
    /*RS485_serial.print("Pressure Value in Texture():");
    RS485_serial.println(pressureBarValue);*/
    
    if (pressureBarValue < 10){
     u8g2.setCursor(110, 75); 
     u8g2.print(pressureBarValue,1);
    }
    else if ((pressureBarValue >= 10) && (pressureBarValue < 100)){
     u8g2.setCursor(85, 75); 
     u8g2.print(pressureBarValue,1);
    }
    else{
     u8g2.setCursor(65, 75); 
     u8g2.print(pressureBarValue,1);
    }

   //  u8g2.setFont(u8g2_font_fub42_tf  );
   //  u8g2.drawStr(152,75,".");
     
    u8g2.setFont(u8g2_font_fub14_tf  );
    if (setPointBar < 10){
     u8g2.setCursor(185, 120); 
     u8g2.print(setPointBar);
    }
    else if ((setPointBar >= 10) && (setPointBar < 100)){
     u8g2.setCursor(185, 120); 
     u8g2.print(setPointBar);
    }
    else{
     u8g2.setCursor(185, 120); 
     u8g2.print(setPointBar);
    }
    }
    if ( unit_mode == 0){
    u8g2.setFont(u8g2_font_victoriabold8_8u   );
    u8g2.drawStr(217,70,"PSI");

    u8g2.setFont(u8g2_font_fub42_tf); //u8g_font_osb35 u8g2_font_fub35_tf 
  
    /*RS485_serial.print("Pressure Value in Texture():");
    RS485_serial.println(pressureSensorValue);*/
    float Displa_temp = (float)pressureSensorValue;
    if (pressureSensorValue < 10){
     u8g2.setCursor(132, 75); //125
     u8g2.print(Displa_temp,1);
    }
    else if ((pressureSensorValue >= 10) && (pressureSensorValue < 100)){
     u8g2.setCursor(98, 75); //90
     u8g2.print(Displa_temp,1);
    }
    else{
     u8g2.setCursor(68, 75); //65
     u8g2.print(Displa_temp,1);
    }

   /*  u8g2.setFont(u8g2_font_fub42_tf);
     u8g2.drawStr(155,75,".");
     u8g2.setFont(u8g2_font_fub42_tf ); //u8g_font_osb35
     u8g2.drawStr(164,75,"0");
  */
     u8g2.setFont(u8g2_font_fub14_tf  );
    if (setPoint < 10){
     u8g2.setCursor(185, 120); 
     u8g2.print(setPoint);
    }
    else if ((setPoint >= 10) && (setPoint < 100)){
     u8g2.setCursor(185, 120); 
     u8g2.print(setPoint);
    }
    else{
     u8g2.setCursor(185, 120); 
     u8g2.print(setPoint);
    }
//    u8g2.setFont(u8g2_font_victoriabold8_8u  );
//    u8g2.drawStr(210,120,".0");
    }
   
    if ( unit_mode == 2){
    u8g2.setFont(u8g2_font_victoriabold8_8u   );
    u8g2.drawStr(217,70,"KGF");

    u8g2.setFont(u8g2_font_fub42_tf); //u8g2_font_fub35_tf
  
    /* RS485_serial.print("Pressure Value in Texture():");
    RS485_serial.println(pressureKgfValue); 
    */
    
    if (pressureKgfValue < 10){
     u8g2.setCursor(110, 75); 
     u8g2.print(pressureKgfValue,1);
    }
    else if ((pressureKgfValue >= 10) && (pressureKgfValue < 100)){
     u8g2.setCursor(90, 75); 
     u8g2.print(pressureKgfValue,1);
    }
    else{
     u8g2.setCursor(65, 75); 
     u8g2.print(pressureKgfValue,1);
    }

   //  u8g2.setFont(u8g2_font_crox5hb_tf  );
   // u8g2.drawStr(152,75,".");
     
     u8g2.setFont(u8g2_font_fub14_tf  );
     u8g2.setCursor(185, 120); 
     u8g2.print(setPointKgf);
    }

    u8g2.setFont(u8g2_font_victoriabold8_8r    );
    u8g2.drawStr(1,120,"TotalTyre:");
    u8g2.setFont(u8g2_font_victoriabold8_8u    );
    u8g2.setCursor(80 , 120); 
    u8g2.print(TyreCount_value);
    
//    u8g2.setFont(u8g2_font_victoriabold8_8u  );
//    u8g2.drawStr(160,120,"Tyre/day:");
//    u8g2.setCursor(220 , 120); 
//    u8g2.print(TyreCount_value);

     
    if ( operation_mode == 1 ){
    u8g2.setFont(u8g2_font_victoriabold8_8u   );
    u8g2.drawStr(140,13,"VAC");
    
    u8g2.setFont(u8g2_font_victoriabold8_8u   );
    if (setTimer < 10){
     u8g2.setCursor(88, 98); 
     u8g2.print(setTimer);
    }
    else{
    u8g2.setCursor(80, 98); 
    u8g2.print(setTimer);
    }
    
    u8g2.setFont(u8g2_font_victoriabold8_8u   );
    u8g2.drawStr(95,98,".0S");
    }
    
    u8g2.setFont(u8g2_font_fub14_tf   );
    u8g2.drawStr(130,120,"SET:");
    
    u8g2.sendBuffer();
        
}

void draw_init(void) {  
  char buf [100];
  char buf1 [100];
  char buf2 [100];
   u8g2.firstPage();  
  do {
    u8g2.setFont(u8g2_font_fub17_tr );
  //  u8g2.drawStr( 0, 40, "GOLDEN PEACOCK");
    recivedData.toCharArray(buf1, 70);
     u8g2.drawStr( 0, 45,buf1);
    
    u8g2.setFont(u8g2_font_8x13B_tf );
 //   u8g2.drawStr( 0, 60, "Authorized :");
   
    recivedData2.toCharArray(buf, 70);
    u8g2.drawStr( 0, 110,buf);
    
    recivedData1.toCharArray(buf2, 70);
    u8g2.drawStr( 0, 85,buf2);

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(170,124,"SW Ver:J51");
    
  
  } while( u8g2.nextPage() );
for (int delay_100 = 0;delay_100<20;delay_100++){
     buzzer_logic(0);
     delay(200);
     wdi_time_reset();
     buzzer_logic(1);
     delay(200);
    
    }
  
}


void draw_Calibiration(void) {  

   u8g2.firstPage();  
  do {

    u8g2.setFont(u8g2_font_8x13B_tf );
    u8g2.drawStr( 0, 20, "      Calibration in PSI        "); //2.660 -1.310 -2.660 =1.368
    
    if (config_parameter_mode ==0){
     u8g2.drawStr( 0, 50, "Actual Pressure   :");
   
    u8g2.setCursor(155, 50); //125
    u8g2.print(pressureSensorValue,1);
     
     u8g2.drawStr( 0, 80, "Correction Factor :");
     u8g2.setCursor(155, 80); //125
     u8g2.print(correctionfactor,1);
    }
    else if (config_parameter_mode == 1) {     
     u8g2.drawStr( 0, 80, "Vacuum Timing %:");
     u8g2.setCursor(155, 80); //125
     u8g2.print(correctionfactor_vac_time,1);
      }
    else if (config_parameter_mode == 2) {
     u8g2.drawStr( 0, 80, "InFlate Timing %:");
     u8g2.setCursor(155, 80); //125
     u8g2.print(correctionfactor_pressure_time,1);
     }  
  } while( u8g2.nextPage() );
for (int delay_100 = 0;delay_100<5;delay_100++){
     delay(10);
     wdi_time_reset();
    }
  
}

void draw(void) {
  u8g2.setColorIndex(1);
  drawTexture();
}

void flash()
{
    setTimer--;
    vac_timer_bit = 1;
    
    if (setTimer == 0){
      vac_timer_bit = 0;
//      MsTimer2::stop();
      vac_timer_SS = 0;
      setBitTimer = 1;
      Vac_mode_pressure = 1;
      Vac_mode_relay = 1;
      Vacam_mode_Relay_off_Logic(0);
      pressureSensorValue = 6;
      Pressure_relay();
      Timer1_fun();
    }
}


//void hx711_setup(){
//  
//   scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN,64);
//
//  // RS485_serial.println("Before setting up the scale:");
//  // RS485_serial.print("read: \t\t");
//  RS485_serial.println(scale.read());      // print a raw reading from the ADC
//
//  // RS485_serial.print("read average: \t\t");
//  RS485_serial.println(scale.read_average(20));   // print the average of 20 readings from the ADC
//
//  // RS485_serial.print("get value: \t\t");
//  RS485_serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)
//
//  // RS485_serial.print("get units: \t\t");
//  RS485_serial.println(scale.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
//            // by the SCALE parameter (not set yet)
//
//  scale.set_scale(2280.f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
//  scale.tare();               // reset the scale to 0
//
//  }
//
// void hx711_read(){
//
//   int presuretpd = 0; 
//   
//
//  if (scale.wait_ready_timeout(sensorereaddelay,0)) {
//    long reading = scale.read(); //  scale.read_average(2);
//    // RS485_serial.print("HX711 reading: ");
//    // RS485_serial.println(reading);
//    pressureSensorRaw = (reading - offset) * 0.0157;
//    presuretpd = (reading - offset) * 0.0000157392; //0.000029511
//
//    //RS485_serial.print("Read : ");
//   //RS485_serial.println(presuretpd);
//
//    if (presuretpd < 5  ){
//      presuretpd = 0;
//    }
//    pressureSensorValue = presuretpd;
//    pressureSensorRaw = pressureSensorValue;
//  } 
//  else {
//    RS485_serial.println("HX711 not found.");
//  
//    }
//  }
//
//


int sensorRead(){
unsigned long temp_Presure_Read = 0;
unsigned int Presure_read_psi = 0; 
 //RS485_serial.print("ang = ");
 
    float x11 =0.0; 
    float y11 =0.0;
    float x22 = 0.0;
    float y22 = 0.0;
    float z11 = 0.0;
    
  // 05v to 4.5v 0 to 15 par
  // 05v to 4.5v 0 to 15 par
  // 0.25v to 2.25 after conversion 0 - 2.5 1024
  // PSI = 
  // 1024/2500 *sensor =
 for(unsigned int j = 0; j < 2; j++){

  if (mode_LMV_HCV_1 == 1){
          pressureSensor[j] = analogRead(Pressure_in);
       }
  else{ 
          pressureSensor[j] = analogRead(Pressure_in2);
        }
   // RS485_serial.print("adc = ");RS485_serial.print(pressureSensor[j]);
    temp_Presure_Read = temp_Presure_Read + pressureSensor[j];
  }

 pressureSensorValue = temp_Presure_Read / 2;
 pressureSensorRaw = pressureSensorValue;
//2.656 = 120 psi

    x11 = correctionfactor;//point_prv ;  95.00 == 200 
    y11 = 1024.00;//Point_count; 1024
    
    x22 = 0.0; // point_leve_prv; 0 
    y22  = 255.0; //Point_level;  217   278 
    z11 = (float)pressureSensorValue;  
   
 if (pressureSensorValue >= correctionfactor + 1) {
  Presure_read_psi =  fuelcalib(x11,y11,z11,x22,y22);
  if (Presure_read_psi < 5){Presure_read_psi = 0;}
  pressureSensorValue = Presure_read_psi;
  }
  else {
     Presure_read_psi = 0;
    pressureSensorValue = Presure_read_psi;
    }
  
//  hx711_read();

if ( END_alram_set == 1  && pressureSensorValue != 0) {pressureSensorValue = setPoint;}

   pressureBarValue = pressureSensorValue/14.501;
   pressureKgfValue = pressureSensorValue*0.07031;
   
  return pressureSensorValue;
}



 void N2Purity(){
     ABC_bit = !ABC_bit;    
    if (abCylinderLED_SetBit == 0 || Vacam_bit_state == 0 || AB_cylinder == 0 ) {
       digitalWrite(abCylinderLED, 0);
      }
    else if(abCylinderLED_SetBit == 1 && Vacam_bit_state == 1) {
        digitalWrite(abCylinderLED, ABC_bit);
       }
    }


    
void buzzer_logic(boolean buzzer_bit ){
digitalWrite(buzzerLED, buzzer_bit);
}

void end_alram(){
 // RS485_serial.print("alm = ");
int alm_time_count = 20;

  if ( alarm_bit == 0){
    sensorRead1 = 1;
    sensorRead(); 
     sensorRead1 = 0;
    Display_referesh();

   for (int i=0;i< alm_time_count;i++){
        buzzer_logic(1);
        delay(200);
        buzzer_logic(0);
        delay(200);
         sensorRead1 = 1;
        sensorRead(); 
         sensorRead1 = 0;
      if( pressureSensorValue == 0 ){i = alm_time_count+1;}
      } 
           vacumRelease_logic =1;
           vacumRelease();
        alarm_bit = 1;      
    }
}

void vacumRelease(){
  if (vacumRelease_logic ==1){
           digitalWrite(vacummPressureLED, HIGH);
           digitalWrite(vacummLED, HIGH);
           delay(2500);
           digitalWrite(vacummPressureLED, LOW);
           digitalWrite(vacummLED, LOW);
           vacumRelease_logic =0;
  }
}


void Button_Read(){
/*  increment  button logic    */ 

if(digitalRead(mode_LMV_HCV) == LOW){
   mode_LMV_HCV_1 =  0;
     if (mode_LMV_HCV_1_setbit == 1 && pressureSensorValue == 0  ){ 
      setPoint = 40;  
      get_ondelay_time ();
      Display_referesh(); 
      }
     mode_LMV_HCV_1_setbit = 0; 
  }
  else {
     mode_LMV_HCV_1 =  1;
     if (mode_LMV_HCV_1_setbit == 0 && pressureSensorValue == 0  ){
       setPoint = 40; 
       get_ondelay_time (); 
       Display_referesh();}
     mode_LMV_HCV_1_setbit = 1;
    }


if(digitalRead(incrementButton) == LOW){
  incrementButton_1++; 
  button_release_inc = 1;
  incrementButton_bz = 1;
  
  if(incrementButton_1 >= debounce_inc_time){
      debounce_inc_time = 1;
     if(operation_mode == 1 && enter_mode == 1 && pressureSensorValue == 0 && Prog_mode_State == 1 && config_mode_state == 1){
      setTimer++;
      temp_cnt++;
      if (temp_cnt > 3){
      setTimer = setTimer + 4;
      temp_cnt = 4;
   //   RS485_serial.print("setTimer + 4 = "); RS485_serial.println(setTimer);
      }
      
      setTimerOld  = setTimer;
   //   RS485_serial.print("setTimer = "); RS485_serial.println(setTimer);
     button_buzzer_logic();
    
     Display_referesh();
     
     }
   else  if((operation_mode == 0  && pressureSensorValue == 0)||(operation_mode == 1 && enter_mode == 0 && Prog_mode_State == 1 && config_mode_state == 1)){
        setPoint++;       

         temp_cnt++;
      
      if (temp_cnt > 3){
      setPoint = setPoint + 4;
      temp_cnt = 4;
   //   RS485_serial.print("setPoint + 4 = "); RS485_serial.println(setPoint);
      }
              
        EEPROM.update(setPoint_address, setPoint);
   //     RS485_serial.print("setPoint = "); RS485_serial.println(setPoint);
   //     RS485_serial.print("temp_cnt = "); RS485_serial.println(temp_cnt);
        setPointBar = (float)setPoint/14.501;
        setPointKgf = (float)setPoint*0.07031; 
        button_buzzer_logic();
        incrementButton_bz = 1;
        Display_referesh();
     }
     else if (Prog_mode_State == 0 && config_mode_state == 0){
      calib_inc_repeat_cnt++;
      if (calib_inc_repeat_cnt >= CALIB_REPEAT_TICKS){
      calib_inc_repeat_cnt = 0;
      if(config_parameter_mode == 0){
      correctionfactor++;
      eeprom_calib_pending = 1; eeprom_calib_address = Caliberation_address; eeprom_calib_value = (byte)correctionfactor;
      }
      else if (config_parameter_mode == 1){
        correctionfactor_vac_time++;
        eeprom_calib_pending = 1; eeprom_calib_address = vac_time_address; eeprom_calib_value = (byte)correctionfactor_vac_time;
        }
      else if (config_parameter_mode == 2){
        correctionfactor_pressure_time++;
        eeprom_calib_pending = 1; eeprom_calib_address = pressure_time_address; eeprom_calib_value = (byte)correctionfactor_pressure_time;
        }
      }
      }
  if ( operation_mode == 2){
        Pressure_relay_st(1);
      }
      incrementButton_1 =0;
     }
  } 
      else {
        incrementButton_bz = 0;
         temp_cnt = 0;
        debounce_inc_time = 1;
        calib_inc_repeat_cnt = 0;

    if ( operation_mode == 2 && button_release_inc == 1 ){
          Pressure_relay_st (0);
          button_release_inc = 0 ;

      }
    }


/*  decrement  button logic    */      
if(digitalRead(decrementButton) == LOW){
  decrementButton_1++; 
  button_release_dec = 1;
  decrementButton_bz = 1;
  
  if(decrementButton_1 >= debounce_dec_time){
    debounce_dec_time = 1;
      if(operation_mode == 1 && enter_mode == 1 && pressureSensorValue == 0  && Prog_mode_State == 1 && config_mode_state == 1){
      setTimer--;
       temp_cnt_dec++;
      if (temp_cnt_dec > 3){
          setTimer = setTimer - 4;
          temp_cnt_dec = 4;
   //       RS485_serial.print("setTimer - 4 = "); RS485_serial.println(setTimer);
          }
          
      setTimerOld  = setTimer;
      button_buzzer_logic();
     
     Display_referesh();
    //  RS485_serial.print("setTimer = "); RS485_serial.println(setTimer);
       }
    else if((operation_mode == 0 && pressureSensorValue == 0) ||(operation_mode == 1 && enter_mode == 0   && Prog_mode_State == 1 && config_mode_state == 1)){
         setPoint--;
         temp_cnt_dec++;
         if (temp_cnt_dec > 3){
         setPoint = setPoint - 4;
         temp_cnt_dec = 4;
 //        RS485_serial.print("setPoint - 4 = "); RS485_serial.println(setPoint);
         }
        EEPROM.update(setPoint_address, setPoint);
  //      RS485_serial.print("setPoint = "); RS485_serial.println(setPoint);
        setPointBar = (float)setPoint/14.501;
        setPointKgf = (float)setPoint*0.07031; 
        button_buzzer_logic();
        decrementButton_bz = 1;
        Display_referesh();
       }
     else if ( Prog_mode_State == 0 && config_mode_state == 0){
      calib_dec_repeat_cnt++;
      if (calib_dec_repeat_cnt >= CALIB_REPEAT_TICKS){
      calib_dec_repeat_cnt = 0;
      if(config_parameter_mode == 0){
          correctionfactor--;
          eeprom_calib_pending = 1; eeprom_calib_address = Caliberation_address; eeprom_calib_value = (byte)correctionfactor;
          }
      else if (config_parameter_mode == 1){
        correctionfactor_vac_time--;
        eeprom_calib_pending = 1; eeprom_calib_address = vac_time_address; eeprom_calib_value = (byte)correctionfactor_vac_time;
        }
      else if (config_parameter_mode == 2){
        correctionfactor_pressure_time--;
        eeprom_calib_pending = 1; eeprom_calib_address = pressure_time_address; eeprom_calib_value = (byte)correctionfactor_pressure_time;
        }
      }
      }
     if ( operation_mode == 2){  
                AB_cylinder = 0;
                N2Purity();    
                Vacam_mode_Relay_Logic(1);
              }
       if (setTimer <= 0){setTimer = 0;}       
       decrementButton_1 =0;
         }
      }
      else {
         decrementButton_bz = 0;
        temp_cnt_dec = 0;
          AB_cylinder = 1;
          debounce_dec_time = 1;
          calib_dec_repeat_cnt = 0;
            if ( operation_mode == 2 && button_release_dec == 1){         
                 Vacam_mode_Relay_off_Logic(0);               
                 button_release_dec = 0;
                 
              }
        }

/*  mode  button logic    */
 if(digitalRead(modeButton) == LOW){
 modeButton_1++; 
 modeButton_bz = 1;
  if(modeButton_1 > debounce_time && modeButton_state == false && startStop_mode == 0 && pressureSensorValue == 0 && startStop_mode_0 == 0){
        operation_mode++;
        enter_mode = 0;
      //  buzzer_logic(1);
      
        Display_referesh();
        modeButton_state = true;
        
        if (config_mode_state == 1){
          if (operation_mode >= 3){operation_mode = 0;}
          }
          else {
       if (operation_mode >= 5){operation_mode = 0;}
       }  
          modeButton_1 = debounce_time+1 ;
          
         }

         if ( config_mode_state == 0 && modeButton_1 > debounce_time && configParamButton_state == false){
           config_parameter_mode++;
         //  RS485_serial.print("config_parameter_mode = "); RS485_serial.println(config_parameter_mode);
           get_ondelay_time ();
           if (config_parameter_mode >= 3){ config_parameter_mode = 0;}
           configParamButton_state = true;
           }
       }
       else {modeButton_1 = 0; modeButton_state = false; modeButton_bz = 0; configParamButton_state = false;}


/*  unit  button logic    */
 if(digitalRead(unitButton) == LOW){
 unitButton_1++; 
 unitButton_bz = 1;
  if(unitButton_1 > debounce_time && unitButton_state == false){
       unit_mode++;
       // buzzer_logic(1);
        
        Display_referesh();
        if (unit_mode >= 3){unit_mode = 0;}  
          unitButton_1 = debounce_time + 1;
          unitButton_state = true;
       //   RS485_serial.print("unit_mode = "); RS485_serial.println(unit_mode);
         }
     }
     else {unitButton_1 = 0; unitButton_state = false;unitButton_bz = 0;}

  
/*  enter  button logic    */
if(digitalRead(enterButton) == LOW){
  enterButton_1++; 
  enterButton_bz = 1;
  if(enterButton_1 > debounce_time && startStop_mode == 0 && operation_mode == 1){
       enter_mode = !enter_mode;
        TyreCount_bit = 0;
        enterButton_1 = 0;
       // buzzer_logic(1); 
       
        Display_referesh();
      //  RS485_serial.print("enter_mode = "); RS485_serial.println(enter_mode);
      }
  else if (enterButton_1 > debounce_time && operation_mode == 3){
        setPoint = 150;
        EEPROM.update(setPoint_address, setPoint);
        operation_mode = 0;
        enterButton_1 = 0;  
       // buzzer_logic(1);
        enterButton_bz = 1;
        Display_referesh();  
        }
   else if (enterButton_1 > debounce_time && operation_mode == 4){
        setPoint = 30;
        EEPROM.update(setPoint_address, setPoint);
        operation_mode = 0;
        enterButton_1 = 0;
       // buzzer_logic(1);
        enterButton_bz = 1;
        Display_referesh();
        }    
      }else {enterButton_bz =0;}

      /*  startStopButton  button logic    */ 
if(digitalRead(startStopButton) == LOW){
 startStopButton_1++; 
 startStopButton_bz = 1;
  if(startStopButton_1 > debounce_time && startStopButton_state == false && operation_mode == 1){
       vac_timer_bit = 1;
       startStop_mode = !startStop_mode;
      // buzzer_logic(1);
      
       Display_referesh();
       startStopButton_1 = debounce_time + 1;
       startStopButton_state = true;
         if (startStop_mode_0 == 0){
            vacumRelease_logic =1;
            }
          
     //  RS485_serial.print("startStop_mode 1 = "); RS485_serial.println(startStop_mode);
       
         }
  else if (startStopButton_1 > debounce_time && startStopButton_state == false && operation_mode == 0){
          startStop_mode_0 = !startStop_mode_0;
          no_inlet_value_on = 0;
         // buzzer_logic(1);      
          startStopButton_bz = 1;   
          startStopButton_1 = debounce_time + 1;
          startStopButton_state = true;
        //  RS485_serial.print("startStop_mode 0 = "); RS485_serial.println(startStop_mode_0);
          if (startStop_mode_0 == 0){
            vacumRelease_logic =1;
            Off_relays();
            }
            else{ Pressure_SetBit = HIGH; }
          }
      }
  else {  startStopButton_1 =0; startStopButton_state = false;  startStopButton_bz = 0;}
   button_buzzer_logic();  
}

void button_buzzer_logic(){
  if (incrementButton_bz == 1|| decrementButton_bz == 1 || modeButton_bz == 1 || unitButton_bz == 1 || enterButton_bz == 1 || startStopButton_bz == 1 ){
   buzzer_logic(1); 
   temp_buzzer_reset_bit = 1;
  }
  else {
       if(temp_buzzer_reset_bit == 1){
       buzzer_logic(0);
       temp_buzzer_reset_bit = 0;
      }
    
   }
 }
 
void PinModeConfig(){

  // initialize the pushbutton pin as an input:
  pinMode(startStopButton, INPUT_PULLUP);
  pinMode(unitButton, INPUT_PULLUP);
  pinMode(modeButton, INPUT_PULLUP);
  pinMode(enterButton, INPUT_PULLUP);
  pinMode(incrementButton, INPUT_PULLUP);
  pinMode(decrementButton, INPUT_PULLUP);
  pinMode(Prog_mode, INPUT_PULLUP);
  pinMode(Config_mode, INPUT_PULLUP);
  pinMode(mode_LMV_HCV, INPUT_PULLUP);
  
  analogReference(ADC_REFERENCE);
  RS485_serial.print("TyreCount_value = ");  RS485_serial.println(TyreCount_value); 
    
 // initialize the LED as an output:
  pinMode(GLCD_LED, OUTPUT);
  digitalWrite(GLCD_LED, HIGH);

  pinMode(GLCD_FS, OUTPUT);
  digitalWrite(GLCD_FS, LOW); // if FS pin defective then we need to set to input mode
  
  //  pinMode(GLCD_FS, INPUT_PULLUP);
  // digitalWrite(GLCD_FS, LOW); // if FS pin defective then we need to set to input mode
  

  pinMode(GLCD_RD, OUTPUT);
  digitalWrite(GLCD_RD, HIGH);

//  pinMode(Status_LED, OUTPUT);
//  digitalWrite(Status_LED, LOW);
  
  pinMode(WDI, OUTPUT);
  digitalWrite(WDI, LOW);
  
  
  pinMode(vacummPressureLED, OUTPUT);
  digitalWrite(vacummPressureLED, LOW);
  
  pinMode(vacummLED, OUTPUT);
  digitalWrite(vacummLED, LOW);
  
  pinMode(pressureLED, OUTPUT);
  digitalWrite(pressureLED, LOW);
  
  pinMode(abCylinderLED, OUTPUT);
  digitalWrite(abCylinderLED, LOW);
  
  pinMode(buzzerLED, OUTPUT);//buzzerLED
  digitalWrite(buzzerLED, LOW);
  
  pinMode(abCylinderLED1, OUTPUT);
  digitalWrite(abCylinderLED1, HIGH);

        vacummPressureLED_State = digitalRead(vacummPressureLED);
        vacummLED_State = digitalRead(vacummLED);
        abCylinderLED1_State = digitalRead(abCylinderLED1);
        Prog_mode_State = digitalRead(Prog_mode);
        config_mode_state  = digitalRead(Config_mode);
 }

void Off_relays(){
   digitalWrite(vacummPressureLED, LOW);
   digitalWrite(vacummLED, LOW);
   digitalWrite(pressureLED, LOW);
    Vac_mode_pressure = 0;
    no_inlet_value_on = 0;
    On_Time_Presure = 2000;
  }


 void Display_referesh(){
 int temp_count = 0;

 u8g2.firstPage(); 
  do {  
    draw(); 
    wdi_time_reset();
    if (temp_count > 1){buzzer_logic(0);}
    temp_count++;
  } while( u8g2.nextPage() );
    
}



  
unsigned int fuelcalib(float x11,float y11,float z11,float x22,float y22)
{
   float f1 =  y11 - x11;
   float f2 =  z11 - x11;
   float f3 =  (f2/f1) * 100;
   
   float h1 = y22 - x22;
   float h2 = (h1/100) * f3;
   float h3 = x22 + h2;
   
   return (unsigned int)h3; 
}

void TyreCount(){
  
  if (Tyre_connect == 1 && error_percentage_bit == 0 && TyreCount_bit == 0 ) { 
    
    if( Tyre_completed > 4 ){ //samplese for completion
      Tyre_completed = 5;
      if (operation_mode == 1 && startStop_mode == 1 && vac_timer_bit == 0){
        TyreCount_bit = 1;
        startStop_mode = 0;
        Display_referesh();
        END_alram_set = 1;   
        end_alram();
        END_alram_set = 0;
          }
      else if (operation_mode == 0 ){
        TyreCount_bit = 1;
        if (vac_timer_bit == 0){
        startStop_mode = 0;
        Display_referesh();   
        END_alram_set = 1;
        end_alram();
        END_alram_set = 0; 
        RS485_serial.println("END1"); 
      }
     }
    else { TyreCount_bit = 0; }
     }
    
    }
  
if (pressureSensorValue == 0 && TyreCount_bit == 1  )
  {
  
   TyreCount_value++;    
   TyreCount_Total++;
   no_inlet_value_on = 0;
//   EEPROM.update(TyreCount_address, TyreCount_value);
   Write_int_eeprom(TyreCount_value,TyreCount_address);
   TyreCount_bit = 0;
   alarm_bit = 0;
    if (vac_timer_bit == 0){
        startStop_mode = 0;
         }
  // RS485_serial.print("TyreCount_value = ");  RS485_serial.println(TyreCount_value); 
  }
  
  }


void wdi_time_reset(){
  digitalWrite(WDI, !digitalRead(WDI));
//  digitalWrite(Status_LED, !digitalRead(Status_LED));
 }



void Pressure_relay(){
//  1 = count  0.4096mv 
//  sensorRead();

float temp_press = pressureSensorValue;
float temp_setPoint = setPoint; 
float temp_press2 = pressureSensorValue;

// RS485_serial.print("SensorValue : "); RS485_serial.println(pressureSensorValue); 
 

if (pressureSensorValue <= 5 && Vac_mode_pressure == 1 && operation_mode == 1) {
    temp_press2 = pressureSensorValue;
    pressureSensorValue = 6;
    on_delay++;
  if (on_delay > on_delay_cnt){on_delay = on_delay_cnt;}
 ///   RS485_serial.print("pressureSensorValue = 6");
  }
 else if (pressureSensorValue == 0) {
  temp_press = 1;
  Tyre_connect = 0;
  Tyre_connect_vac = 0;
  pid_error = 0;
  on_delay = 0;
  PID_clear();
  }
else {
  temp_press = pressureSensorValue;
  on_delay++;
  if (on_delay > on_delay_cnt){on_delay = on_delay_cnt;}
  }


if (pressureSensorValue <= 5 && operation_mode == 0 && no_inlet_value_on <= 2 && startStop_mode_0 == 1 ) {
 //  RS485_serial.print("startStop_mode_0 = ");  RS485_serial.println(startStop_mode_0); 
   temp_press2 = pressureSensorValue;
   pressureSensorValue = 6;  
   temp_press = pressureSensorValue;
   on_delay = on_delay_cnt;
  }

if (pressureSensorValue <= 5 && no_inlet_value_on >= 3 && startStop_mode_0 == 1){
  startStop_mode_0 = 0;
  Vac_mode_pressure = 0;
  no_inlet_value_on = 0;
  }
  
if (setPoint == 0) {temp_setPoint = 10.0;}
else {temp_setPoint = setPoint;}

 if (pressureSensorValue >= 5 && pressureSensorValue <= (setPoint + 1) && TyreCount_bit == 0 && on_delay == on_delay_cnt)  {
    
 error_percentage = ((float)temp_press / (float)temp_setPoint) * 100.0;
 //|| pressureSensorValue == (setPoint+1) || pressureSensorValue == (setPoint-1)
 if( error_percentage == 100 || pressureSensorValue == ( setPoint + 1 ) ){Error_slot = 0 ; Pressure_relay_logic  = 0; On_Time_Presure = 0; Off_Time_Presure = offdelaytime; error_percentage_bit = 0; Vac_mode_pressure = 0; pid_error = 0; Tyre_completed++; Tyre_connect = 1;Vacam_fast_bit_state = 0 ;}

 else if( error_percentage >= 95 && error_percentage < 100 ){Error_slot = 1 ;Pressure_relay_logic = 1;On_Time_Presure = PresureT1; Off_Time_Presure = offdelaytime; error_percentage_bit = 1; Tyre_connect = 1; Vacam_fast_bit_state = 0 ;}

 else if( error_percentage >= 90 && error_percentage < 95 ){Error_slot = 2 ;Pressure_relay_logic = 1;On_Time_Presure = PresureT2; Off_Time_Presure = offdelaytime; error_percentage_bit = 1; Tyre_connect = 1;Vacam_fast_bit_state = 0 ; }
    
 else if( error_percentage >= 75 && error_percentage < 90 ){Error_slot = 3 ;Pressure_relay_logic = 1;On_Time_Presure = PresureT3; Off_Time_Presure = offdelaytime; error_percentage_bit = 1; Tyre_connect = 1;Vacam_fast_bit_state = 0 ; }
 
 else if( error_percentage >= 50 && error_percentage < 75){Error_slot = 4 ;Pressure_relay_logic = 1;On_Time_Presure  = PresureT4; Off_Time_Presure = offdelaytime; error_percentage_bit = 1;Tyre_connect = 1;Vacam_fast_bit_state = 0 ;}

 else if( error_percentage >= 15 && error_percentage < 50 ){Error_slot = 5 ;Pressure_relay_logic = 1;On_Time_Presure = PresureT5; Off_Time_Presure = (offdelaytime+500); error_percentage_bit = 1;Tyre_connect = 1;Vacam_fast_bit_state = 0 ;}

 else if( error_percentage < 15){Error_slot = 6 ;Pressure_relay_logic = 1;On_Time_Presure = PresureT6; Off_Time_Presure   = (offdelaytime+500); error_percentage_bit = 1; Tyre_connect = 1;Vacam_fast_bit_state = 0 ;}
  
 } 
 else if ( pressureSensorValue >= 5 && pressureSensorValue >= (setPoint +1)  && on_delay == on_delay_cnt ) {  
          
           error_percentage = ( (float)temp_setPoint) / (float)temp_press * 100.0;
          // 
           if( error_percentage >= 97 || pressureSensorValue == (setPoint+1)){Pressure_relay_logic  = 0; On_Time_Presure = 0; Off_Time_Presure = offdelaytime; error_percentage_bit = 0; Vac_mode_pressure = 0;  pid_error = 0;  Tyre_completed++; Tyre_connect = 1;Vacam_fast_bit_state = 0 ;}
          
           else if( error_percentage >= 95 && error_percentage < 98 ){Pressure_relay_logic = 2;On_Time_Presure = VacT1; Off_Time_Presure = offdelaytime; error_percentage_bit = 1; Vacam_fast_bit_state = 0 ; }

           else if( error_percentage >= 90 && error_percentage < 95 ){Pressure_relay_logic = 2;On_Time_Presure = VacT2; Off_Time_Presure = offdelaytime; error_percentage_bit = 1; Vacam_fast_bit_state = 0 ;  }
            
           else if( error_percentage >= 75 && error_percentage < 90 ){Pressure_relay_logic = 2;On_Time_Presure = VacT3; Off_Time_Presure = offdelaytime; error_percentage_bit = 1; Vacam_fast_bit_state = 0 ; }
              
           else if( error_percentage >= 50 && error_percentage < 75 ){Pressure_relay_logic = 2;On_Time_Presure = VacT4; Off_Time_Presure = offdelaytime; error_percentage_bit = 1; Vacam_fast_bit_state = 1 ; }
          
           else if( error_percentage >= 15 && error_percentage < 50 ){Pressure_relay_logic = 2;On_Time_Presure = VacT5; Off_Time_Presure = offdelaytime; error_percentage_bit = 1;Vacam_fast_bit_state = 1 ;}
          
           else if( error_percentage < 15){Pressure_relay_logic = 2;On_Time_Presure = VacT6; Off_Time_Presure = offdelaytime; error_percentage_bit = 1; Vacam_fast_bit_state = 1 ;}
          PID_clear();
 }
 else { 
       On_Time_Presure = 0; Off_Time_Presure = offdelaytime; error_percentage_bit = 0; Pressure_relay_logic  = 0; 
      }
//      RS485_serial.print("Pressure_relay_logic"); RS485_serial.println(Pressure_relay_logic);
//       RS485_serial.print("On_Time_Presure"); RS485_serial.println(On_Time_Presure);
//       RS485_serial.print("Off_Time_Presure"); RS485_serial.println(Off_Time_Presure);
  pressureSensorValue = temp_press2 ;
       
//       RS485_serial.print("pressureSensorValue"); RS485_serial.println(pressureSensorValue);
//       RS485_serial.println();
       
  }
  
void get_ondelay_time () {
  
  if (setPoint < 80) {
    tyire_mode =1;
    offdelaytime = 2600; //6000//15mtr 
   /* PresureT1 = 120;
    PresureT2 = 250;
    PresureT3 = 700;   
    PresureT4 = 1400; 
    PresureT5 = 2000;
    PresureT6 = 3000;

    VacT1 = 300;
    VacT2 = 800;
    VacT3 = 2600;
    VacT4 = 3500;
    VacT5 = 4300;
    VacT6 = 6000;
  */
    PresureT1 = 120;
    PresureT2 = 260;
    PresureT3 = 600;
    PresureT4 = 1200; 
    PresureT5 = 1800;
    PresureT6 = 2000;  
  

    VacT1 = 250;
    VacT2 = 650;
    VacT3 = 1500;
    VacT4 = 4000;
    VacT5 = 6000;
    VacT6 = 10000;
    
    PresureT1 = PresureT1*(correctionfactor_pressure_time / 100);
    PresureT2 = PresureT2*(correctionfactor_pressure_time / 100);
    PresureT3 = PresureT3*(correctionfactor_pressure_time / 100);
    PresureT4 = PresureT4*(correctionfactor_pressure_time / 100);
    PresureT5 = PresureT5*(correctionfactor_pressure_time / 100);
    PresureT6 = PresureT6*(correctionfactor_pressure_time / 100);
    
    VacT1 = VacT1*(correctionfactor_vac_time / 100);
    VacT2 = VacT2*(correctionfactor_vac_time / 100);
    VacT3 = VacT3*(correctionfactor_vac_time / 100);
    VacT4 = VacT4*(correctionfactor_vac_time / 100);
    VacT5 = VacT5*(correctionfactor_vac_time / 100);
    VacT6 = VacT6*(correctionfactor_vac_time / 100);

    
  }
  else { 
    
    offdelaytime = 2900;
    tyire_mode =2;
    PresureT1 = 450 *10;
    PresureT2 = 1200 *10; //
    PresureT3 = 2600 *10;   
    PresureT4 = 3000 *10; 
    PresureT5 = 3100 *10;
    PresureT6 = 3200 *10;

    VacT1 = 450   *10;
    VacT2 = 900   *10;
    VacT3 = 1800  *10;
    VacT4 = 2600  *10;
    VacT5 = 3000  *10;
    VacT6 = 3200  *10;
    if (correctionfactor_pressure_time < 120) {correctionfactor_pressure_time = correctionfactor_pressure_time*1.5;}

    PresureT1 = PresureT1*(correctionfactor_pressure_time / 100);
    PresureT2 = PresureT2*(correctionfactor_pressure_time / 100);
    PresureT3 = PresureT3*(correctionfactor_pressure_time / 90);
    PresureT4 = PresureT4*(correctionfactor_pressure_time / 80);
    PresureT5 = PresureT5*(correctionfactor_pressure_time / 70);
    PresureT6 = PresureT6*(correctionfactor_pressure_time / 70);
    
    VacT1 = VacT1*(correctionfactor_vac_time / 100);
    VacT2 = VacT2*(correctionfactor_vac_time / 100);
    VacT3 = VacT3*(correctionfactor_vac_time / 100);
    VacT4 = VacT4*(correctionfactor_vac_time / 100);
    VacT5 = VacT5*(correctionfactor_vac_time / 100);
    VacT6 = VacT6*(correctionfactor_vac_time / 100);
  }
   
   
 
    
   /* 
    RS485_serial.print("VTC = "); RS485_serial.println(correctionfactor_vac_time);
    RS485_serial.print("PTC = "); RS485_serial.println(correctionfactor_pressure_time);
    RS485_serial.print("PT = "); 
    RS485_serial.print(PresureT1);RS485_serial.print(","); 
    RS485_serial.print(PresureT2);RS485_serial.print(","); 
    RS485_serial.print(PresureT3);RS485_serial.print(","); 
    RS485_serial.print(PresureT4);RS485_serial.print(","); 
    RS485_serial.print(PresureT5);RS485_serial.print(","); 
    RS485_serial.println(PresureT6);
    RS485_serial.print("VT = "); 
    RS485_serial.print(VacT1);RS485_serial.print(","); 
    RS485_serial.print(VacT2);RS485_serial.print(","); 
    RS485_serial.print(VacT3);RS485_serial.print(","); 
    RS485_serial.print(VacT4);RS485_serial.print(","); 
    RS485_serial.print(VacT5);RS485_serial.print(","); 
    RS485_serial.println(VacT6); 
    */
}

void Pressure_relay_state() {
  int temp_bit = !Pressure_SetBit; 

 if ( operation_mode == 1 && startStop_mode == 0)  {  
   digitalWrite(pressureLED, LOW);
   Pressure_coil_bit = 0;
   return;
   } 
 else if (vac_timer_bit == 1){
    digitalWrite(pressureLED, LOW);
    Pressure_coil_bit = 0;
    return;
    }
 else {    
  if ((error_percentage_bit  == 1) && (Pressure_relay_logic == 1))  {
    PID_calculation();
    digitalWrite(pressureLED, temp_bit);
      Pressure_coil_bit = temp_bit;
    }
  else {
    digitalWrite(pressureLED, LOW);
    Pressure_coil_bit = 0;
    }
   }
  }

void Pressure_relay_st (int state) {
    digitalWrite(pressureLED, state);
     Pressure_coil_bit = state;
   }

   
void Vacam_mode_Relay_Logic(int state){
       
        vacummPressureLED_State = digitalRead(vacummPressureLED);
        vacummLED_State = digitalRead(vacummLED);
        abCylinderLED1_State = digitalRead(abCylinderLED1);
        
      //   RS485_serial.print("relyon = "); 
          
         if (vacummPressureLED_State == HIGH && vacummLED_State == HIGH && abCylinderLED1_State == LOW){}
else{
         digitalWrite(abCylinderLED1, !state);
         delay(100); 
         digitalWrite(vacummPressureLED, state);
         digitalWrite(vacummLED, state);
         vacum_coil_bit = state;
         }
}

void Vacam_mode_Relay_off_Logic(int state){
           
        vacummPressureLED_State = digitalRead(vacummPressureLED);
        vacummLED_State = digitalRead(vacummLED);
        abCylinderLED1_State = digitalRead(abCylinderLED1);

      //  RS485_serial.print("relyoff = ");
        
if (vacummPressureLED_State == LOW && vacummLED_State == LOW && abCylinderLED1_State == HIGH){}
else{
         digitalWrite(vacummPressureLED, state);
         digitalWrite(vacummLED, state);
         delay(100); 
         digitalWrite(abCylinderLED1, !state);
         digitalWrite(abCylinderLED, !state);
          vacum_coil_bit = state;
      }  
}

void Vacam_Relay_Logic(){
  Vacam_bit_state = Pressure_SetBit;
  int temp_bit = !Vacam_bit_state;  

/*
RS485_serial.print("vac_timer_bit = ");  RS485_serial.print(vac_timer_bit) ;
RS485_serial.print(",operation_mode = ");  RS485_serial.print(operation_mode) ;
RS485_serial.print(",startStop_mode = ");  RS485_serial.println(startStop_mode) ;
*/
  
 if ( operation_mode == 1 && startStop_mode == 0)  {  
   digitalWrite(vacummPressureLED, LOW);
    vacum_coil_bit = 0;
 //  RS485_serial.print("return1 = ");
   return;
   } 
 else if (vac_timer_bit == 1){
    digitalWrite(vacummPressureLED, HIGH);
    vacum_coil_bit = 1;
 //   RS485_serial.print("return2 = ");
    return;
    }
 else {
     if ((error_percentage_bit  == 1) && (Pressure_relay_logic  == 2) )   {
     
     if (temp_bit ==1){ }
           digitalWrite(vacummPressureLED, temp_bit);  
           if(Vacam_fast_bit_state == 1 ) { digitalWrite(vacummLED, temp_bit);}
           vacum_coil_bit = temp_bit;    
          
       }
       else {
           digitalWrite(vacummPressureLED, LOW);
           digitalWrite(vacummLED, LOW);
           vacum_coil_bit = 1;
            }
      }
    //   RS485_serial.print("return3 = ");
 
  }

  
void Timer1_fun(){  
  // on time logic
   Button_Read();
   currentMillis = millis(); 
    wdi_time_reset(); 
 //    RS485_serial.print("Time_state = ");  RS485_serial.print((currentMillis - Off_Time_Presure_previousMillis)) ;RS485_serial.print(","); RS485_serial.println((Off_Time_Presure - 500)); RS485_serial.print(","); RS485_serial.println(Time_state); 
   
if (Time_state == 0 && ((currentMillis - Off_Time_Presure_previousMillis) >= (Off_Time_Presure - 500)) ){ 
    if (sensorRead1 ==0 ){ sensorRead(); }
     Pressure_relay();   
  //  RS485_serial.print("error_percentage = ");  RS485_serial.print(error_percentage) ;; RS485_serial.print(","); RS485_serial.print(Pressure_relay_logic); RS485_serial.print(","); RS485_serial.println(On_Time_Presure); RS485_serial.print(","); RS485_serial.println(Off_Time_Presure); 
  }
    
 if((((Pressure_relay_logic == 1)|| (Pressure_relay_logic == 2))&& ( operation_mode != 2)  && (error_percentage_bit == 1) && (Pressure_SetBit == HIGH)) && (currentMillis - Off_Time_Presure_previousMillis >= Off_Time_Presure))
  {
   // RS485_serial.print("Time_state on = ");  RS485_serial.print(currentMillis) ;
    Pressure_SetBit = LOW; 
    Time_state = 1;
    Pressure_relay_state( );
    Vacam_Relay_Logic();
    Off_Time_Presure_previousMillis = currentMillis;  
    Vac_mode_relay = 0;
    pid_error = 0;
    no_inlet_value_on++;
  }
    // off time logic
  else if ((((Pressure_relay_logic == 1)|| (Pressure_relay_logic == 2)) && ( operation_mode != 2) && (error_percentage_bit == 1) && (Pressure_SetBit == LOW)) && (currentMillis - Off_Time_Presure_previousMillis >= On_Time_Presure ))
  {
  // RS485_serial.print("Time_state off = ");  RS485_serial.print(currentMillis) ;
    Pressure_SetBit = HIGH; 
    Pressure_relay_state(); 
    Time_state = 0;
    Vacam_Relay_Logic();    
    Off_Time_Presure_previousMillis = currentMillis;    
  }
  
 }

 

//PID_calculatio();
void PID_calculation(){
long temp_Prev_time = 0;


if  (Pressure_SetBit == 0 &&  first_inc == 0){
   Prev_time   = millis();
   Prev_pressure = pressureSensorRaw;
   first_inc = 1;
   NUmperofstok++;
}
else if (Pressure_SetBit == 1 && first_inc == 1){
   After_time = millis(); 
}

else if (Pressure_SetBit == 0 && first_inc == 1){
  NUmperofstok++;
  temp_Prev_time = millis(); 
  After_pressure = pressureSensorRaw;
  defer_pressure = After_pressure - Prev_pressure;

defer_time = (After_time - Prev_time);

  if (defer_time > 0  && defer_pressure > 0 && defer_time <= 25000 && pressureSensorValue != 0){

  Actul_gain = defer_pressure/defer_time;
  Eestimated_gain = Actul_gain ;
 
  gain_config();

 correctionfactor_pressure_time  = default_gain + Extragainreq;
 if (correctionfactor_pressure_time > 1000){correctionfactor_pressure_time = 1000;}
 
  
 get_ondelay_time ();
 
  }
 
  else {
  correctionfactor_pressure_time = default_gain;

  }
  
 Prev_pressure = After_pressure;
 Prev_time = temp_Prev_time;
}

print_data();

}


void PID_clear(){

// if (pressureSensorValue == 0){
NUmperofstok = 0;
first_inc = 0;
After_time = 0;
Prev_time = 0;
Error_slot = 0 ;
NUmperofstok_1 =0;
NUmperofstok_3 =0;
NUmperofstok_2 =0;
NUmperofstok_4 =0;
NUmperofstok_5 =0;
NUmperofstok_6 =0;
Prev_pressure = pressureSensorRaw;
After_pressure = pressureSensorRaw;
correctionfactor_pressure_time = default_gain;
Extragainreq = 0;
// }
 get_ondelay_time();
}


void print_data(){

  if  (Pressure_SetBit == 0){
//     RS485_serial.print ("PresureT5:");RS485_serial.println (PresureT5);

//  RS485_serial.print ("NUmperofstok:");RS485_serial.println (NUmperofstok);
// // RS485_serial.print ("After_time:");RS485_serial.println (After_time);
//  RS485_serial.print ("defer_time:");RS485_serial.println (defer_time);


// // RS485_serial.print ("Prev_pressure:");RS485_serial.println (Prev_pressure);
// // RS485_serial.print ("After_pressure:");RS485_serial.println (After_pressure);
// // RS485_serial.print ("defer_pressure:");RS485_serial.println (defer_pressure);
// RS485_serial.print ("Error_slot:");RS485_serial.println (Error_slot);

//RS485_serial.print ("Actul_gain:");RS485_serial.println (Actul_gain);
// // RS485_serial.print ("Eestimated_gain:");RS485_serial.println (Eestimated_gain);
// RS485_serial.print ("correctionfactor_pressure_time:");RS485_serial.println(correctionfactor_pressure_time);
// RS485_serial.println ();
// RS485_serial.println ();
// RS485_serial.println ();
}

}


void gain_config(){

int temp_defer = 0;
int ref_defer = 100; //15

if (Eestimated_gain < 3) {Eestimated_gain = 3;}
else if (Eestimated_gain >= ref_defer ){Eestimated_gain = 6;}

switch (Error_slot)
{

  case 1:
  // RS485_serial.print ("1,");
  temp_defer = (ref_defer - Eestimated_gain)+ NUmperofstok_1 ;
  Extragainreq =   temp_defer * NUmperofstok_1;
  NUmperofstok_1++;
  NUmperofstok_1++;
  if (NUmperofstok_1 > 10) {NUmperofstok_1 = 10;}
  break;

  case 2:
  // RS485_serial.print ("2,");  
  temp_defer = (ref_defer - Eestimated_gain)+ NUmperofstok_2;
  Extragainreq =   temp_defer * NUmperofstok_2;
  NUmperofstok_2 = NUmperofstok_2+3;

  if (NUmperofstok_2 > 10) {NUmperofstok_2 = 10;}
  NUmperofstok_1 = NUmperofstok_2;
  break;

  case 3:
  temp_defer = (ref_defer - Eestimated_gain)+ NUmperofstok_3;
  Extragainreq =   temp_defer * NUmperofstok_3;
  NUmperofstok_3= NUmperofstok_3+2;
  if (NUmperofstok_3 > 7) {NUmperofstok_3 = 7;}
  break;

  case 4:
  temp_defer = (ref_defer - Eestimated_gain) + NUmperofstok_4;
  Extragainreq =   temp_defer * NUmperofstok_4;
  NUmperofstok_4=NUmperofstok_4+2;
  if (NUmperofstok_4 > 7) {NUmperofstok_4 = 7;}
  break;

 case 5:
  
  temp_defer = (ref_defer - Eestimated_gain) + NUmperofstok_5;
  Extragainreq =   temp_defer * NUmperofstok_5;
  NUmperofstok_5=NUmperofstok_5+2;
  if (NUmperofstok_5 > 7) {NUmperofstok_5 = 7;}
  break;

 case 6:
 temp_defer = (ref_defer - Eestimated_gain)+ NUmperofstok_6;
 Extragainreq =   temp_defer * NUmperofstok_6;
 NUmperofstok_6++;
 NUmperofstok_6++;
 if (NUmperofstok_6 > 10) {NUmperofstok_6 = 10;}
  break;


default:
  break;
}

if(Extragainreq < 0){
  Extragainreq = 0;
}


}




void Write_int_eeprom(unsigned int data,unsigned int add){
  
      byte  data_byteL = (data >> 8) & 0xff;
      byte  data_byteH = data & 0xff;
       RS485_serial.print("EEPROM->");   RS485_serial.print(data);          
          EEPROM.update(add, data_byteL);
          add = add + 1;
          delay(20);
          EEPROM.update(add, data_byteH);
          delay(10);
       RS485_serial.println("ok");
 }

unsigned int Read_int_eeprom(unsigned int add){
unsigned int read_value;
        byte data_byteL = EEPROM.read(add);
             add = add + 1;
             delay(10);
        byte data_byteH = EEPROM.read(add);
             read_value = data_byteH|data_byteL<<8;
             return read_value;
             delay(10);
 }
 
void writeString(char add,String data)
{
    int _size = data.length();
    int i;
    for(i=0;i<_size;i++)
    {
      EEPROM.update(add+i,data[i]);
      delay(2);
    }
    EEPROM.update(add+_size,'\0');   //Add termination null character for String Data

}
 
 
String read_String(char add)
{
    //int i;
    char data[100]; //Max 100 Bytes
    int len=0;
    unsigned char k;
    k=EEPROM.read(add);
    while(k != '\0' && len<500)   //Read until null character
    {    
      k=EEPROM.read(add+len);
      data[len]=k;
      len++;
    }
    data[len]='\0';
    return String(data);
}


// Shared scheduler tick, called at TIMER_TICK_HZ (~100Hz) by whichever
// board-specific peripheral is wired up in board_avr128.h / board_2560.h.
void Timer_tick() {
   Timer1_fun();

   if (vac_timer_SS ==1)
   {
    vac_timer_cnt++;
      if(vac_timer_cnt >= TIMER_TICK_HZ){
      flash();
      vac_timer_cnt =0;
      }
    }
}


 
void setup(void) {

String data = "Hello World coupuerg";
String data_tex = "Hello World coupuerg";
//##################################  Testing  ####################################/
//writeString(50, data);  //Address 10 and String type data
//data_tex  = "New Energy";
//writeString(200, data_tex);  //Address 10 and String type data
//data  = "Shareef: 9092751786/9092753786";
//writeString(50, data);  //Address 10 and String type data
////##################################  Testing  ####################################/
 
String inData = ""; 
unsigned int next_line = 1;

recivedData  = "";// "  ";
recivedData1 = "";//"HELP LINE : 9843889843";
recivedData2 = "";//" ";

char recieved = 0x0;
  // initialize serial communication at 9600 bits per second:
//   mySerial.begin(4800);
//  mySerial.println("Hello, world?");
  
  Board_Serial_Begin();
  RS485_serial.print("START V48: ");
  RS485_serial.print(__DATE__);
  RS485_serial.print(" ");
  RS485_serial.println(__TIME__);
 
 
//  hx711_setup();
//  hx711_read();

 correctionfactor = EEPROM.read(Caliberation_address);
 correctionfactor_pressure_time = EEPROM.read(pressure_time_address);
 correctionfactor_vac_time = EEPROM.read(vac_time_address);
   
 if (correctionfactor > 254) {correctionfactor = 104;}
 if (correctionfactor_pressure_time > 254) {correctionfactor_pressure_time = 90;}
 if (correctionfactor_vac_time > 254) {correctionfactor_vac_time = 80;}
 
 default_gain = correctionfactor_pressure_time;
 get_ondelay_time();

  PinModeConfig();
  buzzer_logic(1);
  u8g2.begin();
  wdi_time_reset();

#if defined(Debugmode)
RS485_serial.println("Debuge Mode enabled ->"); 
  
#else

    if(Prog_mode_State == 0 && config_mode_state == 0) { 
      TyreCount_value = 0;
      Write_int_eeprom(TyreCount_value,TyreCount_address);
     }

 while (Prog_mode_State == 0 && config_mode_state == 0) {
        sensorRead();
        Button_Read();
        draw_Calibiration();
        delay(50);
        wdi_time_reset();
        }
        get_ondelay_time ();
      
    if (Prog_mode_State == 0){Serial1.println("Please Enter Company Name: ");}

    
while (Prog_mode_State == 0 && config_mode_state == 1)
{
 while (RS485_serial.available() > 0)
    {
         recieved = RS485_serial.read();
        inData += recieved; 
        wdi_time_reset();  
       
        // Process message when new line character is recieved
        if (recieved == '\n' && next_line == 1)
        {
            data  = inData;
            writeString(50, data);  //Address 10 and String type data
             delay(10);
            recivedData = read_String(50);
            draw_init();
            inData = ""; // Clear recieved buffer
            next_line = 2 ; 
            recieved =0x0;
            inData = "";
            RS485_serial.println("Please Enter Contact Name/Numper 1 "); 
        }
         if (recieved == '\n' && next_line ==2)
        {
            data_tex  = inData;
            writeString(200, data_tex);  //Address 10 and String type data
             delay(10);
            recivedData1 = read_String(200);
            draw_init();
            inData = ""; // Clear recieved buffer
            next_line = 3 ; 
            recieved =0x0;
            RS485_serial.println("Please Enter Contact Name/Numper 2: "); 
        }
           if (recieved == '\n' && next_line ==3)
        {
            data_tex  = inData;
            writeString(350, data_tex);  //Address 10 and String type data
            delay(10);
            recivedData2 = read_String(350);
            draw_init();
            inData = ""; // Clear recieved buffer
            next_line = 1 ; 
            recieved = 0x0;
            RS485_serial.println("Please Enter Company Name: "); 
        }
    }

     wdi_time_reset();  
     Prog_mode_State = digitalRead(Prog_mode);
}  

  RS485_serial.println("Editmode exit"); 

   recivedData  = read_String(50);
   recivedData1 = read_String(200);
   recivedData2 = read_String(350);

#endif

recivedData =  "        COBURG";
recivedData1 = "  HELP LINE1 : 9790644999";
recivedData2 = "  HELP LINE2 : 9944011444";

  draw_init();

  TyreCount_value = Read_int_eeprom(TyreCount_address);
  setPoint = EEPROM.read(setPoint_address);

  if (setPoint > 254){setPoint = 30;}
  
  setPointBar = (float)setPoint/14.501;
  setPointKgf = (float)setPoint*0.07031; 

  abCylinderLED_SetBit = 1;
  Pressure_relay(); 
  Display_referesh(); 
  
  Timer_setup();
//  MsTimer2::set(1000, flash); // 1000ms period 
//  Timer1.initialize(100000);         // initialize timer1, and set a 100ms
//  Timer1.attachInterrupt(Timer1_fun);  // attaches callback() as a timer overflow interrupt


}


void loop(void) {
 loop_cnt++;
 //RS485_serial.print("loop_cnt = ");  RS485_serial.println(loop_cnt);
  wdi_time_reset();
  //Button_Read();
  if (eeprom_calib_pending){
    EEPROM.update(eeprom_calib_address, eeprom_calib_value);
    eeprom_calib_pending = 0;
  }
  TyreCount();
 vacumRelease();
 
   if (operation_mode == 0) { vac_timer_bit = 0;  Vac_mode_pressure = 0;  }

  if (operation_mode == 1 && endTimer == 1 && pressureSensorValue != 0 && startStop_mode == 0 )
  {
    startStop_mode = 1;
    vac_timer_bit  = 1;
  }

  if (operation_mode == 1 && startStop_mode == 1 && endTimer == 1){
      abCylinderLED_SetBit = 0;
      enter_mode = 0;
      setTimerOld = setTimer;
      endTimer = 0;
      Vacam_bit_state = 0;
      setBitTimer = 0;
//      MsTimer2::start();
      vac_timer_SS = 1;
      error_percentage_bit = 1;
      Vacam_mode_Relay_Logic(1);
      N2Purity();  
     }
     
   else if(operation_mode == 1 && startStop_mode == 0 ) {
//      MsTimer2::stop();
      vac_timer_bit = 0;
      setBitTimer = 0;
      abCylinderLED_SetBit = 1;
      setTimer = setTimerOld;
      Vacam_bit_state = 1;
      Vacam_mode_Relay_off_Logic(0);
      endTimer = 1; 
      startStop_mode_bit = 0;    
      }

 if(setBitTimer == 1 && startStop_mode == 1 && operation_mode == 1 ){
    abCylinderLED_SetBit = 1;
    setTimer = setTimerOld;
    TyreCount(); 
    Vac_mode_relay = 1;  
   } 
  currentMillis_abs = millis();  
   if (currentMillis_abs - previousMillis >= interval) {
    previousMillis = currentMillis_abs;
    N2Purity();  
  //  RS485_serial.print("N2Purity");  
    }
  currentMillis = millis(); 
  if ((currentMillis - previousMillis_Dis) >= 1000){
     loop_cnt = 0;
     previousMillis_Dis = currentMillis;
     Display_referesh();
//     RS485_serial.print("Run_time:"); RS485_serial.println(run_time);
     DataPrint();
     Display_offcnt++;
     if (pressureSensorValue >= 6) { Display_offcnt = 0; }
     if (Display_offcnt > 120) {
       Display_offcnt = 121;

       if (DIS_TYPE == 1) {
         digitalWrite(GLCD_LED, HIGH);
       } else {
         digitalWrite(GLCD_LED, LOW);
       }
     } else {

       if (DIS_TYPE == 1) {
         digitalWrite(GLCD_LED, LOW);
       } else {
         digitalWrite(GLCD_LED, HIGH);
       }
     }
    }
  
  }

void DataPrint(){
 
#if defined(Debugmode)
    RS485_serial.println();
    RS485_serial.print("No St = ");  RS485_serial.println(no_inlet_value_on);
    RS485_serial.print("CF = ");  RS485_serial.println(correctionfactor_pressure_time) ;
    RS485_serial.print("On_Time = ");  RS485_serial.println(On_Time_Presure) ;
    RS485_serial.print("ERROR % = ");  RS485_serial.print(error_percentage) ;
     RS485_serial.print(" ->");  RS485_serial.println(Error_slot) ;
    RS485_serial.print("default_gain = ");  RS485_serial.println(default_gain) ;
    RS485_serial.print("Extragainreq = ");  RS485_serial.println(Extragainreq) ;
  
    RS485_serial.print(PresureT1);RS485_serial.print(","); 
    RS485_serial.print(PresureT2);RS485_serial.print(","); 
    RS485_serial.print(PresureT3);RS485_serial.print(","); 
    RS485_serial.print(PresureT4);RS485_serial.print(","); 
    RS485_serial.print(PresureT5);RS485_serial.print(","); 
    RS485_serial.println(PresureT6);
#else

#endif

  }

  
