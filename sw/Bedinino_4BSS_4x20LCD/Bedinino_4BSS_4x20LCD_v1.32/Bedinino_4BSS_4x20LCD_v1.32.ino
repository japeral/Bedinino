
//
// Bedinino
//
// Bedinino is an electronic control unit for Bedini monopole motor/generator systems.
// It is intended for improve system COP (Coefficient Of Performance), and help other 
// people as a platform for start researching this Free Energy devices.
//
//
// You need:
//  - Arduino UNO clone.
//    Ebay: "NEW CH340G ATmega328P UNO R3 Board & Free USB Cable for Arduino DIY"
//        http://www.ebay.es/itm/141248769799 4.76€
//
//  - 1x Bedinino B4SS Shield.
//        http://www.ebay.es/itm/161449558208 17.26€
//
//  - 4 Lines, 20 character, LCD with I2C interface board.
//    Ebay: "2004 20x4 2004A Character LCD Display Module Blue Blacklight"
//        http://www.ebay.es/itm/131282437497 4.46€
//
//  - I2C serial interace for 2x16 4x20 LCD
//        http://www.ebay.es/itm/171450359303 1.26€
//
//
// To do:
//                   Add magnetometer function.
//                   Add instantaneous rotor tachometer RPM function.
//
// v1.32 2014-10-18  * Added elapsed time counter. Up to 49 day 12 hour (unsigned long counting ms rollover).
//                   * Added swap counter.
//
// v1.31 2014-10-18  * Added function for change total number of swapping batteries. From 2 to 4 in swapping_batteries_number constant (asked by Hugo from Asturias).
//
// v1.30 2014-10-12  * Added B4SS Shield compatibilitiy.
//                   * Primary and secondary analog calibration in hardware by potentiometer.
//            
// v1.21 2014-07-14  * Added EEPROM backup for nonvolatile batteries machine state.
//                   * Some improvements to batteies role display on LCD.
//
// v1.20 2014-07-05  * Added primary and secondary battery voltage adquisition with Arduino Vcc (Vref+) compensation 
//                     and voltage dividers attenuation calibration in hardware.
//                   * Added battery switch event by primary battery low voltage event and secondary battery 
//                     hi voltage event with at least 10s in each battery as primary.
//
// v1.10 2014-06-28  * Added 4x20 LCD for information display.
//                   * Added countdown to next battery switch.
//                   * Added status role of each battery on LCD display (primary=P or secondary=S). B1=P B2=S B3=S B4=S.
//
// v1.00 2014-06-23  Initial release, basic timed 4 battery switcher.
//
// by J.A.Peral
//
// Bitcoin donations accepted: 18hzkF2NgSPJAuPmpm9ZvVHRnnMsQypm5Z
//

//
// Pins definition
//
#define BAT1_R_SECONDARY_PIN   2
#define BAT1_S_PRIMARY_PIN     3
#define BAT2_R_SECONDARY_PIN   4
#define BAT2_S_PRIMARY_PIN     5
#define BAT3_R_SECONDARY_PIN   6
#define BAT3_S_PRIMARY_PIN     7
#define BAT4_R_SECONDARY_PIN   8
#define BAT4_S_PRIMARY_PIN     9

#define RELAYS_EN_PIN          10

#define MAGNETIC_ANALOG        A0      // A1302 Ratiometric Linear Hall efect magnetic sensor. Fix sensor to coil. Check Vout>2.5V with N magnetic flux.
#define VPRIMARY_ANALOG        A1      // Resistive voltage divider. R1 = 470K, POT=20K, R2= 100K.
#define VSECONDARY_ANALOG      A2      // Resistive voltage divider. R1 = 470K, POT=20K, R2= 100K.
#define UNUSED_ANALOG          A3
// NOTE: A4 and A5 are not avaliable because pins are used by I2C (LCD display)

//
// Bedinino peripherals enable
//
#define LCD                            // Enable LCD information display.
#define USE_SERIAL                     // Enable USE_SERIAL OUT to PC.

//
// Bedini assembly constants definition
//
#define MAGNETS_IN_ROTOR       4       // Set here the number of magnets attached to your rotor, needed to RPM calculations.

// Leave only one of the folloing  #defines enabled.
#define SWAP_BY_FIXED_TIME
//#define SWAP_BY_VOLTAGE_EVENT

#define BATTERIES_SWAP_TIME                      60000  // 5 minutes = 5*60*1000 = 300000ms
//#define BATTERIES_SWAP_TIME                    1200000 // 20 minutes = 20*60*1000 = 1200000ms
#define BATTERIES_HIGH_VOLTAGE_EVENT             28.00
#define BATTERIES_LOW_VOLTAGE_EVENT              24.00

const unsigned char swapping_batteries_number =  2;

//=========================================
// Libraries include
//
#include <EEPROM.h>

// 4x20 LCD Display
#if defined LCD
  #include <Wire.h>                         // I2C Library
  #include <LiquidCrystal_I2C.h>            // LCD DISPLAY Library
  LiquidCrystal_I2C lcd(0b00100001,20,4);   // A0=open, A1=closed, A2=closed. Set the LCD address to 0x20 for a 20 chars and 4 line display.
#endif

//
//==========================================

//
// Constants definition
//
#define PRIMARY                0
#define SECONDARY              1

enum{
  BAT1=1,
  BAT2,
  BAT3,
  BAT4
};

enum{
  PRIMARY_1_SET=0,
  PRIMARY_1_STAY,
  PRIMARY_2_SET,
  PRIMARY_2_STAY,
  PRIMARY_3_SET,
  PRIMARY_3_STAY,
  PRIMARY_4_SET,
  PRIMARY_4_STAY
};

typedef struct{
  int bat1;
  int bat2;
  int bat3;
  int bat4; 
}rpm;

//=============================================================
// Global variables
//

// Non volatile
struct{
  unsigned char batteries_state;
}nv;

// Volatile
int  swaps_counter=-1;
unsigned char old_batteries_state=0xFF;
unsigned long switchtimer=0;
unsigned long lcdrefresh=0;
unsigned long countdowntimer=0;
unsigned long timefromlastswap=0;
unsigned long swapsecondscountdown=0;
char message[22];

  //---------------------------------------------------------
  // Analog adquisition with smooth filtering (averaging).
  //
  const int numReadings = 32;       // Number of average samples.
  unsigned long adqvcctimer=0;      // Sample rate for Internal Vref and Vcc calculation.
  unsigned long adqiintimer=0;      // Sample rate for iin
  unsigned long adqvpstimer=0;      // Sample rate for Vp and Vs
  // Arduino Vcc power supply (A/D Vref+).
  long vcc_readings[numReadings];    
  long vcc_total=0;    
  long vcc_avg=0;    
  int vcc_index=0; 
  float fVcc;  
  float fmvpercount;
  // Vp = Primary battery voltage
  int vprim_readings[numReadings];      // the readings from the analog input
  int vprim_index = 0;                  // the index of the current reading
  int vprim_total = 0;                  // the running total
  int vprim_avg = 0;                    // the average
  float fVprim;
  // Vs = Secondary battery voltage
  int vsec_readings[numReadings];      // the readings from the analog input
  int vsec_index = 0;                  // the index of the current reading
  int vsec_total = 0;                  // the running total
  int vsec_avg = 0;                    // the average
  float fVsec;
  //---------------------------------------------------------

unsigned long adqmagtimer=0;
int mag;
//
//=================================================================

//
// Local functions
//
void BatSet(unsigned char battery, unsigned char side);
void PrimaryNoneSet(void);
void Vpsadq(void);
void Vccadq(void);
  
//  
// Setup loop
//
void setup(){
  
    // USE_SERIAL Setup.
    #if defined USE_SERIAL
      Serial.begin(9600);
    #endif
         
    // Latching relay control lines Setup
    pinMode(RELAYS_EN_PIN, OUTPUT);    
    pinMode(BAT1_R_SECONDARY_PIN, OUTPUT);
    pinMode(BAT1_S_PRIMARY_PIN, OUTPUT);
    pinMode(BAT2_R_SECONDARY_PIN, OUTPUT);
    pinMode(BAT2_S_PRIMARY_PIN, OUTPUT);
    pinMode(BAT3_R_SECONDARY_PIN, OUTPUT);
    pinMode(BAT3_S_PRIMARY_PIN, OUTPUT);
    pinMode(BAT4_R_SECONDARY_PIN, OUTPUT);
    pinMode(BAT4_S_PRIMARY_PIN, OUTPUT);
    pinMode(RELAYS_EN_PIN, OUTPUT);    
  
       
    // LCD Setup
    #if defined LCD  
      lcd.init();             // initialize the lcd 
      lcd.backlight(); 
      lcd.home();
      lcd.setCursor(2,0);      // Char pos 2, Line 0.
                                                // +0 flash bytes
      strcpy(message, "[Bedinino v1.32]");
      lcd.print(message);
      #if defined USE_SERIAL
      Serial.println(message);
      #endif
      
/*    lcd.print(F("[Bedinino v1.32]"));          // +144 flash bytes      
      #if defined USE_SERIAL
      Serial.println(F("[Bedinino v1.32]"));     //
      #endif
*/
      
      lcd.setCursor(0,1);    // Char pos 0, Line 1      
      strcpy(message, "Bedini Energizer ECU");
      lcd.print(message);
      #if defined USE_SERIAL
        Serial.println(message);
      #endif
      lcd.setCursor(0,2);    // Char pos 0, Line 2.   
      strcpy(message, "GTEL Research group.");
      lcd.print(message);
      #if defined USE_SERIAL
        Serial.println(message);
      #endif
      lcd.setCursor(4,3);    // Char pos 0, Line 3.            
      strcpy(message, "by J.A.Peral");
      lcd.print(message);
      #if defined USE_SERIAL
        Serial.println(message);
      #endif
    #endif
     
      delay(2000);      // Delay for welcome message display time.
           
    #if defined LCD
      lcd.clear();
      lcd.setCursor(0,0);      
      lcd.print("B1=");  
      if (swapping_batteries_number>1){
        lcd.setCursor(5,0);      
        lcd.print("B2=");      
      }
      if (swapping_batteries_number>2){      
        lcd.setCursor(10,0);      
        lcd.print("B3=");      
      }
      if (swapping_batteries_number>3){      
        lcd.setCursor(15,0);      
        lcd.print("B4=");                  
      }
    #endif
    
    // Clean up average readings int array.
    for (int thisReading = 0; thisReading < numReadings; thisReading++){
      vprim_readings[thisReading] = 0; 
      vsec_readings[thisReading] = 0;       
    }
    
    nv.batteries_state=EEPROM.read(0);   
    #if defined USE_SERIAL    
    Serial.print("BatteriesState:");
    Serial.print(nv.batteries_state);
    #endif
    
}
//end setup

// Main loop
void loop(){

  // Analog to Digital conversion adquisition and averaging tasks
  if(millis()-adqvcctimer>62){  // 32 samples @ 62ms/sample = 16sample/s. 2second/32 point averaging.
    adqvcctimer=millis();    
    Vccadq();    
  }     
  if(millis()-adqvpstimer>62){
    adqvpstimer=millis();
    Vpsadq();
  }

  // Battery switcher state machine.
  switch(nv.batteries_state){
    default:                // If uninitialized, default sets PRIMARY_1_SET
    case PRIMARY_1_SET:    
      PrimaryNoneSet();
      #if defined USE_SERIAL
        Serial.println("Primary_1_SET");
      #endif      
      BatSet(BAT1, PRIMARY);
      BatSet(BAT2, SECONDARY);      
      BatSet(BAT3, SECONDARY);      
      BatSet(BAT4, SECONDARY);
      nv.batteries_state=PRIMARY_1_STAY;
      timefromlastswap=millis();     
      swaps_counter++;
      break;
    case PRIMARY_1_STAY:
      #if defined SWAP_BY_FIXED_TIME
      if(millis()-switchtimer>BATTERIES_SWAP_TIME){
      #endif
      #if defined SWAP_BY_VOLTAGE_EVENT
      if((millis()-timefromlastswap)>10000 && (fVsec > BATTERIES_HIGH_VOLTAGE_EVENT || fVprim < BATTERIES_LOW_VOLTAGE_EVENT)){
      #endif     
        switchtimer=millis();
        if(swapping_batteries_number>=2){
          nv.batteries_state=PRIMARY_2_SET;
        }else{
          nv.batteries_state=PRIMARY_1_SET;          
        }
        EEPROM.write(0, nv.batteries_state);
      }
/*      
      #if defined  BATTERIES_STOP_VOLTAGE
        if (millis()>10000 && vsfloat < BATTERIES_STOP_VOLTAGE)  PrimaryNoneSet();
      #endif        
*/      
      break;
      
    case PRIMARY_2_SET:
      PrimaryNoneSet();   
      #if defined USE_SERIAL
        Serial.println("Primary_2_SET");
      #endif          
      BatSet(BAT1, SECONDARY);
      BatSet(BAT2, PRIMARY);
      BatSet(BAT3, SECONDARY);
      BatSet(BAT4, SECONDARY);
      nv.batteries_state=PRIMARY_2_STAY;   
      timefromlastswap=millis();            
      swaps_counter++;      
      break;
    case PRIMARY_2_STAY:
      #if defined SWAP_BY_FIXED_TIME
      if(millis()-switchtimer>BATTERIES_SWAP_TIME){
      #endif
      #if defined SWAP_BY_VOLTAGE_EVENT
      if((millis()-timefromlastswap)>10000 && (fVsec > BATTERIES_HIGH_VOLTAGE_EVENT || fVprim < BATTERIES_LOW_VOLTAGE_EVENT)){
      #endif     
        switchtimer=millis();
        if(swapping_batteries_number>=3){
          nv.batteries_state=PRIMARY_3_SET;
        }else{
          nv.batteries_state=PRIMARY_1_SET;          
        }
        EEPROM.write(0, nv.batteries_state);        
      }            
/*      
      #if defined  BATTERIES_STOP_VOLTAGE
        if (millis()>10000 && vsfloat < BATTERIES_STOP_VOLTAGE)  PrimaryNoneSet();
      #endif       
*/      
      break;      
      
    case PRIMARY_3_SET:
      PrimaryNoneSet();
      #if defined USE_SERIAL
        Serial.println("Primary_3_SET");
      #endif          
      BatSet(BAT1, SECONDARY);
      BatSet(BAT2, SECONDARY);
      BatSet(BAT3, PRIMARY);
      BatSet(BAT4, SECONDARY);    
      nv.batteries_state=PRIMARY_3_STAY;
      timefromlastswap=millis();        
      swaps_counter++;      
      break;
    case PRIMARY_3_STAY:
      #if defined SWAP_BY_FIXED_TIME
      if(millis()-switchtimer>BATTERIES_SWAP_TIME){
      #endif
      #if defined SWAP_BY_VOLTAGE_EVENT
      if((millis()-timefromlastswap)>10000 && (fVsec > BATTERIES_HIGH_VOLTAGE_EVENT || fVprim < BATTERIES_LOW_VOLTAGE_EVENT)){
      #endif     
        switchtimer=millis();
        if(swapping_batteries_number>=4){
          nv.batteries_state=PRIMARY_4_SET;
        }else{
          nv.batteries_state=PRIMARY_1_SET;          
        }
        EEPROM.write(0, nv.batteries_state);        
      }
/*      
      #if defined  BATTERIES_STOP_VOLTAGE
        if (millis()>10000 && vsfloat < BATTERIES_STOP_VOLTAGE)  PrimaryNoneSet();
      #endif              
*/
      break;            
      
    case PRIMARY_4_SET:
      PrimaryNoneSet();       
      #if defined USE_SERIAL
        Serial.println("Primary_4_SET");
      #endif          
      BatSet(BAT1, SECONDARY);
      BatSet(BAT2, SECONDARY);
      BatSet(BAT3, SECONDARY);
      BatSet(BAT4, PRIMARY);
      nv.batteries_state=PRIMARY_4_STAY;  
      timefromlastswap=millis();         
      swaps_counter++;      
      break;
    case PRIMARY_4_STAY:
      #if defined SWAP_BY_FIXED_TIME
      if(millis()-switchtimer>BATTERIES_SWAP_TIME){
      #endif
      #if defined SWAP_BY_VOLTAGE_EVENT
      if((millis()-timefromlastswap)>10000 && (fVsec > BATTERIES_HIGH_VOLTAGE_EVENT || fVprim < BATTERIES_LOW_VOLTAGE_EVENT)){
      #endif     
        switchtimer=millis();
        nv.batteries_state=PRIMARY_1_SET;
        EEPROM.write(0, nv.batteries_state);
      }            
/*      
      #if defined  BATTERIES_STOP_VOLTAGE
        if (millis()>10000 && vsfloat < BATTERIES_STOP_VOLTAGE)  PrimaryNoneSet();
      #endif      
*/
      break;
  }
  //end switch(batteries_state)
 

// LCD Refresh
#if defined LCD
  if(millis()-lcdrefresh>666){
    lcdrefresh=millis();

    // Batteries role display refresh
    switch(nv.batteries_state){
    case PRIMARY_1_STAY:  
      lcd.setCursor(3,0);  lcd.print("P");
      if(swapping_batteries_number>=2){ lcd.setCursor(8,0);  lcd.print("S"); }
      if(swapping_batteries_number>=3){ lcd.setCursor(13,0); lcd.print("S"); }
      if(swapping_batteries_number>=4){ lcd.setCursor(18,0); lcd.print("S"); }
      break;
    case PRIMARY_2_STAY:
      lcd.setCursor(3,0);  lcd.print("S");
      if(swapping_batteries_number>=2){ lcd.setCursor(8,0);  lcd.print("P"); }
      if(swapping_batteries_number>=3){ lcd.setCursor(13,0); lcd.print("S"); }
      if(swapping_batteries_number>=4){ lcd.setCursor(18,0); lcd.print("S"); }          
      break;
    case PRIMARY_3_STAY:
      lcd.setCursor(3,0);  lcd.print("S");
      if(swapping_batteries_number>=2){ lcd.setCursor(8,0);  lcd.print("S"); }
      if(swapping_batteries_number>=3){ lcd.setCursor(13,0); lcd.print("P"); }
      if(swapping_batteries_number>=4){ lcd.setCursor(18,0); lcd.print("S"); }            
      break;
    case PRIMARY_4_STAY:
      lcd.setCursor(3,0);  lcd.print("S");
      if(swapping_batteries_number>=2){ lcd.setCursor(8,0);  lcd.print("S"); }
      if(swapping_batteries_number>=3){ lcd.setCursor(13,0); lcd.print("S"); }
      if(swapping_batteries_number>=4){ lcd.setCursor(18,0); lcd.print("P"); }            
      break;
   default: 
      lcd.setCursor(3,0);  lcd.print(" ");
      lcd.setCursor(8,0);  lcd.print(" ");
      lcd.setCursor(13,0); lcd.print(" ");
      lcd.setCursor(18,0); lcd.print(" ");    
      break;
    }     

    // Vp and Vs translation from averaged 10-bit counts to mV (with Arduino Vcc (Vref+) compensation).
    fVcc = (float)vcc_avg;
    fmvpercount = fVcc / 1024;
    fVprim= (float)vprim_avg * fmvpercount;
    fVsec= (float)vsec_avg * fmvpercount;
   
    // Voltage divider attenuation with potentiometer at center position is:
    //    R1 = 470K, POT=20K, R2= 100K -> attenuation = 110k/(480k+110k)=0.1864
    fVprim= fVprim / 0.1864 / 1000;  // divide by 1000 to convert mV to V.
    fVsec= fVsec / 0.1864 / 1000;

    static unsigned long blink_on_event=0;    
    static unsigned char event_message=0;
    if(millis()-blink_on_event>500){
      blink_on_event=millis();
      event_message^=1; // toggle value 0->1, 1->0.
    }
    lcd.setCursor(0,1);
    dtostrf(fVprim,5,3,message); //
    strcat(message, "Vp");
    if(fVprim < BATTERIES_LOW_VOLTAGE_EVENT && event_message){
      strcpy(message, " LOW ");
    }
    if(fVprim > BATTERIES_HIGH_VOLTAGE_EVENT && event_message){
      strcpy(message, "HIGH ");
    }
    lcd.print(message);     
    lcd.setCursor(10,1);
    dtostrf(fVsec,5,3,message); //      
    strcat(message, "Vs");
    if(fVsec < BATTERIES_LOW_VOLTAGE_EVENT && event_message){
      strcpy(message, " LOW ");
    }
    if(fVsec > BATTERIES_HIGH_VOLTAGE_EVENT && event_message){
      strcpy(message, "HIGH ");
    }
    lcd.print(message);
  }
  //end if(millis()-lcdrefresh>500)

  #if defined SWAP_BY_FIXED_TIME
  if(millis()-countdowntimer>1000){
    countdowntimer=millis();
    swapsecondscountdown--;    
    lcd.setCursor(14,2);    
    lcd.print("     ");
    lcd.setCursor(14,2);
    dtostrf(swapsecondscountdown,5,0,message); //
    strcat(message,"s");
    lcd.print(message);    
  }
  #endif


  lcd.setCursor(0,2);    
  sprintf(message, "Swaps:%04d", swaps_counter);
  lcd.print(message);


  // Elapsed time counter
  lcd.setCursor(0,3);    
  PrintElapsedTime(millis());
  
}
#endif  
// end LCD_REFRESH

//end loop

//-------------------------------------------------------------
// Auxiliary functions
//
void PrintElapsedTime(unsigned long timeNow){
  
 unsigned long day = 86400000; // 86400000 milliseconds in a day
 unsigned long hour = 3600000; // 3600000 milliseconds in an hour
 unsigned long minute = 60000; // 60000 milliseconds in a minute
 unsigned long second =  1000; // 1000 milliseconds in a second

 unsigned long days, hours, minutes, seconds;
 
 days= timeNow / day ;                                   // number of days
 hours = (timeNow % day) / hour;                         // the remainder from days division (in milliseconds) divided by hours, this gives the full hours
 minutes = ((timeNow % day) % hour) / minute ;           // and so on...
 seconds = (((timeNow % day) % hour) % minute) / second;
 sprintf(message, "Elap:%02uD.%02uH:%02uM:%02uS", (int)days, (int)hours, (int)minutes, (int)seconds);
 lcd.print(message);
}

//---

void BatSet(unsigned char battery, unsigned char side){
  
  int pin;

  if (side==PRIMARY){ 
    if(battery == BAT1) pin=BAT1_S_PRIMARY_PIN;
    if(battery == BAT2) pin=BAT2_S_PRIMARY_PIN;
    if(battery == BAT3) pin=BAT3_S_PRIMARY_PIN;
    if(battery == BAT4) pin=BAT4_S_PRIMARY_PIN;
  }else{
    if(battery == BAT1) pin=BAT1_R_SECONDARY_PIN;
    if(battery == BAT2) pin=BAT2_R_SECONDARY_PIN;
    if(battery == BAT3) pin=BAT3_R_SECONDARY_PIN;
    if(battery == BAT4) pin=BAT4_R_SECONDARY_PIN;
  }
  
  digitalWrite(RELAYS_EN_PIN, HIGH); 
 
  digitalWrite(pin, HIGH);    
  delay(80);
  digitalWrite(pin, LOW);      
  delay(80);      
  
 digitalWrite(RELAYS_EN_PIN, LOW); 

#if defined  SWAP_BY_FIXED_TIME
  swapsecondscountdown=BATTERIES_SWAP_TIME/1000; // Recharge time to next swap in counter (seconds)
#endif

}

void PrimaryNoneSet(void){
   Serial.println("Primary_None_SET");          
   BatSet(BAT1, SECONDARY);
   BatSet(BAT2, SECONDARY);      
   BatSet(BAT3, SECONDARY);      
   BatSet(BAT4, SECONDARY);
}

void Vpsadq(void){
    // subtract the last reading:
    vprim_total= vprim_total - vprim_readings[vprim_index];         
    vsec_total= vsec_total - vsec_readings[vsec_index];             
    // read from the sensor:  
    vprim_readings[vprim_index] = analogRead(VPRIMARY_ANALOG);
    delay(1);
    vsec_readings[vsec_index] = analogRead(VSECONDARY_ANALOG);
    delay(1);
    // add the reading to the total:
    vprim_total= vprim_total + vprim_readings[vprim_index];       
    vsec_total= vsec_total + vsec_readings[vsec_index];           
    // advance to the next position in the array:  
    vprim_index++;
    vsec_index++;    

    // if we're at the end of the array...
    if (vprim_index >= numReadings)              
      // ...wrap around to the beginning: 
      vprim_index = 0;                           

    if (vsec_index >= numReadings)              
      // ...wrap around to the beginning: 
      vsec_index = 0;                           
      
    // calculate the average:
    vprim_avg = vprim_total / numReadings;
    vsec_avg = vsec_total / numReadings;  
}


void Vccadq(void){    
    // subtract the last reading:    
    vcc_total= vcc_total - vcc_readings[vcc_index];         
    // read from the sensor:  
    vcc_readings[vcc_index] = readVcc(); 
    // add the reading to the total:
    vcc_total= vcc_total + vcc_readings[vcc_index];       
    // advance to the next position in the array:  
    vcc_index++;

    // if we're at the end of the array...
    if(vcc_index >= numReadings)              
      // ...wrap around to the beginning: 
      vcc_index = 0;                           

    // calculate the average:
    vcc_avg = vcc_total / numReadings;           
}

//-----------------------------------------------------
// Aux Functions
//
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
//
//  Adjust literal coeficient to match VCC to your digital multimeter reading voltage btw +5V and GND.
//
//  result = 1125300L / result; // Calculate Vcc (in mV); 1.1    *1023*1000
//  result = 1135530L / result; // Calculate Vcc (in mV); 1.11   *1023*1000
//  result = 1145760L / result; // Calculate Vcc (in mV); 1.12   *1023*1000  
    result = 1150464L / result; // Calculate Vcc (in mV); 1.1235 *1023*1000  
//  result = 1149852L / result; // Calculate Vcc (in mV); 1.124  *1023*1000  
//  result = 1150875L / result; // Calculate Vcc (in mV); 1.125  *1023*1000  
//  result = 1155990L / result; // Calculate Vcc (in mV); 1.13   *1023*1000  
  return result; // Vcc in millivolts
}

