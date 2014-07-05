
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
//          http://www.ebay.es/itm/141248769799?ssPageName=STRK:MEWNX:IT&_trksid=p3984.m1497.l2649
//
//  - 4x Latching relay SPDT 8A 5V.
//    Ebay: "Latching/Bistable Relay module 5V, Relay board for AVR, PIC, Arduino, -from EU"
//          http://www.ebay.es/itm/171178593872?ssPageName=STRK:MEWNX:IT&_trksid=p3984.m1497.l2649
//
//  - 4 Lines, 20 character, LCD with I2C interface board. ""
//    Ebay: "2004 20x4 2004A Character LCD Display Module Blue Blacklight"
//          http://www.ebay.es/itm/2004-20x4-2004A-Character-LCD-Display-Module-Blue-Blacklight-/281317730016?pt=LH_DefaultDomain_0&hash=item417fd7e6e0&_uhb=1
//
//  - 2x 220K 1% 1/4W resistor
//  - 2x 100K 1% 1/4W resistor
//
// To do:
//                   Add magnetometer function.
//                   Add instantaneous rotor tachometer RPM function.
//            
// v1.20 2014-07-05  * Added primary and secondary battery voltage adquisition with Arduino Vcc (Vref+) compensation 
//                     and voltage dividers attenuation calibration in soft.
//                   * Added primary current draw using ACS712-5A current (hall effect) sensor. (need some improvements
//                     to avoid unstable reading).
//                   * Added battery switch event by primary battery low voltage event (11.50V) and secondary battery 
//                     hi voltage event (13.00V) with at least 10s in each battery as primary.
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

#define IINPUT_ANALOG          A0      // Allegro ACS712 - 5A bipolar Hall efect current sensor
#define VPRIMARY_ANALOG        A1      // Resistive voltage divider. Attenuation is 3.2V/V (16V max/ 5Vmax = 3.2V/V), R1 = 220K 1% 1/4w, R2= 100K 1% 1/4w.
#define VSECONDARY_ANALOG      A2      // Resistive voltage divider. Attenuation is 3.2V/V (16V max/ 5Vmax = 3.2V/V), R1 = 220K 1% 1/4w, R2= 100K 1% 1/4w.
#define MAGNETIC_ANALOG        A3      // A1302 Ratiometric Linear Hall efect magnetic sensor. Fix sensor to coil. Check Vout>2.5V with N magnetic flux.
                                       // NOTE: A4 and A5 are not avaliable because pins are used by I2C (LCD display)

//
// Bedinino peripherals enable
//
#define LCD                           // Enable LCD information display.
//#define USE_SERIAL                  // Enable USE_SERIAL OUT to PC.

//
// Bedini assembly constants definition
//
#define MAGNETS_IN_ROTOR       4       // Set here the number of magnets attached to your rotor, needed to RPM calculations.

// Leave only one of the folloing  #defines enabled.
//#define SWAP_BY_FIXED_TIME
#define SWAP_BY_VOLTAGE_EVENT

#define BATTERIES_SWAP_TIME          300000  // 5 minutes = 5*60*1000 = 300000ms
//#define BATTERIES_SWAP_TIME        1200000 // 20 minutes = 20*60*1000 = 1200000ms
#define BATTERIES_HIGH_VOLTAGE_EVENT 13.00
#define BATTERIES_LOW_VOLTAGE_EVENT  11.50

//=========================================
// Libraries include
//

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
unsigned long switchtimer=0;
unsigned long lcdrefresh=0;
unsigned char batteries_state=PRIMARY_1_SET;
unsigned long countdowntimer=0;
unsigned long timefromlastswap=0;
unsigned long swapsecondscountdown=0;
char message[22];

  //---------------------------------------------------------
  // Analog adquisition with smooth filtering (averaging).
  //
  const int numReadings = 10;       // Number of average samples.
  unsigned long adqvcctimer=0;      // Sample rate for Internal Vref and Vcc calculation.
  unsigned long adqiintimer=0;      // Sample rate for iin
  unsigned long adqvpstimer=0;      // Sample rate for Vp and Vs
  // Arduino Vcc power supply (A/D Vref+).
  long vccreadings[numReadings];    
  long vcctotal=0;    
  long vccaverage=0;    
  int vccindex=0; 
  float vccfloat;  
  float mvpercountfloat;
  // Iin = Instantaneous input current from primary battery.
  int iinzero;
  int iinreadings[numReadings];     // the readings from the analog input
  int iinindex = 0;                 // the index of the current reading
  int iintotal = 0;                 // the running total
  int iinaverage = 0;               // the average
  float iinfloat;
  // Vp = Primary battery voltage
  int vpreadings[numReadings];      // the readings from the analog input
  int vpindex = 0;                  // the index of the current reading
  int vptotal = 0;                  // the running total
  int vpaverage = 0;                // the average
  float vpfloat;
  // Vs = Secondary battery voltage
  int vsreadings[numReadings];      // the readings from the analog input
  int vsindex = 0;                  // the index of the current reading
  int vstotal = 0;                  // the running total
  int vsaverage = 0;                // the average
  float vsfloat;
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
void Iinadq(void);
void Vpsadq(void);
void Vccadq(void);
  
//  
// Setup loop
//
void setup(){
    PrimaryNoneSet();
     
    // USE_SERIAL Setup.
    #if defined USE_SERIAL
      Serial.begin(9600);
    #endif
         
    // Latching relay control lines Setup
    pinMode(BAT1_R_SECONDARY_PIN, OUTPUT);
    pinMode(BAT1_S_PRIMARY_PIN, OUTPUT);
    pinMode(BAT2_R_SECONDARY_PIN, OUTPUT);
    pinMode(BAT2_S_PRIMARY_PIN, OUTPUT);
    pinMode(BAT3_R_SECONDARY_PIN, OUTPUT);
    pinMode(BAT3_S_PRIMARY_PIN, OUTPUT);
    pinMode(BAT4_R_SECONDARY_PIN, OUTPUT);
    pinMode(BAT4_S_PRIMARY_PIN, OUTPUT);
       
    // LCD Setup
    #if defined LCD  
      lcd.init();             // initialize the lcd 
      lcd.backlight(); 
      lcd.home();
      lcd.setCursor(2,0);      // Char pos 2, Line 0.
      strcpy(message, "[Bedinino v1.20]");
      lcd.print(message);
      #if defined USE_SERIAL
        Serial.println(message);
      #endif
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
     
      delay(2000);      // Delay for welcome message display time & analog channels stabilization before zeros calibration.
      
      analogRead(IINPUT_ANALOG);           // Sure SAR A/D Input capacitor charged to channel voltage.
      for (int i=0; i<200; i++){
        iinzero= analogRead(IINPUT_ANALOG);  // Adquire current voltage in output sensor as 0 mA.        
        delay(2);                            // Delay beetween samples.                                        
      }

//    iinzero=512;
      
    #if defined LCD
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(iinzero);
      delay(2000);
    #endif      
      
    #if defined LCD
      lcd.clear();
      lcd.setCursor(5,0);
      lcd.print("Vp");
      
      lcd.setCursor(5,1);
      lcd.print("Vs");
      lcd.setCursor(12,1);
      lcd.print("Mm");
      
      lcd.setCursor(0,2);      
      lcd.print("B1=");      
      lcd.setCursor(5,2);      
      lcd.print("B2=");      
      lcd.setCursor(10,2);      
      lcd.print("B3=");      
      lcd.setCursor(15,2);      
      lcd.print("B4=");                  
    #endif
    
    // Clean up average readings int array.
    for (int thisReading = 0; thisReading < numReadings; thisReading++){
      iinreadings[thisReading] = 0; 
      vpreadings[thisReading] = 0; 
      vsreadings[thisReading] = 0;       
    }
    
}
//end setup

// Main loop
void loop(){

/*
  // Syncronize with magnet incoming to coil. 
  // (Be sure you properly place labeled A123 Hall effect sensor side to coil, and back to North pole magnets).
  while(analogRead(MAGNETIC_ANALOG) < 600  || (millis()-adqmagtimer>1000)){

    // If wait for magnetic analog times out, currentrpm=0, and continues for no blocking main loop.
    if(millis()-adqmagtimer>1000){
      adqmagtimer=millis();
      currentrpm=0;
    } // If no timeout, rotor is spinning, and then we can compute RPM speed.
    else{
        
    }

  } 
*/

  // Analog to Digital conversion adquisition and averaging tasks
  if(millis()-adqvcctimer>50){
    adqvcctimer=millis();    
    Vccadq();    
  }     
  if(millis()-adqiintimer>50){
    adqiintimer=millis();
    Iinadq();
  }
  if(millis()-adqvpstimer>50){
    adqvpstimer=millis();
    Vpsadq();
  }

  // Battery switcher state machine.
  switch(batteries_state){
    default:    
    case PRIMARY_1_SET:    
      PrimaryNoneSet();
      #if defined USE_SERIAL
        Serial.println("Primary_1_SET");
      #endif      
      BatSet(BAT1, PRIMARY);
      BatSet(BAT2, SECONDARY);      
      BatSet(BAT3, SECONDARY);      
      BatSet(BAT4, SECONDARY);
      batteries_state=PRIMARY_1_STAY;
      timefromlastswap=millis();      
      break;
    case PRIMARY_1_STAY:
      #if defined SWAP_BY_FIXED_TIME
      if(millis()-switchtimer>BATTERIES_SWAP_TIME){
      #endif
      #if defined SWAP_BY_VOLTAGE_EVENT
      if((millis()-timefromlastswap)>10000 && (vsfloat > BATTERIES_HIGH_VOLTAGE_EVENT || vpfloat < BATTERIES_LOW_VOLTAGE_EVENT)){
      #endif     
        switchtimer=millis();
        batteries_state=PRIMARY_2_SET;
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
      batteries_state=PRIMARY_2_STAY;   
      timefromlastswap=millis();            
      break;
    case PRIMARY_2_STAY:
      #if defined SWAP_BY_FIXED_TIME
      if(millis()-switchtimer>BATTERIES_SWAP_TIME){
      #endif
      #if defined SWAP_BY_VOLTAGE_EVENT
      if((millis()-timefromlastswap)>10000 && (vsfloat > BATTERIES_HIGH_VOLTAGE_EVENT || vpfloat < BATTERIES_LOW_VOLTAGE_EVENT)){
      #endif     
        switchtimer=millis();
        batteries_state=PRIMARY_3_SET;
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
      batteries_state=PRIMARY_3_STAY;
      timefromlastswap=millis();            
      break;
    case PRIMARY_3_STAY:
      #if defined SWAP_BY_FIXED_TIME
      if(millis()-switchtimer>BATTERIES_SWAP_TIME){
      #endif
      #if defined SWAP_BY_VOLTAGE_EVENT
      if((millis()-timefromlastswap)>10000 && (vsfloat > BATTERIES_HIGH_VOLTAGE_EVENT || vpfloat < BATTERIES_LOW_VOLTAGE_EVENT)){
      #endif     
        switchtimer=millis();
        batteries_state=PRIMARY_4_SET;
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
      batteries_state=PRIMARY_4_STAY;  
      timefromlastswap=millis();            
      break;
    case PRIMARY_4_STAY:
      #if defined SWAP_BY_FIXED_TIME
      if(millis()-switchtimer>BATTERIES_SWAP_TIME){
      #endif
      #if defined SWAP_BY_VOLTAGE_EVENT
      if((millis()-timefromlastswap)>10000 && (vsfloat > BATTERIES_HIGH_VOLTAGE_EVENT || vpfloat < BATTERIES_LOW_VOLTAGE_EVENT)){
      #endif     
        switchtimer=millis();
        batteries_state=PRIMARY_1_SET;
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

    lcd.setCursor(0,3);
    lcd.print("Vcc=        ");
    lcd.setCursor(4,3);
    vccfloat=vccaverage;
    dtostrf(vccfloat,5,1,message); //
    strcat(message,"mV");
    lcd.print(message);  

    // Convert to milli amps
    float temp;
//  iinaverage=549; // DEBUG: force 1000mA
    temp=iinaverage;
    iinfloat  = 26.394 * temp - 13514;        // ACS712-5Amp, 185mV/A counts to mA conversion.

    // Plot on LCD
    lcd.setCursor(8,0);
    lcd.print("         ");
    lcd.setCursor(8,0);
    dtostrf(iinfloat,3,0,message); //
    strcat(message,"mA");
    lcd.print(message);  
    #if defined USE_SERIAL
      Serial.print(message);
      Serial.print("-");
      Serial.println(iinaverage);
    #endif      
        
    // Plot on LCD VP & VS
    
/*  // DEPRECATED. NO Vcc (Vref+) compensation neither calibration.
    // Map 10bit ADC 0-1023 counts range to 0-16000mV (16V). Calibrate potentiometer to match reading.
    vpfloat=map(vpaverage ,  0, 1023,  0,   16000);  // Primary voltage counts to mv conversion 
    vpfloat=vpfloat/1000;                            // Convert from mV to V.
    vsfloat=map(vsaverage ,  0, 1023,  0,   16000);  // Secondary voltage counts to mv conversion
    vsfloat=vsfloat/1000;                            // Convert from mV to V.        
*/  
    // NEW.
    // Vp and Vs translation from averaged 10-bit counts to mV (with Arduino Vcc (Vref+) compensation).
    mvpercountfloat = vccfloat / 1024;
    vpfloat= (float)vpaverage * mvpercountfloat;
    vsfloat= (float)vsaverage * mvpercountfloat;
// Uncoment and download to proceed with voltage divider calibration.
//#define CALIBRATION_MODE
// Ideal voltage divider attenuation is 3.2V/V (16V max/ 5Vmax = 3.2V/V), R1 = 220K 1% 1/4w, R2= 100K 1% 1/4w.
    #if not defined CALIBRATION_MODE       // You need a digital multimeter reading Primary (Vp) and secondary (Vs) voltage batteries to proceed calibration.
      vpfloat= vpfloat * 12.00 / 3750.00;  // Replace 12.00 with multimeter voltage reading on primary bat. Replace 37500.00 with LCD second line reading.
      vsfloat= vsfloat * 12.00 / 3750.00;  // Replace 12.00 with multimeter voltage reading on recondary bat. Replace 37500.00 with LCD third line reading.
    #endif

    static unsigned long blink_on_event=0;    
    static unsigned char event_message=0;
    if(millis()-blink_on_event>500){
      blink_on_event=millis();
      event_message^=1; // toggle value 0->1, 1->0.
    }
    lcd.setCursor(0,0);
    dtostrf(vpfloat,5,2,message); //
    if(vpfloat < BATTERIES_LOW_VOLTAGE_EVENT && event_message){
      strcpy(message, " LOW ");
    }
    if(vpfloat > BATTERIES_HIGH_VOLTAGE_EVENT && event_message){
      strcpy(message, "HIGH ");
    }
    lcd.print(message);     
    lcd.setCursor(0,1);
    dtostrf(vsfloat,5,2,message); //      
    if(vsfloat < BATTERIES_LOW_VOLTAGE_EVENT && event_message){
      strcpy(message, " LOW ");
    }
    if(vsfloat > BATTERIES_HIGH_VOLTAGE_EVENT && event_message){
      strcpy(message, "HIGH ");
    }
    lcd.print(message);
  }
  //end if(millis()-lcdrefresh>500)

  #if defined SWAP_BY_FIXED_TIME
  if(millis()-countdowntimer>1000){
    countdowntimer=millis();
    swapsecondscountdown--;    
    lcd.setCursor(15,1);    
    lcd.print("    ");
    lcd.setCursor(15,1);
    dtostrf(swapsecondscountdown,4,0,message); //
    strcat(message,"s");
    lcd.print(message);    
  }
  #endif
 
}
#endif
//end loop

//-------------------------------------------------------------
// Auxiliary functions
//
void BatSet(unsigned char battery, unsigned char side){
  
  int pin;

#if defined LCD
  if (side==PRIMARY){ 
    if(battery == BAT1){ pin=BAT1_S_PRIMARY_PIN;   lcd.setCursor(3,2);  lcd.print("P"); }
    if(battery == BAT2){ pin=BAT2_S_PRIMARY_PIN;   lcd.setCursor(8,2);  lcd.print("P"); }
    if(battery == BAT3){ pin=BAT3_S_PRIMARY_PIN;   lcd.setCursor(13,2); lcd.print("P"); }
    if(battery == BAT4){ pin=BAT4_S_PRIMARY_PIN;   lcd.setCursor(18,2); lcd.print("P"); }
  }else{
    if(battery == BAT1){ pin=BAT1_R_SECONDARY_PIN; lcd.setCursor(3,2);  lcd.print("S"); }
    if(battery == BAT2){ pin=BAT2_R_SECONDARY_PIN; lcd.setCursor(8,2);  lcd.print("S"); }
    if(battery == BAT3){ pin=BAT3_R_SECONDARY_PIN; lcd.setCursor(13,2); lcd.print("S"); }
    if(battery == BAT4){ pin=BAT4_R_SECONDARY_PIN; lcd.setCursor(18,2); lcd.print("S"); }
  }
#else
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
#endif

  digitalWrite(pin, HIGH);    
  delay(80);
  digitalWrite(pin, LOW);      
  delay(80);      

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

void Iinadq(void){    
    // subtract the last reading:
    iintotal= iintotal - iinreadings[iinindex];         
    // read from the sensor:  
    iinreadings[iinindex] = analogRead(IINPUT_ANALOG); 
    // add the reading to the total:
    iintotal= iintotal + iinreadings[iinindex];       
    // advance to the next position in the array:  
    iinindex++;

    // if we're at the end of the array...
    if (iinindex >= numReadings)              
      // ...wrap around to the beginning: 
      iinindex = 0;                           

    // calculate the average:
    iinaverage = iintotal / numReadings;           
}

void Vpsadq(void){
    // subtract the last reading:
    vptotal= vptotal - vpreadings[vpindex];         
    vstotal= vstotal - vsreadings[vsindex];             
    // read from the sensor:  
    vpreadings[vpindex] = analogRead(VPRIMARY_ANALOG);
    delay(1);
    vsreadings[vsindex] = analogRead(VSECONDARY_ANALOG);
    delay(1);
    // add the reading to the total:
    vptotal= vptotal + vpreadings[vpindex];       
    vstotal= vstotal + vsreadings[vsindex];           
    // advance to the next position in the array:  
    vpindex++;
    vsindex++;    

    // if we're at the end of the array...
    if (vpindex >= numReadings)              
      // ...wrap around to the beginning: 
      vpindex = 0;                           

    if (vsindex >= numReadings)              
      // ...wrap around to the beginning: 
      vsindex = 0;                           
      
    // calculate the average:
    vpaverage = vptotal / numReadings;
    vsaverage = vstotal / numReadings;  
}


void Vccadq(void){    
    // subtract the last reading:    
    vcctotal= vcctotal - vccreadings[vccindex];         
    // read from the sensor:  
    vccreadings[vccindex] = readVcc(); 
    // add the reading to the total:
    vcctotal= vcctotal + vccreadings[vccindex];       
    // advance to the next position in the array:  
    vccindex++;

    // if we're at the end of the array...
    if(vccindex >= numReadings)              
      // ...wrap around to the beginning: 
      vccindex = 0;                           

    // calculate the average:
    vccaverage = vcctotal / numReadings;           
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

