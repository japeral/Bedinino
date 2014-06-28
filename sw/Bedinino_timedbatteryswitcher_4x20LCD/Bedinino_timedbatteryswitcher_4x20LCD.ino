
//
// Bedinino
//
// Bedinino is an electronic control unit for Bedini monopole motor/generator systems.
// It is intended for improve system COP (Coefficient Of Performance), and help other 
// people as a platform for start researching this Free Energy devices.
//
// This is the list of desired requeriments that will be implemented:
// 
// (DONE)  4 bistable relay automatic swap for primary and secondary battery, with countdown timer.
// (DONE)  4 lines, 20 character LCD information display.
// (TO DO) Primary battery voltage adquisition.
// (TO DO) Secondary battery voltaga adquisition vía isolated channel.
// (TO DO) Primary current adquisition by shunt or hall effect current sensors.
// (TO DO) Secondary current input adquisition and RMS calculations (by software or dedicated hardware).
// (TO DO) Input mAh totalizer.
// (TO DO) Output mAh totalizer.
// (TO DO) Ratiometric magnetic sensor adquisition.
// (TO DO) Tachometer function.
// (TO DO) N-Channel Mosfet drive.
// (TO DO) Pulse delay control by potentiometer.
// (TO DO) Pulse width control by potentiometer.
// (TO DO) Instantaneous COP calculation.
// (TO DO) Automatic Sweet point search algorithm.
// (TO DO) Selfstarting driving some kind of electric motor.
// (TO DO) Serialized data output to PC for logging and data plotting in Matlab or LowView.
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
// v1.10 2014-06-28  Added 4x20 LCD for information display.
//                   Added countdown to next battery switch.
//                   Added magnetometer function.
//                   Added instantaneous rotor tachometer function.
//                   Added RPM 10 second tendence indicator (stable, accelerating, decelerating, stopped).
//                   Added rpm reading hold before battery switch.
//
// v1.00 2014-06-23  Initial release, basic timed 4 battery switcher.
//
// by J.A.Peral
//

// Pins definition
#define BAT1_R_SECONDARY_PIN   2
#define BAT1_S_PRIMARY_PIN     3
#define BAT2_R_SECONDARY_PIN   4
#define BAT2_S_PRIMARY_PIN     5
#define BAT3_R_SECONDARY_PIN   6
#define BAT3_S_PRIMARY_PIN     7
#define BAT4_R_SECONDARY_PIN   8
#define BAT4_S_PRIMARY_PIN     9

// Bedini constants definition
//#define BATTERIES_SWAP_TIME    300000  // 5 minutes = 5*60*1000 = 300000ms
#define BATTERIES_SWAP_TIME    1200000 // 20 minutes = 20*60*1000 = 1200000ms
#define MAGNETS_IN_ROTOR       4       // Set here the number of magnets attached to your rotor.

// Ecu peripherals enable
#define LCD                           // Enable LCD information display.

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

// Constants definition
#define PRIMARY                0
#define SECONDARY              1

enum{
  BAT1=1,
  BAT2,
  BAT3,
  BAT4
};

enum{
  PRIMARY_NONE_SET=0,
  PRIMARY_NONE_STAY,
  PRIMARY_1_SET,
  PRIMARY_1_STAY,
  PRIMARY_2_SET,
  PRIMARY_2_STAY,
  PRIMARY_3_SET,
  PRIMARY_3_STAY,
  PRIMARY_4_SET,
  PRIMARY_4_STAY
};


// Global variables
unsigned long switchtimer=0;
unsigned long lcdrefresh=0;
unsigned char batteries_state=0;
unsigned long countdowntimer=0;
unsigned long swapsecondscountdown=0;
char message[21];

//---------------------------------------------------------
// Analog adquisition with smooth filtering (averaging).
//
const int numReadings = 10;       // Number of average samples.
unsigned long adqiintimer=0;      // Sample rate for iin
unsigned long adqvpstimer=0;      // Sample rate for Vp and Vs
// Iin = Instantaneous input current from primary battery.
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

unsigned long adqmagtimer=0;
int mag;
//
//---------------------------------------------------------


// Local functions
void BatSet(unsigned char battery, unsigned char side);
void PrimaryNoneSet(void);
  
void setup(){
 
    // Serial Setup.
    Serial.begin(9600);
         
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
      strcpy(message, "[Bedinino v1.10]");
      lcd.print(message);
      Serial.println(message);
      lcd.setCursor(0,1);    // Char pos 0, Line 1      
      strcpy(message, "Bedini Energizer ECU");
      lcd.print(message);
      Serial.println(message);      
      lcd.setCursor(0,2);    // Char pos 0, Line 2.   
      strcpy(message, "GTEL Research group.");
      lcd.print(message);
      Serial.println(message);            
      lcd.setCursor(4,3);    // Char pos 0, Line 3.            
      strcpy(message, "by J.A.Peral");
      lcd.print(message);
      Serial.println(message);            
      delay(4000);
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
    
    // Clean up iinreadings int array.
    for (int thisReading = 0; thisReading < numReadings; thisReading++){
      iinreadings[thisReading] = 0; 
      vpreadings[thisReading] = 0; 
      vsreadings[thisReading] = 0;       
    }
    
    
}

void loop(){

  // Battery switcher state machine.
  switch(batteries_state){
    default:
    case PRIMARY_NONE_SET:
      PrimaryNoneSet();
      batteries_state=PRIMARY_NONE_STAY;
      break;    
    case PRIMARY_NONE_STAY:
      batteries_state=PRIMARY_1_SET;
      break;
      
    case PRIMARY_1_SET:    
      PrimaryNoneSet();    
      Serial.println("Primary_1_SET");      
      BatSet(BAT1, PRIMARY);
      BatSet(BAT2, SECONDARY);      
      BatSet(BAT3, SECONDARY);      
      BatSet(BAT4, SECONDARY);
      batteries_state=PRIMARY_1_STAY;
      break;
    case PRIMARY_1_STAY:
      if(millis()-switchtimer>BATTERIES_SWAP_TIME){
        switchtimer=millis();
        batteries_state=PRIMARY_2_SET;
      }            
      break;
      
    case PRIMARY_2_SET:
      PrimaryNoneSet();   
      Serial.println("Primary_2_SET");
      BatSet(BAT1, SECONDARY);
      BatSet(BAT2, PRIMARY);
      BatSet(BAT3, SECONDARY);
      BatSet(BAT4, SECONDARY);
      batteries_state=PRIMARY_2_STAY;      
      break;
    case PRIMARY_2_STAY:
      if(millis()-switchtimer>BATTERIES_SWAP_TIME){
        switchtimer=millis();
        batteries_state=PRIMARY_3_SET;
      }            
      break;      
      
    case PRIMARY_3_SET:
      PrimaryNoneSet();
      Serial.println("Primary_3_SET");      
      BatSet(BAT1, SECONDARY);
      BatSet(BAT2, SECONDARY);
      BatSet(BAT3, PRIMARY);
      BatSet(BAT4, SECONDARY);    
      batteries_state=PRIMARY_3_STAY;
      break;
    case PRIMARY_3_STAY:
      if(millis()-switchtimer>BATTERIES_SWAP_TIME){
        switchtimer=millis();
        batteries_state=PRIMARY_4_SET;
      }            
      break;            
      
    case PRIMARY_4_SET:
      PrimaryNoneSet();       
      Serial.println("Primary_4_SET");      
      BatSet(BAT1, SECONDARY);
      BatSet(BAT2, SECONDARY);
      BatSet(BAT3, SECONDARY);
      BatSet(BAT4, PRIMARY);
      batteries_state=PRIMARY_4_STAY;      
      break;
    case PRIMARY_4_STAY:
      if(millis()-switchtimer>BATTERIES_SWAP_TIME){
        switchtimer=millis();
        batteries_state=PRIMARY_1_SET;
      }            
      break;
  }

  
  if(millis()-lcdrefresh>500){
    lcdrefresh=millis();

    // Convert to milli amps
//  int temp;
    // The offset of 512 has to be determined, perhaps during startup.
    iinfloat = ((float) (iinaverage  - 512) * 5.0 / 1023.0 ) * 1000.0 / 185.0;    
    iinfloat=iinfloat*1000;
//  temp=iinfloat;
//  iinfloat  = .0264 * iinaverage -13.51;  // ACS712-5Amp, 185mV/A    
//  average = average + (.0264 * analogRead(A0) -13.51);  // ACS712-5Amp, 185mV/A
    // Plot on LCD
    lcd.setCursor(8,0);
    lcd.print("         ");
    lcd.setCursor(8,0);
    dtostrf(iinfloat,3,0,message); //
    strcat(message,"mA");
    lcd.print(message);
    Serial.print(message);
    Serial.print("-");
    Serial.println(iinaverage);
    
        
    // Plot on LCD VP & VS
    #if defined LCD
      lcd.setCursor(0,0);
      dtostrf(vpfloat,5,2,message); //      
      lcd.print(message);
      lcd.setCursor(0,1);
      dtostrf(vsfloat,5,2,message); //      
      lcd.print(message);
    #endif     
  }
  
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


  if(millis()-adqiintimer>50){
    adqiintimer=millis();
    
    // subtract the last reading:
    iintotal= iintotal - iinreadings[iinindex];         
    // read from the sensor:  
    iinreadings[iinindex] = analogRead(A0); 
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

  if(millis()-adqvpstimer>500){
    adqvpstimer=millis();
    
    // subtract the last reading:
    vptotal= vptotal - vpreadings[vpindex];         
    vstotal= vstotal - vsreadings[vsindex];             
    // read from the sensor:  
    vpreadings[vpindex] = analogRead(A1); 
    vsreadings[vsindex] = analogRead(A2);     
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
    
    vpfloat=map(vpaverage ,  0, 1024,  0,   15);
    vsfloat=map(vsaverage ,  0, 1024,  0,   15);    
    
  }
  
  if(millis()-adqmagtimer>100){
    adqmagtimer=millis();
    mag = analogRead(A3);    
  }
  
}

void BatSet(unsigned char battery, unsigned char side){
  
  int pin;

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
  digitalWrite(pin, HIGH);    
  delay(80);
  digitalWrite(pin, LOW);      
  delay(80);      
  
  swapsecondscountdown=BATTERIES_SWAP_TIME/1000;
}

void PrimaryNoneSet(void){
   Serial.println("Primary_None_SET");          
   BatSet(BAT1, SECONDARY);
   BatSet(BAT2, SECONDARY);      
   BatSet(BAT3, SECONDARY);      
   BatSet(BAT4, SECONDARY);
}


