
//
// Bedinidno
//
// 
//
// by J.A.Peral 23/jun/2014
//

#define BATTERIES_SWAP_TIME    300000  // 5 minutes = 5*60*1000 = 300000ms

#define BAT1_R_SECONDARY_PIN   2
#define BAT1_S_PRIMARY_PIN     3
#define BAT2_R_SECONDARY_PIN   4
#define BAT2_S_PRIMARY_PIN     5
#define BAT3_R_SECONDARY_PIN   6
#define BAT3_S_PRIMARY_PIN     7
#define BAT4_R_SECONDARY_PIN   8
#define BAT4_S_PRIMARY_PIN     9

//

#define PRIMARY                0
#define SECONDARY              1

#define BAT1                   1
#define BAT2                   2
#define BAT3                   3
#define BAT4                   4

#define PRIMARY_NONE_SET       0
#define PRIMARY_NONE_STAY      1
#define PRIMARY_1_SET          2
#define PRIMARY_1_STAY         3
#define PRIMARY_2_SET          4
#define PRIMARY_2_STAY         5
#define PRIMARY_3_SET          6
#define PRIMARY_3_STAY         7
#define PRIMARY_4_SET          8
#define PRIMARY_4_STAY         9

unsigned long switchtimer=0;
unsigned long serialflush=0;
unsigned char batteries_state=0;

void BatSet(unsigned char battery, unsigned char side);
void PrimaryNoneSet(void);
  
void setup(){
    Serial.begin(9600);
  
    pinMode(BAT1_R_SECONDARY_PIN, OUTPUT);
    pinMode(BAT1_S_PRIMARY_PIN, OUTPUT);
    pinMode(BAT2_R_SECONDARY_PIN, OUTPUT);
    pinMode(BAT2_S_PRIMARY_PIN, OUTPUT);
    pinMode(BAT3_R_SECONDARY_PIN, OUTPUT);
    pinMode(BAT3_S_PRIMARY_PIN, OUTPUT);
    pinMode(BAT4_R_SECONDARY_PIN, OUTPUT);
    pinMode(BAT4_S_PRIMARY_PIN, OUTPUT);
}

void loop(){
  
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
  
  if(millis()-serialflush>1000){
    serialflush=millis();
  }
  
}

void BatSet(unsigned char battery, unsigned char side){
  
  int pin;

  if (side==PRIMARY){ 
    if(battery == BAT1)  pin=BAT1_S_PRIMARY_PIN;
    if(battery == BAT2)  pin=BAT2_S_PRIMARY_PIN;
    if(battery == BAT3)  pin=BAT3_S_PRIMARY_PIN;
    if(battery == BAT4)  pin=BAT4_S_PRIMARY_PIN;
  }else{
    if(battery == BAT1)  pin=BAT1_R_SECONDARY_PIN;
    if(battery == BAT2)  pin=BAT2_R_SECONDARY_PIN;
    if(battery == BAT3)  pin=BAT3_R_SECONDARY_PIN;
    if(battery == BAT4)  pin=BAT4_R_SECONDARY_PIN;
  }
  digitalWrite(pin, HIGH);    
  delay(80);
  digitalWrite(pin, LOW);      
  delay(80);      
}

void PrimaryNoneSet(void){
   Serial.println("Primary_None_SET");          
   BatSet(BAT1, SECONDARY);
   BatSet(BAT2, SECONDARY);      
   BatSet(BAT3, SECONDARY);      
   BatSet(BAT4, SECONDARY);
}


