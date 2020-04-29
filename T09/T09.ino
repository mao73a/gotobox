/*
 *  Mandatory includes
 */
#include <Arduino.h>
#include <TinyMPU6050.h>
#include <TM1637Display.h>
#include "RTClib.h"
#include "remote_decode.h"

#define DISPLAY1_CLK 2
#define DISPLAY1_DIO 3
#define DISPLAY2_CLK 4
#define DISPLAY2_DIO 5
#define BUZZER_PIN 12
#define IRREC_PIN 7

//
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
//SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,   // 8

const uint8_t napis_CAL[] = {
  SEG_A | SEG_D | SEG_E | SEG_F ,           // C
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,   // A
  SEG_D | SEG_E | SEG_F,   // L
  0
};
const uint8_t napis_HI[] = {
  SEG_B | SEG_C |  SEG_E | SEG_F | SEG_G,   // H
  SEG_B | SEG_C ,   // I
  0,
  0
};

TM1637Display display1 = TM1637Display(DISPLAY1_CLK, DISPLAY1_DIO);
TM1637Display display2 = TM1637Display(DISPLAY2_CLK, DISPLAY2_DIO);
MPU6050 mpu (Wire, MPU6050_ADDRESS_HIGH); //ADO=5V!!!
RTC_DS1307 rtc;
IRrecv irrecv(IRREC_PIN); 


void buzzer(int pCzas=1){
  digitalWrite(BUZZER_PIN,HIGH);
  delay(pCzas);
  digitalWrite(BUZZER_PIN,LOW);
}

void remoteServe(int pKey)
{
 //Serial.print(" remoteServe");   
 //Serial.println(pKey); 
    if(pKey!=REMOTE_UNKNOWN){
      buzzer(1);
    } 
    if (pKey>=0 && pKey<=9){
        Serial.print(pKey); 
        Serial.print(" "); 
    }
    else if (pKey==REMOTE_LEFT){
        Serial.print("Left "); 
    }
    else if (pKey==REMOTE_RIGHT){
        Serial.print("Right "); 
    }
    else if (pKey==REMOTE_UP){
        Serial.print("Up "); 
    }
    else if (pKey==REMOTE_DOWN){
        Serial.print("Down "); 
    }    
    else if (pKey==REMOTE_ASTER){
        Serial.print("* "); 
    }  
    else if (pKey==REMOTE_HASH){
        Serial.print("# "); 
    }      
    else if (pKey==REMOTE_OK){
        Serial.println("Enter "); 
    }          
    else {
        //Serial.println("UNKNOWN"); 
    }
}



void setup() {
 int i;
  // Initialization
  mpu.Initialize();
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
  }  
  
  pinMode(BUZZER_PIN,OUTPUT);//initialize the buzzer pin as an output
  irrecv.enableIRIn(); // uruchamia odbiornik podczerwieni
  
  // Calibration
  Serial.begin(9600);
  display1.clear();
  display2.clear();  
  display1.setBrightness(7);
  display2.setBrightness(7);  
  display2.setSegments(napis_HI);
  for(i=5;i>=0;i--){
   display1.showNumberDec(i);
   delay(1000);
  }
  display1.clear();
  display2.clear();    
  display1.setSegments(napis_CAL);
  display2.setSegments(napis_CAL);  
  Serial.println("=====================================");
  Serial.println("Starting calibration...");
  mpu.Calibrate();
  Serial.println("Calibration complete!");
  Serial.println("Offsets:");
  Serial.print("GyroX Offset = ");
  Serial.println(mpu.GetGyroXOffset());
  Serial.print("GyroY Offset = ");
  Serial.println(mpu.GetGyroYOffset());
  Serial.print("GyroZ Offset = ");
  Serial.println(mpu.GetGyroZOffset());
  display1.clear();
  display2.clear();
}

/*
 *  Loop
 */
void loop() {
  decode_results vIRResults;
  DateTime now = rtc.now();

  if (irrecv.decode(&vIRResults))
  { 
        remoteServe(remoteDecode(vIRResults));
        irrecv.resume();
        delay(100);
  }
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);  
  Serial.print(" (");  
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();  
  
  mpu.Execute();
  Serial.print("AngX = ");
  Serial.print(mpu.GetAngX());
  Serial.print("  /  AngY = ");
  Serial.print(mpu.GetAngY());
  Serial.print("  /  AngZ = ");
  Serial.println(mpu.GetAngZ());
  display1.showNumberDec(mpu.GetAngX()*10, true, 4, 0); 
  //display1.showNumberDec(mpu.GetAngX()*10);
  display2.showNumberDec(-mpu.GetAngZ()*10, true, 4, 0); 
  //display2.showNumberDec(-mpu.GetAngZ()*10); 
}
