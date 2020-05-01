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
#define DIODE_PIN 6

#define LEADING_ZERO 0


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
const uint8_t napis_Cor[] = {
  SEG_A | SEG_D | SEG_E | SEG_F ,           // C
  SEG_C | SEG_D | SEG_E |  SEG_G,   // o
  SEG_E | SEG_G,   // r,
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

float makeNumber360(float pNum){
    float newPos=pNum;
    while(newPos<0.0){
      newPos+=3600.0;
    }
    while(newPos>=3600.0){
      newPos-=3600.0;
    }
    return newPos;
}
    
class GyroState {
  private:
    boolean fFreeze;
    unsigned long fFreezeTime=0;
    unsigned long fDriftPerMinuteTime=0;
    float fGyroX, fGyroY, fFreezedGyroX;//input gyro angles
    float fAzOffsetX=0, fAzOffsetY=0;//output azimuthal angles
    float fDriftPerMinute=0, fCumulativePrevDrift=0;
  public:
    unsigned long lastPrint=0;
    boolean getFreeze(){
      return fFreeze;
    }

    void setFreezeOn(){
      fFreeze=true;
      digitalWrite(DIODE_PIN, fFreeze);
      fFreezeTime=millis();
      if (fDriftPerMinuteTime>0){
        fCumulativePrevDrift+=(millis()-fDriftPerMinuteTime)*fDriftPerMinute/60000.0;
      }
    }
    float calculateFreezeTimeDriftPerMinute(){
      unsigned long vFreezeSpan=0;    
      float vDriftPerMinute=0.0;
      vFreezeSpan=millis()-fFreezeTime;
      if(vFreezeSpan>0){
         vDriftPerMinute=((fFreezedGyroX-fGyroX)*60000.0)/vFreezeSpan; //per minute
      }
      return vDriftPerMinute;
    }
    void setDriftPerMinute(float pN){
        fDriftPerMinute=pN;
        fDriftPerMinuteTime=millis();
        Serial.print("Drift set to ");
        Serial.print(pN);        
        Serial.println(" per minute!!!!!!!!!!!!!!!!");        
    }
    void setFreezeOff(){
      fFreeze=false;
      digitalWrite(DIODE_PIN, fFreeze);
    }

    void setGyroCoord(float pX, float pY){
      if(!fFreeze){
        fGyroX=pX;
        fGyroY=pY;
      } else {
        fFreezedGyroX=pX;  
      }
    }
    void setAzCoord(float pX, float pY){
        fAzOffsetX=pX-fGyroX;
        fAzOffsetY=pY-fGyroY;
    }
    void setAzCoordByIncrement(float pX, float pY){
        fAzOffsetX+=pX;
        fAzOffsetY+=pY;
    }

    float getAzCoordX(){
        float vX=fGyroX+fAzOffsetX-fCumulativePrevDrift;
        Serial.print(" mialo byc "); 
        Serial.println(vX);         

        if (fDriftPerMinuteTime>0){
            vX-= (millis()-fDriftPerMinuteTime)*fDriftPerMinute/60000.0;
        }

        Serial.print(" jest "); 
        Serial.println(makeNumber360(vX));         
        Serial.println(fDriftPerMinuteTime);          
        Serial.println(fCumulativePrevDrift     );                  
        Serial.println(fAzOffsetX     );                          
        
   
       
        return makeNumber360(vX);
    }

    float getAzCoordY(){
        return fGyroY+fAzOffsetY;
    }
};
static GyroState gGyroState;

enum Tmenustate{home, freeze, freeze_question, pos_set, calibration, time_set, local_sid_time, longitude};

uint8_t diplayData1[] = { 0xff, 0xff, 0xff, 0xff }; 
class NavigationState{

    private:
        boolean fAllowDisplayGyro=true;
        float vCalculatedDrift;        
        int vSetAzX, vSetAzY;

    public:
        Tmenustate fMenuState=home;
        int fMenuSubstate=0;
        int fBrightness=0;
        boolean allowDisplayGyro(){
            return fAllowDisplayGyro;
        }
        void remoteServe(int pKey)
        {
         //Serial.print(" remoteServe");
         //Serial.println(pKey);

            if(pKey!=REMOTE_UNKNOWN){
              buzzer(1);
            }

            switch (fMenuState){
                case home:
                    if (pKey==REMOTE_ASTER){
                        fMenuState=freeze;
                        gGyroState.setFreezeOn();
                        fAllowDisplayGyro=false;
                        vSetAzX = gGyroState.getAzCoordX();
                        vSetAzY = gGyroState.getAzCoordY();
                    } else if (pKey==REMOTE_UP){
                        fBrightness=constrain(fBrightness+1,0,3);
                    } else if (pKey==REMOTE_DOWN){
                        fBrightness=constrain(fBrightness-1,0,3);
                    } else if (pKey==REMOTE_HASH){
                      display1.setSegments(napis_CAL);
                      display2.setSegments(napis_CAL);
                      mpu.Calibrate();
                    }
                    fMenuSubstate=0;
                    display1.setBrightness(fBrightness);
                    display2.setBrightness(fBrightness);                    
                    delay(200);
                    break;

                case freeze:
                    if (pKey==REMOTE_ASTER){
                        display2.setSegments(napis_Cor);
                        fMenuState=freeze_question;
                        vCalculatedDrift=gGyroState.calculateFreezeTimeDriftPerMinute();
                        fAllowDisplayGyro=false;  
                        display1.clear();
                        display2.clear();
                        display2.setSegments(napis_Cor);                    
                        display1.showNumberDec(round(vCalculatedDrift), false, 4, 0);
                        delay(1000);                        
                    } else if (pKey==REMOTE_LEFT){
                        vSetAzX=makeNumber360(vSetAzX-1);
                    } else if (pKey==REMOTE_RIGHT){
                        vSetAzX=makeNumber360(vSetAzX+1);
                    } else if (pKey==REMOTE_UP){
                        vSetAzY++;
                    } else if (pKey==REMOTE_DOWN){
                        vSetAzY--;
                    } else if (pKey==REMOTE_HASH && fMenuSubstate>0 && fMenuSubstate<5 ){
                        diplayData1[fMenuSubstate-1]=SEG_D;
                        display2.setSegments(diplayData1+fMenuSubstate-1, 1, fMenuSubstate-1);  
                        fMenuSubstate--;      
                   } else if (pKey==REMOTE_HASH && fMenuSubstate>=5 && fMenuSubstate<9 ){
                        diplayData1[fMenuSubstate-4-1]=SEG_D;
                        display1.setSegments(diplayData1+fMenuSubstate-4-1, 1, fMenuSubstate-4-1); 
                        fMenuSubstate--;                           
                    } else if (pKey>=0 && pKey<=9 && fMenuSubstate<4){
                        if (fMenuSubstate==0)
                        { 
                            for(int i=0;i<5;i++){diplayData1[i] = SEG_D;}
                            display2.setSegments(diplayData1);     
                            delay(200);
                        }
                        fMenuSubstate++;
                        diplayData1[fMenuSubstate-1]=display2.encodeDigit(pKey);
                        display2.setSegments(diplayData1+fMenuSubstate-1, 1, fMenuSubstate-1);     
                    }
                    else if (pKey>=0 && pKey<=9 && fMenuSubstate>=4 && fMenuSubstate<8){
                        if (fMenuSubstate==4)
                        { 
                            for(int i=0;i<5;i++){diplayData1[i] = SEG_D;}
                            display1.setSegments(diplayData1);     
                            delay(200);
                        }
                        fMenuSubstate++;
                        diplayData1[fMenuSubstate-1]=display1.encodeDigit(pKey);
                        display1.setSegments(diplayData1+fMenuSubstate-1, 1, fMenuSubstate-1);                           
                    }       
                    
                    if (fMenuSubstate==0){
                        display2.showNumberDec(vSetAzX, LEADING_ZERO, 4, 0);
                        display1.showNumberDec(vSetAzY, LEADING_ZERO, 4, 0);
                    }

                    delay(200);
                    break;
                case freeze_question:         
                    if (pKey==REMOTE_OK){
                        gGyroState.setDriftPerMinute(vCalculatedDrift);
                    } else {}
                    gGyroState.setFreezeOff();
                    fAllowDisplayGyro=true;
                    fMenuState=home;
                    delay(200);
                    
                    break;                        
            }


            if (pKey>=0 && pKey<=9){
                Serial.print(pKey);
                Serial.print(" ");
            }
            else if (pKey==REMOTE_LEFT){
                Serial.print("Left ");
                gGyroState.setAzCoordByIncrement(-1,0);
            }
            else if (pKey==REMOTE_RIGHT){
                Serial.print("Right ");
                gGyroState.setAzCoordByIncrement(+1,0);
            }
            else if (pKey==REMOTE_UP){
                Serial.print("Up ");
                gGyroState.setAzCoordByIncrement(0,+1);
            }
            else if (pKey==REMOTE_DOWN){
                Serial.print("Down ");
                gGyroState.setAzCoordByIncrement(0,-1);
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
      
};

static NavigationState gNavigationState;



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
  pinMode(DIODE_PIN, OUTPUT);//czerwona dioda freeze

  // Calibration
  Serial.begin(9600);
  display1.clear();
  display2.clear();
  display1.setBrightness(gNavigationState.fBrightness);
  display2.setBrightness(gNavigationState.fBrightness);
  display2.setSegments(napis_HI);
  for(i=1;i>=0;i--){
   display1.showNumberDec(i);
   delay(1000);
  }

  /*
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
  */

  mpu.SetGyroOffsets(-327.04, 109.60, -76.95);
  display1.clear();
  display2.clear();
  gGyroState.setAzCoord(0,0);
}



void loop() {
  decode_results vIRResults;
  DateTime now;

  if (irrecv.decode(&vIRResults))
  {
        gNavigationState.remoteServe(remoteDecode(vIRResults));
        irrecv.resume();
        delay(100);
  }
  mpu.Execute();
  gGyroState.setGyroCoord(-mpu.GetAngZ()*10, mpu.GetAngX()*10);
  if (gNavigationState.allowDisplayGyro()){
      display1.showNumberDec(gGyroState.getAzCoordY(), LEADING_ZERO, 4, 0);
      display2.showNumberDec(gGyroState.getAzCoordX(), LEADING_ZERO, 4, 0);
  }
  if ((millis()-gGyroState.lastPrint)>1000){
      gGyroState.lastPrint=millis();
      now = rtc.now();
      Serial.print(now.year(), DEC);
      Serial.print('/');
      Serial.print(now.month(), DEC);
      Serial.print('/');
      Serial.print(now.day(), DEC);
      Serial.print(" ");
      Serial.print(now.hour(), DEC);
      Serial.print(':');
      Serial.print(now.minute(), DEC);
      Serial.print(':');
      Serial.print(now.second(), DEC);
      Serial.println();


      Serial.print("AngX = ");
      Serial.print(mpu.GetAngX());
      Serial.print("  /  AngY = ");
      Serial.print(mpu.GetAngY());
      Serial.print("  /  AngZ = ");
      Serial.println(mpu.GetAngZ());

  }

}
