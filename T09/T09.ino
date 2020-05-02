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

#define LEADING_ZERO 1

#define NUMERIC_ERROR 0x7FFF

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
    unsigned long fFreezeTimeStart=0;
    unsigned long fDriftPerMinuteStartTime=0;
    boolean  fApplyDriftCorection=false;
    float fGyroX, fGyroY, fFreezedGyroX;//input gyro angles
    float fAzOffsetX=0, fAzOffsetY=0;//output azimuthal angles
    float fDriftPerMinute=0;
  public:
    unsigned long lastPrint=0;
    boolean getFreeze(){
      return fFreeze;
    }

    void setFreezeOn(){
      fFreeze=true;
      digitalWrite(DIODE_PIN, fFreeze);
      fFreezeTimeStart=millis();
      fFreezedGyroX=fGyroX;
    }
    float calculateFreezeTimeDriftPerMinute(){
      unsigned long vFreezeSpan=0;
      float vDriftPerMinute=0.0;
      vFreezeSpan=millis()-fFreezeTimeStart;
      if(vFreezeSpan>0){
         vDriftPerMinute=((fGyroX-fFreezedGyroX)*60000.0)/vFreezeSpan; //per minute
      }
      return vDriftPerMinute;
    }
    void setDriftPerMinute(float pN){
        fApplyDriftCorection = (abs(pN-0.0)>0.001);        
        fDriftPerMinute=pN;
        fDriftPerMinuteStartTime==millis();

        Serial.print("Drift set to ");
        Serial.print(pN);
        Serial.println(" per minute!!!!!!!!!!!!!!!!");
    }
    void setFreezeOff(){
      fFreeze=false;
      digitalWrite(DIODE_PIN, fFreeze);
    }

    void setGyroCoord(float pX, float pY){
        fGyroX=pX;
        fGyroY=pY;
    }

    void setAzCoord(float pX, float pY){
        fAzOffsetX=pX-fGyroX;        
        fAzOffsetY=pY-fGyroY;        
        fDriftPerMinuteStartTime=millis();
    }
    
    void setAzCoordX(float pX){
        fAzOffsetX=pX-fGyroX;        
        fDriftPerMinuteStartTime=millis();
    }    
    
    void setAzCoordByIncrement(float pX, float pY){
        fAzOffsetX+=pX;
        fAzOffsetY+=pY;
    }

    float getAzCoordX(){
        float vX=fGyroX+fAzOffsetX;
        //Serial.print(" mialo byc ");
        //Serial.println(vX);

        if (fApplyDriftCorection){
            vX-= (millis()-fDriftPerMinuteStartTime)*fDriftPerMinute/60000.0;
        }

        /*Serial.print(" jest ");
        Serial.println(makeNumber360(vX));
        Serial.println(fDriftPerMinuteStartTime);
        Serial.println(fCumulativePrevDrift     );
        Serial.println(fAzOffsetX     );*/
        return makeNumber360(vX);
    }

    float getAzCoordY(){
        return fGyroY+fAzOffsetY;
    }
};
static GyroState gGyroState;

enum Tmenustate{home, freeze, freeze_enter_coord, freeze_question, pos_set, calibration, time_set, local_sid_time, longitude};

uint8_t diplayData[] = { 0xff, 0xff, 0xff, 0xff };
char   tempInputData[] = { 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0x0 };   

class NavigationState{

    private:
 
        boolean fAllowDisplayGyro=true;
        float vCalculatedDrift;
        int vSetAzX, vSetAzY;
        int vEnteredAzX, vEnteredAzY;
        


    public:
        Tmenustate fMenuState=home;
        int fMenuSubstate=0;
        int fBrightness=0;

        boolean allowDisplayGyro(){
            return fAllowDisplayGyro;
        }

        void showPrompt(TM1637Display pDisplay, int pBufferStartPos){
            for(int i=0;i<5;i++){
                diplayData[i] = SEG_D;
                if (pBufferStartPos+i>8){buzzer(500);}
                tempInputData[pBufferStartPos+i]=0xf;
            }
            pDisplay.setSegments(diplayData);
        }
        
        int inputDataToIntAndPrint(int pFirstPos, int pLen, TM1637Display pDisplay, int pDefaultValue)
        {  int vResult=0, v10Pow=pLen-1, vDisplayIdx=0;
        
           for (int i=pFirstPos; i<pFirstPos+pLen; i++){
                if (i>8){buzzer(500);}
                if (vDisplayIdx>4){buzzer(500);delay(5000);buzzer(500);}
                if (tempInputData[i]<=9 && vResult!=NUMERIC_ERROR){
                   vResult+=tempInputData[i]*pow(10,v10Pow);
                   diplayData[vDisplayIdx]=pDisplay.encodeDigit(tempInputData[i]);
                } else {
                   vResult=NUMERIC_ERROR;           
                   diplayData[vDisplayIdx]=SEG_D;        
                }
                v10Pow--;                 
                vDisplayIdx++;
           }
           if (pFirstPos<4){
                pDisplay.setSegments(diplayData, pLen, pFirstPos);
           } else {
                pDisplay.setSegments(diplayData, pLen, pFirstPos-4);
           }
           Serial.println(" inputDataToIntAndPrint.result 1 and 2");
           Serial.println(pFirstPos);
           Serial.println(vResult);           
           if (vResult==NUMERIC_ERROR){
               vResult=pDefaultValue;
               if (pDefaultValue!=NUMERIC_ERROR){
                   if (pFirstPos<4){               
                        pDisplay.showNumberDec(pDefaultValue, LEADING_ZERO, pLen, pFirstPos);
                   } else {
                        pDisplay.showNumberDec(pDefaultValue, LEADING_ZERO, pLen, pFirstPos-4);
                   }
               }
           }
           Serial.println(vResult);           
           return vResult;
        }
        

        void remoteServe(int pKey)
        {
         //Serial.print(" remoteServe");
         //Serial.println(pKey);
            int vTmpNum;

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
                      gGyroState.setDriftPerMinute(0.0);
                      gGyroState.setAzCoordX(vSetAzX);
                    }
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
                    } else if (pKey==REMOTE_HASH){
                        fMenuState=freeze_enter_coord;
                        fMenuSubstate=0;
                        vEnteredAzX = vSetAzX; 
                        vEnteredAzY = vSetAzY;
                        showPrompt(display2, 0);                        
                    }      
                    if (pKey!=REMOTE_HASH && pKey!=REMOTE_ASTER){
                        display2.showNumberDec(vSetAzX, LEADING_ZERO, 4, 0);
                        display1.showNumberDec(vSetAzY, LEADING_ZERO, 4, 0);
                    }
                    delay(200);
                    break;

                case freeze_enter_coord:
                    if (pKey==REMOTE_HASH){
                        if (fMenuSubstate<4){
                            fMenuSubstate=5;
                            vEnteredAzX = inputDataToIntAndPrint(0, 4, display2, vEnteredAzX);
                            showPrompt(display1,4);                            
                        } else {
                            fMenuSubstate=0;
                            vEnteredAzY = inputDataToIntAndPrint(4, 4, display1, vEnteredAzY);
                            showPrompt(display2,0);                            
                        }
                    } else if (pKey==REMOTE_LEFT && fMenuSubstate>0 && fMenuSubstate<5 ){
                        tempInputData[fMenuSubstate-1]=0xf;
                        diplayData[0]=SEG_D;
                        display2.setSegments(diplayData, 1, fMenuSubstate-1);
                        fMenuSubstate--;
                    } else if (pKey==REMOTE_LEFT && fMenuSubstate>=5 && fMenuSubstate<9 ){
                        tempInputData[fMenuSubstate-1]=0xf;
                        diplayData[0]=SEG_D;
                        display1.setSegments(diplayData, 1, fMenuSubstate-4-1);
                        fMenuSubstate--;
                    } else if (pKey>=0 && pKey<=9 && fMenuSubstate<4){
                        fMenuSubstate++;
                        tempInputData[fMenuSubstate-1]=pKey;
                        vTmpNum = inputDataToIntAndPrint(0, 4, display2, NUMERIC_ERROR);
                        vEnteredAzX =    (vTmpNum==NUMERIC_ERROR ? vEnteredAzX : vTmpNum);
                        //diplayData[0]=display2.encodeDigit(pKey);
                        //display2.setSegments(diplayData, 1, fMenuSubstate-1);
                        if (fMenuSubstate==4){
                            showPrompt(display1, 4);
                        }
                    } else if (pKey>=0 && pKey<=9 && fMenuSubstate>=4 && fMenuSubstate<8){
                        fMenuSubstate++;
                        tempInputData[fMenuSubstate-1]=pKey;                        
                        vTmpNum = inputDataToIntAndPrint(4, 4, display1, NUMERIC_ERROR);
                        vEnteredAzY =  (vTmpNum==NUMERIC_ERROR ? vEnteredAzY : vTmpNum);
                        //diplayData[0]=display1.encodeDigit(pKey);
                        //display1.setSegments(diplayData, 1, fMenuSubstate-4-1);
                    } else if (pKey==REMOTE_ASTER){
                        gGyroState.setAzCoord(vSetAzX, vSetAzY);                         
                        gGyroState.setFreezeOff();                        
                        fAllowDisplayGyro=true;
                        fMenuState=home;
                        delay(200);                        
                    } else if (pKey==REMOTE_OK){
                        gGyroState.setAzCoord(vEnteredAzX, vEnteredAzY);                        
                        vSetAzX=vEnteredAzX; vSetAzY=vEnteredAzY;
                        buzzer(30);delay(30);buzzer(30);
                        fMenuState=freeze;                        
                        remoteServe(REMOTE_ASTER);
                    }
                    break;

                case freeze_question:
                    if (pKey==REMOTE_OK){
                        gGyroState.setDriftPerMinute(vCalculatedDrift);
                    }
                    gGyroState.setFreezeOff();
                    gGyroState.setAzCoord(vSetAzX, vSetAzY);                       
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
      Serial.print("     AccX = ");
      Serial.print(mpu.GetRawGyroX());      
      Serial.print("     AccY = ");
      Serial.print(mpu.GetRawGyroY());      
      Serial.print("     AccZ = ");
      Serial.println(mpu.GetRawGyroZ());      
      
      

  }

}
