/*
 *  Mandatory includes
 */
#include <Arduino.h>
#include <TinyMPU6050.h>
#include <TM1637Display.h>
#include "remote_decode.h"
#include "AvgStd.h"

#define enc_2A 2
#define enc_2B 3
long pulses_enc2  = 33600; //600*(280/20)*4 - Data: gears: large:280 teeth, small:20 teeth, implustaror: 600 impluses 

#define DISPLAY1_CLK 8
#define DISPLAY1_DIO 9
#define DISPLAY2_CLK 4
#define DISPLAY2_DIO 5
//#define BUZZER_PIN 12
#define BUZZER_PIN A0
#define IRREC_PIN 7
//#define DIODE_PIN 6
#define DIODE_PIN A1
#define LEADING_ZERO 1
#define NUMERIC_ERROR 0x7FFF

#define SEND_STELLARIUM_PERIOD_MS 100//every X millis
#define READ_MPU_PERIOD_MS 50//every X millis
#define DISPLAY_COORDINATES_PERIOD_MS 50
#define SMOOTHING_ARRAY_COUNT 10

//currently not used
#define AUTOFREEZE_TIME_ENABLE 10000 //millis
#define AUTOFREEZE_TIME_CORRECTING_DRIFT 30 //seconds
#define AUTOFREEZE_MOTION_DETECTION_SIGMA 7
#define AUTOFREEZE_MOTION_DETECTION_COUNT 3
#define AUTOCALLIBRATION_TIME 4000 //millis



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

const uint8_t napis_Auto[] = {
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,   // A
  SEG_C | SEG_D | SEG_E ,   // u
  SEG_D | SEG_E | SEG_F | SEG_G,   // t
  SEG_C | SEG_D | SEG_E | SEG_G   // o
};

const uint8_t napis_on[] = {
  SEG_C | SEG_D | SEG_E | SEG_G,   // o
  SEG_C | SEG_E | SEG_G,   // n
  0,
  0
};

const uint8_t napis_off[] = {
  SEG_C | SEG_D | SEG_E | SEG_G,   // o
  SEG_A | SEG_E | SEG_F | SEG_G,   // F
  SEG_A | SEG_E | SEG_F | SEG_G,   // F
  0
};

TM1637Display display1 = TM1637Display(DISPLAY1_CLK, DISPLAY1_DIO);
TM1637Display display2 = TM1637Display(DISPLAY2_CLK, DISPLAY2_DIO);
//MPU6050 mpu (Wire, MPU6050_ADDRESS_HIGH); //ADO=5V!!!
MPU6050 mpu (Wire); //ADO=5V!!!

IRrecv irrecv(IRREC_PIN);

unsigned long gProfilingStartTime;
unsigned long gProfilingCount;
volatile int gLastEncoded2 = 0;
volatile long gEncoderValue2 = 0;

void buzzer(int pCzas=1){
  digitalWrite(BUZZER_PIN,HIGH);
  delay(pCzas);
  digitalWrite(BUZZER_PIN,LOW);
}

bool buzzerOnOff(boolean pOn){
    digitalWrite(BUZZER_PIN, pOn ? HIGH : LOW);
    return true;        
}  

void redDiode(boolean pOn){
    digitalWrite(DIODE_PIN, pOn ? HIGH : LOW);
}

void blinkRedDiode(int pMillis){
    static unsigned long sPrevBlink=0;
    static boolean sdiodeState;
    if (millis()-sPrevBlink>=pMillis){
        sdiodeState = !sdiodeState;
        redDiode(sdiodeState);
        sPrevBlink=millis();
    }
    
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

int gDisplay1LastValue=1234, gDisplay2LastValue=1234;
bool gDisplay1LastShowDot, gDisplay2LastShowDot;

void Display1_IVC(int num, bool leading_zero, uint8_t length, uint8_t pos, bool pShowDot)
{
    bool vLeadingZero=leading_zero;    
    if (length==0 && pos==0){
        gDisplay1LastValue=9234;
        return;
    }        
    if (gDisplay1LastValue==num && gDisplay1LastShowDot==pShowDot){
        return;
    }
    if (num<0){
        vLeadingZero=false;
    }
    
    //display1.showNumberDec(num, LEADING_ZERO, 4, 0);
    if (num<-999){    
        display1.showNumberDecEx(-999, pShowDot ? 0xffff : 0x0000, vLeadingZero, 4, 0);//signalize (value too small)
    } else {
        display1.showNumberDecEx(num, pShowDot ? 0xffff : 0x0000, vLeadingZero, 4, 0);
    }
    gDisplay1LastValue=num;
    gDisplay1LastShowDot=pShowDot;
}
void Display2_IVC(int num, bool leading_zero, uint8_t length, uint8_t pos, bool pShowDot)
{
    bool vLeadingZero=leading_zero;
    if (length==0 && pos==0){
        gDisplay2LastValue=9234;
        return;
    }            
    if (gDisplay2LastValue==num && gDisplay2LastShowDot==pShowDot){
        return;
    }
    if (num<0){
        vLeadingZero=false;
    }
    
    if (num<-999){
        display2.showNumberDecEx(-999, pShowDot ? 0xffff : 0x0000, vLeadingZero, 4, 0);//signalize (value too small)
    } else {
        display2.showNumberDecEx(num, pShowDot ? 0xffff : 0x0000, vLeadingZero, 4, 0);
    }
    gDisplay2LastValue=num;
    gDisplay2LastShowDot=pShowDot;    
}

class MotionDetection {
    private:
        int16_t xStableMin, xStableMax, yStableMin, yStableMax;   
        int fMotionDetectionCounter=0;
    public:

        AvgStd dataX;
        AvgStd dataY;
        AvgStd dataZ;
        
        void CalibrateReset(){
            dataX.reset(); dataY.reset(); dataZ.reset();            
        }
        void CalibrateGeatherData(){
                mpu.UpdateRawGyro();
                dataX.checkAndAddReading(mpu.GetRawGyroX());
                dataY.checkAndAddReading(mpu.GetRawGyroY());
                dataZ.checkAndAddReading(mpu.GetRawGyroZ());
        }      
        void CalibrateCommit(){
            xStableMin=dataX.getMean() - dataX.getStd() * AUTOFREEZE_MOTION_DETECTION_SIGMA/2.0;
            xStableMax=dataX.getMean() + dataX.getStd() * AUTOFREEZE_MOTION_DETECTION_SIGMA/2.0;
            yStableMin=dataY.getMean() - dataY.getStd() * AUTOFREEZE_MOTION_DETECTION_SIGMA/2.0;
            yStableMax=dataY.getMean() + dataY.getStd() * AUTOFREEZE_MOTION_DETECTION_SIGMA/2.0;
            mpu.SetGyroOffsets(dataX.getMean(), dataY.getMean(), dataZ.getMean());
            
            Serial.print("  Gyro X  mean=");
            Serial.print( dataX.getMean() );
            Serial.print("  min= ");
            Serial.print( dataX.getMin() ); 
            Serial.print("  max= ");
            Serial.print( dataX.getMax());
            Serial.print("  std= ");
            Serial.println(dataX.getStd());
            Serial.print("  xStableMin= ");
            Serial.println(xStableMin);
            Serial.print("  xStableMax= ");
            Serial.println(xStableMax);
            
            Serial.print("  GyroY  mean=");
            Serial.print( dataY.getMean() );
            Serial.print("  min= ");
            Serial.print( dataY.getMin() ); 
            Serial.print("  max= ");
            Serial.print( dataY.getMax());
            Serial.print("  std= ");
            Serial.println(dataY.getStd());
            Serial.print("  yStableMin= ");
            Serial.println(yStableMin);
            Serial.print("  yStableMax= ");
            Serial.println(yStableMax);            
            
            Serial.print("  GyroZ  mean=");
            Serial.print( dataZ.getMean() );
            Serial.print("  min= ");
            Serial.print( dataZ.getMin() ); 
            Serial.print("  max= ");
            Serial.print( dataZ.getMax());
            Serial.print("  std= ");
            Serial.println(dataZ.getStd());               
        }
        void Calibrate(int pIterations){
            //dataX.setRejectionSigma(6.0);
            //dataY.setRejectionSigma(6.0);
            int vProgresDiv=100;
            int j=pIterations/vProgresDiv;
            display2.setSegments(napis_CAL);
            display1.setSegments(napis_CAL);
            delay(2000);
            dataX.reset(); dataY.reset(); dataZ.reset();
            for(int i=0;i<pIterations;i++){
                mpu.UpdateRawGyro();
                dataX.checkAndAddReading(mpu.GetRawGyroX());
                dataY.checkAndAddReading(mpu.GetRawGyroY());
                dataZ.checkAndAddReading(mpu.GetRawGyroZ());
                if (i%vProgresDiv==0){
                    display1.showNumberDec(j--);
                }
            }
            CalibrateCommit();
        }

};



class GyroState {
  private:
    boolean fFreeze;
    unsigned long fFreezeTimeStart=0;
    unsigned long fDriftPerMinuteStartTime=0;
    boolean  fApplyDriftCorection=false;
    float fGyroX, fGyroY, fFreezedGyroX;//input gyro angles
    float fAzOffsetX=0, fAzOffsetY=0;//output azimuthal angles
    float fTargetX, fTargetY;
    float fDriftPerMinute=0;
    float movingAvgArr[SMOOTHING_ARRAY_COUNT];    
    float fMovingAvgSum=0;
    int fmovingAvgArr_Idx=0;
  public:
    unsigned long lastPrint=0;

    boolean getFreeze(){
        return fFreeze;
    }
    unsigned long getFreezeTimeStart(){
        return fFreezeTimeStart;
    }
    
    void init(){
        for (int i=0; i<SMOOTHING_ARRAY_COUNT; i++){
          movingAvgArr[i]=0;
        }        
    }

    void setFreezeOn(){
      fFreeze=true;
      redDiode(true);
      digitalWrite(DIODE_PIN, fFreeze);
      fFreezeTimeStart=millis();
      fFreezedGyroX=fGyroX;
      Serial.println("GyroState FREEZ ON  ");
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
      redDiode(false);
    }

    void setGyroCoord(float pX, float pY){
        fGyroX=pX;
        fGyroY=pY;
        if (fmovingAvgArr_Idx>=SMOOTHING_ARRAY_COUNT){
            fmovingAvgArr_Idx=0;
        }
        fMovingAvgSum-=movingAvgArr[fmovingAvgArr_Idx];
        fMovingAvgSum+=pY;
        movingAvgArr[fmovingAvgArr_Idx++]=pY;
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
    
    float getSmoothAzCoordY(){
        //Serial.println("");
        //Serial.print("|   >>> getSmoothAzCoordY: ");
        //Serial.print(" fMovingAvgSum=");
        //Serial.print(fMovingAvgSum);
        //Serial.print(" fAzOffsetY=");
        //Serial.print(fAzOffsetY);
        //Serial.print(" fmovingAvgArr_Idx=");
        //Serial.print(fmovingAvgArr_Idx);
        //Serial.print(" Result=");
        //Serial.println(fMovingAvgSum/float(SMOOTHING_ARRAY_COUNT) + fAzOffsetY);

        return fMovingAvgSum/float(SMOOTHING_ARRAY_COUNT) + fAzOffsetY;
    }
    
    void setTarget(float pX, float pY){
        fTargetX=pX;
        fTargetY=pY;
    }
    
    float getRelativeCoordX(){
        return fTargetX-getAzCoordX();
    }
    
    float getRelativeCoordY(){
        return fTargetY-getAzCoordY();
    }    
    
};

int RS_service(float &pAzimuth, float &pAltitude)
{
  static byte byteHi;
  static byte byteLo;
  static int gInputState = 0;
  static int incomingByte;
  static unsigned gOdczytInteger01;
  static unsigned gOdczytInteger02;  
   
  if (Serial.available() > 0) {
     incomingByte = Serial.read();
     if      (gInputState == 0 && char(incomingByte) == 'g') {
       gInputState++;
     }
     else if (gInputState == 1 && char(incomingByte) == 'o') {
       gInputState++;
     }     
     else if (gInputState == 2 && char(incomingByte) == 't') {
       gInputState++;
     }     
     else if (gInputState == 3 && char(incomingByte) == 'o') {
       gInputState++;
     }
     else if (gInputState == 4) {
       byteHi = incomingByte;
       gInputState++;
     }
     else if (gInputState == 5) {
       byteLo = incomingByte;
       gOdczytInteger01 = byteHi * 256 + byteLo;
       gInputState++;
     }
     else if (gInputState == 6) {
       byteHi = incomingByte;
       gInputState++;
     }
     else if (gInputState == 7) {
       byteLo = incomingByte;
       gOdczytInteger02 = byteHi * 256 + byteLo;
       gInputState++;
     }
     else if (gInputState == 8 && char(incomingByte) == '.') {
       gInputState=0;
       pAzimuth = gOdczytInteger01*360.0/65535.0;
       pAltitude= (gOdczytInteger02*180.0/65535.0-90.0);
       return 1; //sygnal odebrany - mozna go przetwarzac w glownej petli       
     }
   
  }
  return 0;
}

void sendPositionToStellarium(float pAz, float pAl){
   //unsigned vAz=round(pAz*10);
   //int vAl=round(pAl*10);
   Serial.print("AZAL:");
   Serial.print(round(pAz*100.0)/100.0);    
   Serial.print(",");
   Serial.println(round(pAl*100.0)/100.0); 
}

static GyroState gGyroState;
static MotionDetection gMotionDetection;

enum Tmenustate{home, freeze, freeze_enter_coord, freeze_question, pos_set, calibration, home_relative, remote_pressed};
enum TGyroDisplayMode{absolute, relative, settings};

uint8_t diplayData[] = { 0xff, 0xff, 0xff, 0xff };
char   tempInputData[] = { 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0x0 };

class NavigationState{

    private:

        TGyroDisplayMode fDisplayGyroMode=absolute;
        float vCalculatedDrift;
        int vSetAzX, vSetAzY;
        int vEnteredAzX, vEnteredAzY;
        unsigned long int fAutoFreezeStartTime;


    public:
        Tmenustate fMenuState=home;
        int fMenuSubstate=0;
        int fBrightness=0;

        Tmenustate getMenuState(){
            return fMenuState;
        }
        TGyroDisplayMode DisplayGyroMode(){
            return fDisplayGyroMode;
        }


        void showPrompt(TM1637Display pDisplay, int pBufferStartPos){
            for(int i=0;i<5;i++){
                diplayData[i] = SEG_D;
                if (pBufferStartPos+i>8){buzzer(500);}
                tempInputData[pBufferStartPos+i]=0xf;
            }
            pDisplay.setSegments(diplayData);
        }
        
        void  setDisplayGyroMode(TGyroDisplayMode pValue){
            Display1_IVC(0, LEADING_ZERO, 0, 0, false);//reset cache
            Display2_IVC(0, LEADING_ZERO, 0, 0, false);            
            fDisplayGyroMode=pValue;
        }

        int inputDataToIntAndPrint(int pFirstPos, int pLen, TM1637Display pDisplay, int pDefaultValue){
           int vResult=0, v10Pow=pLen-1, vDisplayIdx=0;

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


        void remoteServe(int pKey, Tmenustate pForcedState, float pParam1=0, float pParam2=0)
        {
            int vTmpNum;
            if (pKey==REMOTE_UNKNOWN && pForcedState==remote_pressed){
                return;
            }
            if (pForcedState==fMenuState){
                return;
            }
            if(pKey!=REMOTE_UNKNOWN){
              buzzer(1);
            }

            switch (fMenuState){
                case home:
                    if (pKey==REMOTE_ASTER){
                        fMenuState=freeze;
                        gGyroState.setFreezeOn();
                        setDisplayGyroMode(settings);
                        vSetAzX = gGyroState.getAzCoordX();
                        vSetAzY = gGyroState.getAzCoordY();
                    } else if (pKey==REMOTE_UP){
                        fBrightness=constrain(fBrightness+1,0,3);
                    } else if (pKey==REMOTE_DOWN){
                        fBrightness=constrain(fBrightness-1,0,3);
                    } else if (pKey==REMOTE_HASH){
                      display1.setSegments(napis_CAL);
                      display2.setSegments(napis_CAL);
                      //mpu.Calibrate();
                      vSetAzX = gGyroState.getAzCoordX();
                      gMotionDetection.Calibrate(2000);
                      gGyroState.setDriftPerMinute(0.0);
                      gGyroState.setAzCoordX(vSetAzX);
                    } else if (pForcedState==pos_set){
                        buzzer(30);  
                        gGyroState.setTarget(pParam1, pParam2);
                        fMenuState = home_relative;
                        setDisplayGyroMode(relative);
                    }
                    display1.setBrightness(fBrightness);
                    display2.setBrightness(fBrightness);
                    Display1_IVC(0, LEADING_ZERO, 0, 0, false);//reset display cache to change brightness
                    Display2_IVC(0, LEADING_ZERO, 0, 0, false);                                
                    break;
                case home_relative:
                    if (pKey==REMOTE_ASTER){
                        fMenuState=home;
                        Display1_IVC(0, LEADING_ZERO, 0, 0, false);//reset display
                        Display2_IVC(0, LEADING_ZERO, 0, 0, false);                            
                        setDisplayGyroMode(absolute);
                        fMenuState=home;    
                        redDiode(false);
                    } else if (pForcedState==pos_set){
                        buzzer(30);  
                        gGyroState.setTarget(pParam1, pParam2);
                        Display1_IVC(0, LEADING_ZERO, 0, 0, false);//reset display cache to change brightness
                        Display2_IVC(0, LEADING_ZERO, 0, 0, false);                          
                    }                    
                    break;
                case freeze:
                    if (pKey==REMOTE_ASTER){
                        gGyroState.setFreezeOff();
                        Display1_IVC(0, LEADING_ZERO, 0, 0, false);//reset display
                        Display2_IVC(0, LEADING_ZERO, 0, 0, false);                            
                        setDisplayGyroMode(absolute);
                        fMenuState=home;    
                    } else if (pKey==REMOTE_OK){
                        //buzzer(30);delay(100);buzzer(30);
                        gGyroState.setFreezeOff();
                        gGyroState.setAzCoord(vSetAzX, vSetAzY);
                        Display1_IVC(0, LEADING_ZERO, 0, 0, false);//reset display
                        Display2_IVC(0, LEADING_ZERO, 0, 0, false);                            
                        setDisplayGyroMode(absolute);                        
                        fMenuState=home;  
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
                    } else if (pForcedState==pos_set){
                        vSetAzX = pParam1;
                        vSetAzY = pParam2;
                        buzzer(30);
                        remoteServe(REMOTE_OK, remote_pressed);
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
                        fMenuState=freeze;
                        remoteServe(REMOTE_ASTER, remote_pressed);
                    } else if (pKey==REMOTE_OK){
                        vSetAzX=vEnteredAzX; vSetAzY=vEnteredAzY;
                        fMenuState=freeze;
                        remoteServe(REMOTE_OK, remote_pressed);
                    } else if (pForcedState==pos_set){
                         fMenuState=freeze;
                        remoteServe(-1, pos_set, pParam1, pParam2 );
                    }
                    break;
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

};

static NavigationState gNavigationState;

void encoderSetup(){
  pinMode(enc_2A, INPUT_PULLUP);
  pinMode(enc_2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc_2A), encoder_interrupt_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_2B), encoder_interrupt_callback, CHANGE);    
}

void encoder_interrupt_callback() {
  int encoded2 = (digitalRead(enc_2A) << 1) | digitalRead(enc_2B);
  int sum  = (gLastEncoded2 << 2) | encoded2;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) gEncoderValue2 ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) gEncoderValue2 --;
  gLastEncoded2 = encoded2;
}

float encoder_get_azimuth() {
  long h_deg, h_min, h_seg, A_deg, A_min, A_seg;
  long Az_tel_s;
  float result;
/*
  if (gEncoderValue2 >= pulses_enc2 || gEncoderValue2 <= -pulses_enc2) {
    gEncoderValue2 = 0;
  }
  int enc2 = gEncoderValue2 / 1500;
  long encoder2_temp = gEncoderValue2 - (enc2 * 1500);
  long map2 = enc2 * map(1500, 0, pulses_enc2, 0, 1296000);

  Az_tel_s  = map2 + map (encoder2_temp, 0, pulses_enc2, 0, 1296000);

  if (Az_tel_s < 0) Az_tel_s = 1296000 + Az_tel_s;
  if (Az_tel_s >= 1296000) Az_tel_s = Az_tel_s - 1296000 ;
  return Az_tel_s/ 360.0;
  */
  if (gEncoderValue2<0){
       gEncoderValue2+=pulses_enc2;
  }
  if (gEncoderValue2>=pulses_enc2){
       gEncoderValue2-=pulses_enc2;
  }
  result = map(gEncoderValue2, 0, pulses_enc2, 0, 3600);
  return result;
}


void setup() {
  int i;
  display1.clear();
  display2.clear();
  gGyroState.init();

 
  // Initialization
  mpu.Initialize();

  pinMode(BUZZER_PIN,OUTPUT);//initialize the buzzer pin as an output
  irrecv.enableIRIn(); // uruchamia odbiornik podczerwieni
  pinMode(DIODE_PIN, OUTPUT);//czerwona dioda freeze

  // Calibration
  //Serial.begin(9600);
  Serial.begin(115200);

  display1.setBrightness(gNavigationState.fBrightness);
  display2.setBrightness(gNavigationState.fBrightness);


 //c
  Serial.println("MotionDetection initialize...");
  gMotionDetection.Calibrate(2000);
  display1.clear();
  display2.clear();
  gGyroState.setAzCoord(0,0);
  encoderSetup();    
  gProfilingStartTime=millis();
}



unsigned long gCalibrationStartTime;
unsigned long gLastSentToStellarium=0;
unsigned long gLastPositionRead=0;
unsigned long gLastDisplayCoordinatesTime=0;


void loop() {
    decode_results vIRResults;
    int vTimeToFreeze=0;
    int vMenuState;
    float sGotoAzimuth;
    float sGotoAltitude;    
    float vXposition, vYposition;

    vMenuState = gNavigationState.getMenuState();
    if (irrecv.decode(&vIRResults))
    {
        gNavigationState.remoteServe(remoteDecode(vIRResults), remote_pressed);
        irrecv.resume();
        Serial.print("MENU STATE: ");
        Serial.print(vMenuState);
        Serial.print(" -> ");
        Serial.println(gNavigationState.getMenuState());
        delay(100);
    }

     if ((millis()-gLastPositionRead)>READ_MPU_PERIOD_MS){
        gLastPositionRead=millis();    
        if (RS_service(sGotoAzimuth, sGotoAltitude)){
            Serial.print("|->  Received coordinates: Az=");
            Serial.print(sGotoAzimuth);
            Serial.print(" Alt=");
            Serial.println(sGotoAltitude);
              //gGyroState.setAzCoord(sGotoAzimuth*10.0, sGotoAltitude*10.0);
              //Display1_IVC(gGyroState.getAzCoordY(), LEADING_ZERO, 4, 0, false); //reset cache
              //Display2_IVC(gGyroState.getAzCoordX(), LEADING_ZERO, 4, 0, false);           
            gNavigationState.remoteServe(-1, pos_set, sGotoAzimuth*10, sGotoAltitude*10);  
        }
        mpu.Execute();      
        gGyroState.setGyroCoord(
                //-mpu.GetAngZ()*10,
                encoder_get_azimuth(),
                -mpu.GetAngX()*10);  
        vXposition=gGyroState.getAzCoordX();
        //vYposition=gGyroState.getAzCoordY();
        vYposition=gGyroState.getSmoothAzCoordY();
        
        //Serial.print("|<-  Send coordinates: Az=");
        //Serial.print(vXposition/10.0);
        //Serial.print(" Alt=");
        //Serial.println(vYposition/10.0);     

        if(gNavigationState.DisplayGyroMode()==relative){
            blinkRedDiode(500);
        }
    }
    
    if ((millis()-gLastSentToStellarium)>SEND_STELLARIUM_PERIOD_MS){
        sendPositionToStellarium(vXposition/10.0,vYposition/10.0);
        gLastSentToStellarium=millis();            
    }
    
    if ((millis()-gLastDisplayCoordinatesTime)>DISPLAY_COORDINATES_PERIOD_MS){
        if (gNavigationState.DisplayGyroMode()==absolute){
            Display1_IVC(vYposition, LEADING_ZERO, 4, 0, false);
            Display2_IVC(vXposition, LEADING_ZERO, 4, 0, false);
        } else if (gNavigationState.DisplayGyroMode()==relative){
            Display1_IVC(gGyroState.getRelativeCoordY(), 0, 4, 0, false);
            Display2_IVC(gGyroState.getRelativeCoordX(), 0, 4, 0, false);        
        } 
       gLastDisplayCoordinatesTime=millis();        
    }

    if ((millis()-gGyroState.lastPrint)>10000){
        gGyroState.lastPrint=millis();


        Serial.print("|   AngX = ");
        Serial.print(  mpu.GetAngX());
        Serial.print("  /  AngY = ");
        Serial.print(mpu.GetAngY());
        Serial.print("  /  AngZ = ");
        Serial.println(mpu.GetAngZ());
        Serial.print("|     AccX = ");
        Serial.print(mpu.GetRawGyroX());
        Serial.print("     AccY = ");
        Serial.print(mpu.GetRawGyroY());
        Serial.print("     AccZ = ");
        Serial.println(mpu.GetRawGyroZ());
        Serial.print("|     Profiling = ");
        Serial.println( (gProfilingCount*1000/(millis() - gProfilingStartTime)) );
    }
    gProfilingCount++;
}
