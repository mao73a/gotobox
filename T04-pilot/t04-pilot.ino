#define ESP32
#include <IRremote.h>       // biblioteka

IRrecv irrecv(7); 
decode_results gIRResults;
#define BUZZER_PIN 12

#define REMOTE_UNKNOWN -1
#define REMOTE_0 0
#define REMOTE_1 1
#define REMOTE_2 2
#define REMOTE_3 3
#define REMOTE_4 4
#define REMOTE_5 5
#define REMOTE_6 6
#define REMOTE_7 7
#define REMOTE_8 8
#define REMOTE_9 9

#define REMOTE_ASTER 10
#define REMOTE_HASH 11
#define REMOTE_LEFT 12
#define REMOTE_RIGHT 13
#define REMOTE_UP 14
#define REMOTE_DOWN 15
#define REMOTE_OK 16

void buzzer(int pCzas=1){
  digitalWrite(BUZZER_PIN,HIGH);
  delay(pCzas);
  digitalWrite(BUZZER_PIN,LOW);
}

void setup()
{
   Serial.begin(9600);
   Serial.print("Start!");       
   pinMode(BUZZER_PIN,OUTPUT);//initialize the buzzer pin as an output
   irrecv.enableIRIn(); // uruchamia odbiornik podczerwieni
}


int remoteDecode()
{
    int vResult=REMOTE_UNKNOWN;
     
    if (gIRResults.decode_type == NEC)
    {
        switch(gIRResults.value){
            case 0xFF38C7: 
                vResult= REMOTE_OK; 
                break;
            case 0xFF6897: 
                vResult=REMOTE_ASTER;
                break;
            case 0xFFB04F: 
                vResult=REMOTE_HASH;    
                break;
                
            case 0xFF10EF: 
                vResult=REMOTE_LEFT;                
                break;
            case 0xFF5AA5: 
                vResult=REMOTE_RIGHT;                
                break;
            case 0xFF18E7: 
                vResult=REMOTE_UP;                
                break;
            case 0xFF4AB5: 
                vResult=REMOTE_DOWN;
                break;
                
            case 0xFF9867: 
                vResult=REMOTE_0;
                break;
            case 0xFFA25D: 
                vResult=REMOTE_1;
                break;
            case 0xFF629D: 
                vResult=REMOTE_2;
                break;
            case 0xFFE21D: 
                vResult=REMOTE_3;
                break;
            case 0xFF22DD: 
                vResult=REMOTE_4;
                break;
            case 0xFF02FD: 
                vResult=REMOTE_5;
                break;
            case 0xFFC23D: 
                vResult=REMOTE_6;
                break;
            case 0xFFE01F: 
                vResult=REMOTE_7;                
                break;
            case 0xFFA857: 
                vResult=REMOTE_8;
                break;
            case 0xFF906F: 
                vResult=REMOTE_9;
                break;
        }
    }

    if(vResult!=REMOTE_UNKNOWN){
      buzzer(1);
    }
    return vResult;
}

void remoteServe(int pKey)
{
 //Serial.print(" remoteServe");   
 //Serial.println(pKey); 
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

void loop()
{
  if (irrecv.decode(&gIRResults)) // sprawdza, czy otrzymano sygnał IR
  { 
        remoteServe(remoteDecode());
        //Serial.print(gIRResults.value);
        //Serial.print(" ");        
        //Serial.println(gIRResults.value, HEX); 
        irrecv.resume(); // odbiera następną wartość
        delay(100);
  }
}
