int gInputState=0;

int RS_nasluchwianieObsluga(unsigned &pAzimuth, int &pAltitude)
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
       pAzimuth = round(gOdczytInteger01*360.0/65535.0*10.0);
       pAltitude= round((gOdczytInteger02*180.0/65535.0-90.0)*10.0);
       return 1; //sygnal odebrany - mozna go przetwarzac w glownej petli       
     }
   
  }
  return 0;
}

void sendPositionToStellarium(unsigned pAz, int pAl){
   Serial.print("AZAL:");
   Serial.print(pAz);    
   Serial.print(",");
   Serial.println(pAl); 
}
void setup() {
  Serial.begin(9600);

}

void loop() {
  static unsigned sGotoAzimuth;
  static int sGotoAltitude;
  static int sSendCounter=0;
  bool sWysylajPozycje=false;
  // put your main code here, to run repeatedly:
  if (RS_nasluchwianieObsluga(sGotoAzimuth, sGotoAltitude)){
     sWysylajPozycje=true;
  }
  if(sWysylajPozycje && sSendCounter<100){
      sendPositionToStellarium(sGotoAzimuth, sGotoAltitude);
      sSendCounter++;
      delay(100);      
  } else {
    sWysylajPozycje=false;  
    sSendCounter=false;
  }

  
  
}
