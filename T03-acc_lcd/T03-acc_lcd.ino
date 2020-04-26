/*
 *  Mandatory includes
 */
#include <Arduino.h>
#include <TinyMPU6050.h>
#include <TM1637Display.h>

#define CLK1 2
#define DIO1 3
#define CLK2 4
#define DIO2 5

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

TM1637Display display1 = TM1637Display(CLK1, DIO1);
TM1637Display display2 = TM1637Display(CLK2, DIO2);
MPU6050 mpu (Wire);

/*
 *  Setup
 */
void setup() {
 int i;
  // Initialization
  mpu.Initialize();

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