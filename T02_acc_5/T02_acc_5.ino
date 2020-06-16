#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro(0x69);

int16_t ax, ay, az, gx, gy, gz;

double timeStep, time, timePrev;
double arx, ary, arz, grx, gry, grz, gsx, gsy, gsz, rx, ry, rz;

int i;
double gyroScale = 131;

void setup() {

  Wire.begin();
  Serial.begin(9600);
  accelgyro.initialize();

  time = millis();

  i = 1;

}

void loop() {

  // set up time for integration
  timePrev = time;
  time = millis();
  timeStep = (time - timePrev) / 1000; // time-step in s

  // collect readings
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // apply gyro scale from datasheet
  gsx = gx/gyroScale;   gsy = gy/gyroScale;   gsz = gz/gyroScale;

  // calculate accelerometer angles
    arx = (180/3.141592) * atan(ax / sq(square(ay) + sq(az))); 
    ary = (180/3.141592) * atan(ay / sq(square(ax) + sq(az)));
    arz = (180/3.141592) * atan(sqrt(sq(ay) + sq(ax)) / az);    

  // set initial values equal to accel values
  if (i == 1) {
    grx = arx;
    gry = ary;
    grz = arz;
  }
  // integrate to find the gyro angle
  else{
    grx = grx + (timeStep * gsx);
    gry = gry + (timeStep * gsy);
    grz = grz + (timeStep * gsz);
  }  

  // apply filter
    rx = (0.96 * arx) + (0.04 * grx);
    ry = (0.96 * ary) + (0.04 * gry);
    rz = (0.96 * arz) + (0.04 * grz);

  // print result
  Serial.print(rx);   Serial.print("\t");
  Serial.print(ry);   Serial.print("\t");
  Serial.println(rz);

  i = i + 1;
  delay(200);

}