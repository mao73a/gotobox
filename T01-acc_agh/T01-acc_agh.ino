#include "Wire.h"
#include "Math.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelerometer;

const float pi = 3.141592; 
const int sample_no = 400; // no of samples for aproximation
int16_t ax, ay, az;
float x, y, z;
int  sample;
float _angle_x, angle_x, _angle_y, angle_y;
long ax_sum, ay_sum, az_sum;

void setup() {
    Wire.begin();
    Serial.begin(9600);
    accelerometer.initialize();
    if (accelerometer.testConnection());
    Serial.println("MPU 6050 connection OK...");  
}

void loop() {
    accelerometer.getAcceleration(&ax, &ay, &az);
    ax_sum = ax_sum + ax;
    ay_sum = ay_sum + ay;
    az_sum = az_sum + az; 
    sample++;
      
    if (sample == sample_no)
    {
     // mean values
     x = ax_sum/sample_no;
     y = ay_sum/sample_no;
     z = az_sum/sample_no;
     
     // Calculate of roll and pitch in deg
     angle_x = atan2(x, sqrt(square(y) + square(z)))/(pi/180);
     angle_y = atan2(y, sqrt(square(x) + square(z)))/(pi/180);
     
     // Reset values for next aproximation   
     sample=0;
     ax_sum = 0;
     ay_sum = 0;
     az_sum = 0;
        
    Serial.print(angle_x);
    Serial.print("\t"); // \t = tablator 
    Serial.println(round(angle_y*10.0)/10.0);
    }
}
