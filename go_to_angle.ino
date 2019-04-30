#include <LSM303.h>
#include <Wire.h>
#include <math.h>

#define MOTOR1_PINA 10
#define MOTOR1_PINB 11
#define MOTOR2_PINA 9 
#define MOTOR2_PINB 5 

#define angleTolerance 10 //+- five degrees
#define rotate_speed 0.4

#define pi 3.14159

#define threshold_jerk  0.3
LSM303 imu;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
double oldimu_y; 
double current_angle = 0; 

void setup() {
  Serial.begin(9600);
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.read();
  oldimu_y = (double)imu.a.y*0.061/1000.0;
}

void moveToAngle(float targetAngle){ //angle should be given in radians 
  imu.read(); 
  double currentAngle = 1/tan(imu.m.y/imu.m.x)*(180.0/pi); //convert to degrees
  int difference = targetAngle - currentAngle; 
  
  while(difference > angleTolerance){
    
  }
}

void rotate(int numRotations){ //number of times to rotate based on desired angle
  
}
void setAngle(double desiredAngle){ //angle assumed to be received in degrees
  int rotations = (int) desiredAngle / 360; 
  rotate(rotations); 
  int angle = (int) desiredAngle%360;
  double angleInRad = desiredAngle*(pi/180.0); 
  moveToAngle(angleInRad);
  
}
void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    
  }
}
