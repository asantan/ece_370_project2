include <LSM303.h>
#include <Wire.h>

#define MOTOR1_PINA 10
#define MOTOR1_PINB 11
#define MOTOR2_PINA 9 
#define MOTOR2_PINB 5 

#define threshold_jerk  0.3
LSM303 imu;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
double oldimu_y; 

bool pickedup = false; 

void setup(){
  Serial.begin(9600);
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.read();
  oldimu_y = (double) imu.a.y*0.061/1000.0; 
  pinMode(MOTOR1_PINA, OUTPUT); 
  pinMode(MOTOR1_PINB, OUTPUT);
  pinMode(MOTOR2_PINA, OUTPUT); 
  pinMode(MOTOR2_PINB, OUTPUT); 
  analogWrite(MOTOR1_PINA, 0);
  analogWrite(MOTOR1_PINA, 0);
  analogWrite(MOTOR2_PINB, 0);
}

int setSpeed(float s){}

void setVelocity(float velocity){}

void checkIMU(){}

void loop(){}
