#include <LSM303.h>
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.read();
  oldimu_y = (double)imu.a.y*0.061/1000.0;
}

int setSpeed(float s){
  if(s > 1.0) s = 1.0; 
  if(s < 0.0) s = 0.0; 
  int out = (int) (s*255.0); 
  return out; 
}


void setVelocity(float velocity){
  float s = velocity; 
  if(s < 0) s = -s; 
  int sp1 = setSpeed(s);  //speed motor 1
  int sp2 = setSpeed(s);                //speed motor 2
  if(!pickedup){
    if(velocity >= 0){ //forward  
      analogWrite(MOTOR1_PINB, 0);
      analogWrite(MOTOR1_PINA, sp1);
      analogWrite(MOTOR2_PINA, 0);
      analogWrite(MOTOR2_PINB, sp2);
    }
    else { //backward
      analogWrite(MOTOR1_PINA, 0);
      analogWrite(MOTOR1_PINB, sp1);
      analogWrite(MOTOR2_PINB, 0);
      analogWrite(MOTOR2_PINA, sp2);
    }
  }
  else{
      analogWrite(MOTOR1_PINA, 0);
      analogWrite(MOTOR1_PINB, 0);
      analogWrite(MOTOR2_PINB, 0);
      analogWrite(MOTOR2_PINA, 0);
  }
}


void checkIMU(){
  imu.read();
  double changeIn = (double)imu.a.y*0.061/1000.0;
  double jerk = abs(oldimu_y - changeIn);
  if(jerk > threshold_jerk){
    Serial.println("Pickup detected");
    pickedup = true; 
  }
 
  oldimu_y = (double)imu.a.y*0.061/1000.0;
}


void loop() {
  // put your main code here, to run repeatedly:
  checkIMU();
  setVelocity(0.5); 
}
