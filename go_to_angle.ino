#include <LSM303.h>
#include <Wire.h>
#include <math.h>

#define MOTOR1_PINA 10
#define MOTOR1_PINB 11
#define MOTOR2_PINA 9 
#define MOTOR2_PINB 5 

#define angleTolerance 10 //+- degrees
#define rotate_speed 0.3

#define Kp  2

#define pi 3.14159


LSM303 imu;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
const double toGauss = 0.061/1000.0; 

void setup() {
  Serial.begin(9600);
  motorsetup();
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.read();
}

void motorsetup(){
  pinMode(MOTOR1_PINA, OUTPUT); 
  pinMode(MOTOR1_PINB, OUTPUT);
  pinMode(MOTOR2_PINA, OUTPUT); 
  pinMode(MOTOR2_PINB, OUTPUT); 
  analogWrite(MOTOR1_PINA, 0);
  analogWrite(MOTOR1_PINA, 0);
  analogWrite(MOTOR2_PINB, 0);
  analogWrite(MOTOR2_PINB, 0);
}

int setSpeed(float s){
  if(s > 1.0) s = 1.0; 
  if(s < 0.0) s = 0.0; 
  int out = (int) (s*255.0); 
  return out; 
}

void setAngularVelocity(float velocity){
  float s = velocity; 
  if(s < 0) s = -s; 
  int sp1 = setSpeed(s);  //speed motor 1
  int sp2 = setSpeed(s);  //speed motor 2
  if(velocity < 0){ //turn clockwise  
    analogWrite(MOTOR1_PINB, 0);
    analogWrite(MOTOR1_PINA, sp1);
    analogWrite(MOTOR2_PINB, 0);
    analogWrite(MOTOR2_PINA, sp2);
  }
  else { //turn counterclockwise
    analogWrite(MOTOR1_PINA, 0);
    analogWrite(MOTOR1_PINB, sp1);
    analogWrite(MOTOR2_PINA, 0);
    analogWrite(MOTOR2_PINB, sp2);
  }
}

void motorsOff(){
  analogWrite(MOTOR1_PINA, 0);
  analogWrite(MOTOR1_PINB, 0);
  analogWrite(MOTOR2_PINA, 0);
  analogWrite(MOTOR2_PINB, 0);
}

void moveToAngle(int targetAngle){ //angle should be given in degrees 
  imu.read(); 
  double currentAngle = imu.heading(); //convert to degrees
  double error = (currentAngle > targetAngle) ? currentAngle - targetAngle : targetAngle - currentAngle; 
  int mod_diff = (int)error % 360; // %make error between 0 (inclusive) and 360.0 (exclusive)
  double dist = mod_diff > 180.0 ? 360.0 - mod_diff: mod_diff; //for rollovers
  Serial.print("Mod_diff: ");
  Serial.print(mod_diff);
  Serial.print(" Current Angle: ");
  Serial.println(currentAngle);
  Serial.print(" Distance to target ");
  Serial.println(dist);
  while(dist > angleTolerance){
    
    if(mod_diff > 180.0){//clockwise
      imu.read(); 
      dist = mod_diff;  
      setAngularVelocity(rotate_speed);
    }
    else{ //clockwise
      imu.read();
      dist = targetAngle - imu.heading(); 
      setAngularVelocity(-rotate_speed);
    }
    mod_diff = (int)error % 360; // %make error between 0 (inclusive) and 360.0 (exclusive)
    Serial.print("Current Angle: ");
    imu.read();
    
    Serial.print(imu.heading());
    Serial.print(" Error: ");
    Serial.println(error);
    Serial.print(" Distance to target ");
    Serial.println(dist);
  }
  Serial.print(" Final Error: ");
  Serial.println(error);
  motorsOff(); 
  
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()){
    int desiredAngle = Serial.parseInt();
    imu.read();
    Serial.print("Angle when recv ");
    Serial.println(imu.heading());
    Serial.print("Received angle ");
    Serial.println(desiredAngle);
    moveToAngle(desiredAngle); 
  }
}
