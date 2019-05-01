#include <LSM303.h>
#include <Wire.h>
#include <math.h>

#define MOTOR1_PINA 10
#define MOTOR1_PINB 11
#define MOTOR2_PINA 9 
#define MOTOR2_PINB 5 

#define angleTolerance 15 //+- degrees
#define rotate_speed 0.3

#define Kp  2

#define pi 3.14159


LSM303 imu;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
double oldimu_y; 
double current_angle = 0; 
const double toGauss = 0.061/1000.0; 

void setup() {
  Serial.begin(9600);
  motorsetup();
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.read();
  oldimu_y = (double)imu.a.y*0.061/1000.0;
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


void moveToAngle(double targetAngle){ //angle should be given in degrees 
  imu.read();
  double rawDiff =  
  double currentAngle = (((double)imu.m.x)*toGauss*(180.0/pi));; //convert to degrees
  int error = targetAngle - currentAngle;
  Serial.print(" Initial Actual Angle: ");
  Serial.print(currentAngle);
  Serial.print(" Initial Calculated error: ");
  Serial.print(error); 
  Serial.print(" Angle tolerance: "); 
  Serial.println(angleTolerance);
  while(abs(error) > angleTolerance){
    Serial.print("Target Angle: ");
    Serial.print(targetAngle); 
    Serial.print(" Actual Angle: ");
    Serial.print(currentAngle);
    Serial.print(" Calculated errror: ");
    Serial.println(error); 
    
    if(error > 0){ //move clockwise
      Serial.println("Positive error");
      setAngularVelocity(rotate_speed); 
    }
    else{ //move counterclockwise
      setAngularVelocity(-rotate_speed); 
    }
    imu.read();
    currentAngle = ((double)imu.m.x*toGauss*(180.0/pi)); //to degrees
    error = currentAngle - targetAngle;
  }
  Serial.println("Target angle reached"); 
  Serial.print("Final Actual Angle: ");
  Serial.print(currentAngle);
  Serial.print(" Final 0Calculated errror: ");
  Serial.println(error); 
  analogWrite(MOTOR1_PINA, 0);
  analogWrite(MOTOR1_PINB, 0);
  analogWrite(MOTOR2_PINB, 0);
  analogWrite(MOTOR2_PINA, 0);
}

void setAngle(double desiredAngle){ //angle assumed to be received in degrees
  //int rotations = (int) desiredAngle / 360; 
  //rotate(rotations);
  moveToAngle(desiredAngle);
  
}
void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()){
    int desiredAngle = Serial.parseInt();
    Serial.print("Received angle ");
    Serial.println(desiredAngle);
    setAngle(desiredAngle); 
  }
}
