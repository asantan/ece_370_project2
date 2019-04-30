#include <LSM303.h>
#include <Wire.h>
#include <math.h>

#define MOTOR1_PINA 10
#define MOTOR1_PINB 11
#define MOTOR2_PINA 9 
#define MOTOR2_PINB 5 

#define angleTolerance 10 //+- five degrees
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
void moveToAngle(float targetAngle){ //angle should be given in radians 
  imu.read(); 
  int startSpeed = 0.5; 
  double currentAngle = 1/tan((imu.m.y*toGauss)/(imu.m.x*toGauss))*(180.0/pi); //convert to degrees
  int error = targetAngle - currentAngle;
  Serial.print(" Initial Actual Angle: ");
  Serial.print(currentAngle);
  Serial.print("Initial Calculated error: ");
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
    
    if(error > 0){
      Serial.println("Positive error");
      analogWrite(MOTOR1_PINB, 0);
      analogWrite(MOTOR1_PINA, rotate_speed);
      analogWrite(MOTOR2_PINA, 0);
      analogWrite(MOTOR2_PINB, rotate_speed); 
    }
    else{
      Serial.println("Negative error");
      analogWrite(MOTOR1_PINA, 0);
      analogWrite(MOTOR1_PINB, rotate_speed);
      analogWrite(MOTOR2_PINB, 0);
      analogWrite(MOTOR2_PINA, rotate_speed); 
    }
    imu.read();
    currentAngle = 1/tan((imu.m.y*toGauss)/(imu.m.x*toGauss))*(180.0/pi); 
    error = targetAngle - currentAngle;
  }
  Serial.println("Target angle reached");
  analogWrite(MOTOR1_PINA, 0);
  analogWrite(MOTOR1_PINB, 0);
  analogWrite(MOTOR2_PINB, 0);
  analogWrite(MOTOR2_PINA, 0);
}

void rotate(int numRotations){ //number of times to rotate based on desired angle
  
}

void setAngle(int desiredAngle){ //angle assumed to be received in degrees
  //int rotations = (int) desiredAngle / 360; 
  //rotate(rotations); 
  int angle = (int) desiredAngle%360;
  Serial.print("Angle after moduloing with 360: ");
  Serial.print(angle);
  Serial.println(" degrees");
  double angleInRad = desiredAngle*(pi/180.0); 
  moveToAngle(angleInRad);
  
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
