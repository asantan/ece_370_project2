#include <SPI.h>
#include <Wire.h>
#include <WiFi101.h>
#include <LSM303.h>
#include "secrets.h"
#include <WiFiUdp.h>

#define MOTOR1_PINA 5
#define MOTOR1_PINB 11
#define MOTOR2_PINA -1  //placeholder
#define MOTOR2_PINB -2  //placeholder


#define ir_1  6 
#define ir_2  12

//function prototypes
bool StationaryGyro();
void SetGyroAccMag();
void GetOffsetsAndInitialQuat();
void SetVariables();
void GenerateRotationMatrix();
void GetPitch();
void GetRoll();
void GetYaw();
void GetEuler();
void AHRS();
void InitSensors();
void PollSensors();
void OutputForCalibration();
void imuSetup();
void motorsetup();
void apsetup();
void irsetup();
void tickinterrupt();
int setSpeed(float s);
void setVelocity(float velocity); 
void readUDP();
//end function prototypes 

//imu chip
LSM303 imu;

//tracking current state 
enum statelist {NONE, TOP, BOTTOM, BOTH};
uint8_t state = NONE; 
uint8_t lastState = NONE;
unsigned long tick = 0; 
unsigned long tock = 0; 
unsigned long timelapsed = tick - tock;

// track position of encoder
volatile long encoderPosition = 0; 
volatile long tickCount = 0; 
float currentAngle = 0; 


//track direction of wheel 
int currentDirection = CLOCKWISE; 
float desiredPosition;

//used for tracking motor speed
float vel = 0.0; 

//Proportional constant for feedback loop
int kp = 2; 

//end motor servoing variables

//wifi variables
char ssid[] = SECRET_SSID; 
char pass[] = SECRET_PASS; 
int status = WL_IDLE_STATUS; 
WiFiUDP Udp; 
WiFiServer  server(80); 
//end wifi variables 

void setup() {
  Serial.begin(9600);
  Serial.println("Keeping imu still and level during startup will yield best results");
  Wire.begin();
  Wire.setClock(400000); //value from pololu imu github
  SPI.begin(); 
  apsetup();
  irsetup();
  motorsetup();
  imuSetup();
}
 
void imuSetup(){
  if(imu.init() == false){
    Serial.println("LSM303 imu init failure");
  }
  Serial.println("imu detected");
  
  switch ((int) imu.getDeviceType()) {
  case imu.device_DLH:
    Serial.print("device_DLH");
    break;
  case imu.device_DLM:
    Serial.print("device_DLM");
    break;
  case imu.device_DLHC:
    Serial.print("device_DLHC");
    break;
  case imu.device_D:
    Serial.println("device_D");
    break;
  default:
    Serial.print("invalid LSM303 type");
    break;
}
}

void irsetup(){
  pinMode(ir_1, INPUT);
  pinMode(ir_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ir_1), tickInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(ir_2), tickInterrupt, RISING);
}

void motorsetup(){
  pinMode(MOTOR1_PINA, OUTPUT); 
  pinMode(MOTOR1_PINB, OUTPUT);
  pinMode(MOTOR2_PINA, OUTPUT); 
  pinMode(MOTOR2_PINB, OUTPUT); 
}

void apsetup(){
  WiFi.setPins(8, 7, 4, 2);
  Serial.println("Acces point pins set");
  if(WiFi.status() == WL_NO_SHIELD){
    Serial.println("WiFi shield not present");
    while(true); //stop program
  }

  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  status = WiFi.beginAP(ssid);
  if(status != WL_AP_LISTENING){
    Serial.println("Creating AP failed");
    while(true); //stop program
  }
  delay(10000); 
  server.begin();
  printWiFiStatus();
  Serial.println();
  attachInterrupt(digitalPinToInterrupt(UDP_PORT), readUDP, RISING); 
  Udp.begin(UDP_PORT);
  
}

void readUDP(){
  int pktSize = Udp.parsePacket();
  if(packetSize){
    
  }
}
void tickInterrupt()
{
  //ir1 is MSB
  tick = millis();
  int ir1 = digitalRead(ir_1);
  int ir2 = digitalRead(ir_2); 
  Serial.println("IR 1: " + String(ir1));
  Serial.println("IR 2: " + String(ir2));
  if(ir1 == '1' && ir2 == '1') state = BOTH;
  else if(ir1 == 1 && ir2 == 0) state = TOP;
  else if(ir1 == 0 && ir2 == 1) state = BOTTOM; 
  else state = NONE; 

  /*switch(state){ //determine direction from previous state
    tick = millis();
    case NONE:
      if(lastState == TOP) currentDirection = CLOCKWISE; 
      else currentDirection = COUNTERCLOCKWISE; 
      break;
    case TOP:
      if(lastState == BOTH) currentDirection = CLOCKWISE; 
      else currentDirection = COUNTERCLOCKWISE;
   
      break; 
    case BOTTOM:
      if(lastState == NONE) currentDirection = CLOCKWISE; 
      else currentDirection = COUNTERCLOCKWISE;  
      break;
    case BOTH: 
      if(lastState == BOTTOM) currentDirection = CLOCKWISE; 
      else currentDirection = COUNTERCLOCKWISE; 
      break; 
  }
  lastState = state; //update last state
  */
  currentAngle += currentDirection*2*degrees_per_tick; //calculate angle traveled
  Serial.print("Calculated angle ");
  Serial.println(currentAngle);
  if((4*degrees_per_tick + desiredPosition) >= currentAngle && currentAngle >= (desiredPosition - 4*degrees_per_tick))
      {
        setVelocity(0); 
        Serial.println("Stop");
 
      } 
  tock = millis();
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
  int sp = setSpeed(s); 
    
  if(velocity >= 0){ //forward
    analogWrite(MOTOR1_PINB, 0);
    //digitalWrite(MOTOR1_PINB, LOW); //important to set low first to avoid shorting
    analogWrite(MOTOR1_PINA, sp);
  }
  else { //backward
    analogWrite(MOTOR1_PINA, 0);
    //digitalWrite(PINA, LOW);
    analogWrite(MOTOR1_PINB, sp);    
   
  }
}


void loop() {
  // put your main code here, to run repeatedly:

}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.println(ip);

}
