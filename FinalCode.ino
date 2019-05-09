#include <LSM303.h>
#include  <WiFiUdp.h>
#include <WiFi101.h>
#include "secrets.h"
#include <Wire.h>
#include <math.h>
#include <SPI.h>

#define MOTOR1_PINA 10
#define MOTOR1_PINB 11
#define MOTOR2_PINA 9 
#define MOTOR2_PINB 5 

#define angleTolerance 10 //+- degrees
#define rotate_speed 0.3

#define Kp  2

#define pi 3.14159

#define threshold_jerk  0.2


#define ir_left 6  //ir pin number
#define ir_right 12 //ir pin number

#define radius    6 //wheel radius in cm
#define baseline  6 //baseline length in cm 
#define pi      3.1415926535
#define two_pi  6.2831850718 

#define tick_per_revolution 2 //number is not considered accurate right now
#define revolutions_per_360 80 
   
const float degrees_per_tick = 360/(tick_per_revolution*revolutions_per_360);
const float radians_per_tick = degrees_per_tick*(pi/180); //change in degree per tick
const float arclength = radius*radians_per_tick;    //Arclength can be viewed as line bc of small angle change
const float delta_phi_per_tick = 1/(tan(arclength/baseline));    //angle between current position and origin


int currentDirection = 0;

float phi_global = 0.0; 
float x_global = 0.0; 
float y_global = 0.0; 
 
LSM303 imu;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
double oldimu_y; 

const double toGauss = 0.061/1000.0; 


char ssid[] = SECRET_SSID;  // network SSID)
char pass[] = SECRET_PASS;  //network password

bool pickedup = false; 

int status = WL_IDLE_STATUS; 
WiFiServer server(80); 

WiFiUDP Udp; 


void setup() {
  Serial.begin(9600);
  Wire.begin();
  motorsetup();
  imusetup();
  APsetup();
  imu.init();
  imu.enableDefault();
  imu.read();
  oldimu_y = (double)imu.a.y*0.061/1000.0;

}

void APsetup(){
  WiFi.setPins(8, 7, 4, 2);
  while(!Serial);
  if(WiFi.status() == WL_NO_SHIELD){
    //stop program
    while(true);
  }

  // by default the local IP address of will be 192.168.1.1
  // you can override it with the following:
  //WiFi.config(IPAddress(10, 0, 0, 30));
  
  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  status = WiFi.beginAP(ssid);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }
  //wait 10 seconds for connection
  delay(10000);
  //start web server on port 80
  printWiFiStatus(); 
  server.begin();
  Serial.print("\nListening for UDP packets on port ");
  Serial.println(UDP_PORT_LISTEN); 
  Serial.print("Result of UDP.begin ");
  Serial.println(Udp.begin(UDP_PORT_LISTEN));
}

void imusetup(){
  imu.init();
  imu.enableDefault();
  imu.read();
  //moveToAngle(0); 
  oldimu_y = (double) imu.a.y*0.061/1000.0;
}

void irsetup(){
  attachInterrupt(digitalPinToInterrupt(ir_left), left_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ir_right), right_tick, RISING);
}

void left_tick(){ //tick detected
  float local_phi = -delta_phi_per_tick;
  float delta_x = (baseline/2)*sin(local_phi);         //x and y are change along origin
  float delta_y = (baseline/2) - (baseline/2)*cos(local_phi); //sign of phi is important 
  float delta_x_prime = delta_x*cos(phi_global) + delta_y*sin(phi_global);
  float delta_y_prime = delta_x*sin(phi_global) + delta_y*cos(phi_global);
  rise_l(local_phi, delta_x_prime, delta_y_prime);
  
}

//calculates distance covered by right weel
void right_tick(){
  float local_phi = delta_phi_per_tick;
  float delta_x = (baseline/2)*sin(local_phi);         //x and y are change along origin
  float delta_y = (baseline/2) - (baseline/2)*cos(local_phi); //sign of phi is important 
  float delta_x_prime = delta_x*cos(phi_global) + delta_y*sin(phi_global);
  float delta_y_prime = delta_x*sin(phi_global) + delta_y*cos(phi_global);
  rise_r(local_phi, delta_x_prime, delta_y_prime);
}

//update global position based off movement of right wheel
void rise_r(float delta_phi, float delta_x_prime, float delta_y_prime){
  phi_global = phi_global + delta_phi; 
  x_global = x_global + delta_x_prime; //x and y are global change vs in reference 
  y_global = y_global + delta_y_prime; //to robot
  printGlobalPosition();
}

//update global position based off movement of left wheel
void rise_l(float phi, float delta_x_prime, float delta_y_prime){
  phi_global = phi_global - phi; 
  x_global = x_global + delta_x_prime; 
  y_global = y_global - delta_y_prime; 
  printGlobalPosition();
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
  
  while(error > angleTolerance){
    
    if(mod_diff < 180.0){//counterclockwise
      imu.read(); 
      error = targetAngle-imu.heading(); 
      setAngularVelocity(-rotate_speed);
    }
    else{ //clockwise
      imu.read();
      error = imu.heading() - targetAngle; 
      setAngularVelocity(rotate_speed);
    }
    mod_diff = (int)error % 360; // %make error between 0 (inclusive) and 360.0 (exclusive)
    dist = mod_diff > 180.0 ? 360.0 - mod_diff: mod_diff; //for rollovers
    Serial.print("Current Angle: ");
    Serial.print(currentAngle);
    Serial.print(" Error: ");
    Serial.println(error);
  }
  motorsOff(); 
  
}

void printGlobalPosition(){
  Serial.print("Global phi ");
  Serial.println(phi_global);
  Serial.print("X location ");
  Serial.println(x_global);
  Serial.print("Y location");
  Serial.println(y_global);
}

void loop() {
 int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      udp_recv udp; 
      memset(&udp, 0, sizeof(udp));
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      IPAddress remoteIp = Udp.remoteIP();
      Serial.print(remoteIp);
      Serial.print(", port ");
      Serial.println(Udp.remotePort());

      // read the packet into packetBufffer
      int len = Udp.read((byte *) &udp, 255);
      //int len = Udp.read(packetBuffer, 255);
      Serial.println("Contents:");
      //Serial.println(packetBuffer); 
      Serial.println(udp.velocity);
      Serial.println(udp.theta);
      Serial.println(udp.rst); 
      setVelocity(udp.velocity);
  }
  //checkIMU();
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
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}
