#include <LSM303.h>
#include <Wire.h>
#include <WiFi101.h>
#include "secrets.h"

#define MOTOR1_PINA 10
#define MOTOR1_PINB 11
#define MOTOR2_PINA 9 
#define MOTOR2_PINB 5 

char ssid[] = SECRET_SSID;  // network SSID)
char pass[] = SECRET_PASS;  //network password

int status = WL_IDLE_STATUS; 
WiFiServer(80); 

void setup() {

}

void APsetup(){
  WiFi.setPins(8, 7, 4, 2);
  while(!Serial);
  if(WiFi.status() == WL_NO_SHIELD){
    //stop program
    while(true);
  }

  //connect to WiFi network
  while(status != WL_CONNECTED){
    Serial.println("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);
    status = WiFi.begin(ssid, pass);
    //wait 10 seconds
    delay(10000); 
  }

  Serial.println("Connected to wifi");
  printWiFiStatus();

  Serial.println("\nStarting connection to server");
  //if connection received report via serial
  Udp.begin(UDP_PORT);
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

void loop() {
  // put your main code here, to run repeatedly:

}
