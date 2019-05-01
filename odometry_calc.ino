#include <SPI.h>
#include <BasicLinearAlgebra.h>

#define CLOCKWISE -1
#define COUNTERCLOCKWISE 1

#define MOTOR1_PINA 10
#define MOTOR1_PINB 11
#define MOTOR2_PINA 9 
#define MOTOR2_PINB 5 

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
const float delta_rad_per_tick = delta_phi_per_tick*(pi/180);   //angle between current position and origin
//bool writing = false; 


float phi_global = 0.0; 
float x_global = 0.0; 
float y_global = 0.0; 

float phi_global_matrix = 0.0; 
float x_global_matrix = 0.0; 
float y_global_matrix = 0.0; 

//BasicLinearAlgebra is wrapped in namespace BLA
//BLA specifies you are using BasicLinearAlgebra Matrix
BLA:: Matrix<4,4> r_rot = {
                      0, 0, 0, 0, 
                      0, 0, 0, baseline/2, 
                      0, 0, 0, 0,
                      0, 0, 0, 1
                    };


BLA:: Matrix<4,4> l_rot = {
                      0, 0, 0, 0, 
                      0, 0, 0, -baseline/2, 
                      0, 0, 0, 0,
                      0, 0, 0, 1
                    };


BLA:: Matrix<4,4> r_trans = {
                      1, 0, 0, 0, 
                      0, 1, 0, -baseline, 
                      0, 0, 1, 0,
                      0, 0, 0, 1
                    };


BLA:: Matrix<4,4> l_trans = {
                      1, 0, 0, 0, 
                      0, 1, 0, baseline, 
                      0, 0, 1, 0,
                      0, 0, 0, 1
                    };

BLA:: Matrix<4, 4> T = {
                    cos(phi_global_matrix), -sin(phi_global_matrix), 0, x_global_matrix,
                    sin(phi_global_matrix), cos(phi_global_matrix), 0, y_global_matrix, 
                    0, 0, 1, 0, 
                    0, 0, 0, 1 
                    
                    
                    };
 

int currentDirection = 0;

void setup(){
  Serial.begin(9600);
  SPI.begin();
  pinMode(MOTOR1_PINA, OUTPUT); 
  pinMode(MOTOR1_PINB, OUTPUT);
  pinMode(MOTOR2_PINA, OUTPUT); 
  pinMode(MOTOR2_PINB, OUTPUT); 
  analogWrite(MOTOR1_PINA, 0);
  analogWrite(MOTOR1_PINA, 0);
  analogWrite(MOTOR2_PINB, 0);
  analogWrite(MOTOR2_PINB, 0);

  attachInterrupt(digitalPinToInterrupt(ir_left), left_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ir_right), right_tick, RISING);
}

//calculates distance covered by left wheel. Local change
void left_tick(){ //tick detected
  float local_phi = -delta_rad_per_tick;
  float delta_x = (baseline/2)*sin(local_phi);         //x and y are change along origin
  float delta_y = (baseline/2) - (baseline/2)*cos(local_phi); //sign of phi is important 
  //rise_l(local_phi, delta_x, delta_y); 
  matrix_l(); //matrix method
}

//calculates distance covered by right weel. Local Change
void right_tick(){
  float local_phi = delta_rad_per_tick;
  float delta_x = (baseline/2)*sin(local_phi);         //x and y are change along origin
  float delta_y = (baseline/2) - (baseline/2)*cos(local_phi); //sign of phi is important 
  //rise_r(local_phi, delta_x, delta_y); //analytical method
  matrix_r();//matrix method
}

/* 
 * begin section for analytical method of obtaining x, y, theta_z. Update global change in position
 */

//update global position based off movement of right wheel
void rise_r(float delta_phi, float delta_x, float delta_y){
  phi_global = phi_global + delta_phi; 
  delay(10); 
  float delta_x_prime = delta_x*cos(phi_global) + delta_y*sin(phi_global); //global change
  float delta_y_prime = delta_x*sin(phi_global) + delta_y*cos(phi_global); //global change
  x_global = x_global + delta_x_prime; //x and y are global change vs in reference 
  y_global = y_global + delta_y_prime; //to robot
  printGlobalPosition();
}

//update global position based off movement of left wheel
void rise_l(float phi, float delta_x, float delta_y){
  phi_global = phi_global + phi; //change in angle already has needed negative sign 
  delay(10);
  float delta_x_prime = delta_x*cos(phi_global) + delta_y*sin(phi_global); //global change
  float delta_y_prime = delta_x*sin(phi_global) + delta_y*cos(phi_global); //global change
  x_global = x_global + delta_x_prime; 
  y_global = y_global - delta_y_prime; //negative already accounted for? 
  printGlobalPosition();
   
}
/* 
 * end section for analytical method of obtaining x, y, theta_z 
 */


/*
 * begin matrix method section for x, y, theta_z
 */
  void matrix_l(){
    T = T*l_rot*l_trans;      
  }

  void matrix_r(){
    T = T*r_rot*r_trans; 
  };
 /*  
  * end matrix ethod for x, y, theta_z 
  */

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
  


void printGlobalPosition(){
  Serial.print("Global phi ");
  Serial.print(phi_global);
  Serial.println(" rad");
  Serial.print("Global X location ");
  Serial.print(x_global);
  Serial.println(" cm");
  Serial.print("Global Y location "); //updating too far?
  Serial.print(y_global);
  Serial.println(" cm");
}


void matrixGlobalPosition(){
  Serial.print("Global phi ");
  Serial.print(phi_global_matrix);
  Serial.println(" rad");
  Serial.print("Global X location ");
  Serial.print(x_global_matrix);
  Serial.println(" cm");
  Serial.print("Global Y location "); //updating too far?
  Serial.print(y_global_matrix);
  Serial.println(" cm");
}

void test(){
  int testLength = 100;
  for(int i = 0; i < testLength; i++){
    
    right_tick();
    delay(10);
    left_tick();
    delay(10); 
  }
  x_global_matrix = T(0, 3);
  y_global_matrix = T(1, 3);
  delay(10);
  Serial.println("matrix method");
  Serial.print("Global phi ");
  Serial.print(phi_global_matrix); //how does it update?
  Serial.println(" rad");
  Serial.print("Global X Final location ");
  Serial.print(x_global_matrix);
  Serial.println(" cm");
  Serial.print("Global Y Final location ");
  Serial.print(y_global_matrix);
  Serial.println(" cm"); 
}

void motorsOff(){
  analogWrite(MOTOR1_PINA, 0);
  analogWrite(MOTOR1_PINB, 0);
  analogWrite(MOTOR2_PINA, 0);
  analogWrite(MOTOR2_PINB, 0);
}


int looptest = 1; 
void loop(){
    delay(10000);
    if(y_global >= 50){
      motorsOff();
    } 
    else setVelocity(0.5);
    
    /*
    while(Serial.available()){
      float vel = Serial.parseFloat();
      setVelocity(vel);
    }
    */
}
