// Sharp IR GP2Y0A41SK0F Distance Test
// http://tinkcore.com/sharp-ir-gp2y0a41-skf/
#include <Servo.h>

#define wallSensorFront A2 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)
#define wallSensorRight A5
#define wallSensorLeft A1
#define irSensorFront A3       //Readings from IR sensor to analog pin

int llinesen = 13; //line sensor on the left
int rlinesen = 12; //line sensor on the right
int rWheelPin = 10; //right wheel
int lWheelPin = 9; //left wheel
int blueLed = 11; // Indicates robot sensed
int redLed = 8; // Indicated wall sensed  
int greenLed = 7;
// Radio on pins 2-6
               
//thresholds
int irThresh=600;            //Other Robot Distance
int lineThresh = 750;           //Line Sensor
float distThresh_f = 4.75;         //Forward Wall
int distThresh_r = 7;          //Right Wall
int distThresh_l = 10;          //Left Wall

Servo rWheel;
Servo lWheel;


void setup() {
  Serial.begin(9600); // start the serial port
  pinMode(rWheelPin, OUTPUT);
  pinMode(lWheelPin, OUTPUT);
  pinMode(rlinesen, INPUT);
  pinMode(blueLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  //digitalWrite(irLed,HIGH);       //supply 5 volts to photodiode  

  rWheel.attach(rWheelPin);
  lWheel.attach(lWheelPin);
  rWheel.write(90);
  lWheel.write(90);

}

void loop() {

  tests();
  //irSense();
  //lineFollow();
  //distCheck();
}

void moveForward() {
  rWheel.write(80);   //87   slow  
  lWheel.write(100);   //93   slow
  delay(50);
}

void distCheck() {
  float f_volts = analogRead(wallSensorFront)*0.0048828125;  // value from sensor * (5/1024)
  float f_distance = 13*pow(f_volts, -1); // worked out from datasheet graph
 
  float r_volts = analogRead(wallSensorRight)*0.0048828125;  // value from sensor * (5/1024)
  int r_distance = 13*pow(r_volts, -1); // worked out from datasheet graph
  
  float l_volts = analogRead(wallSensorLeft)*0.0048828125;  // value from sensor * (5/1024)
  int l_distance = 13*pow(l_volts, -1); // worked out from datasheet graph
  Serial.println(r_distance);

  if ((r_distance <= distThresh_r)|| (l_distance <= distThresh_l) || (f_distance <= distThresh_f)) {        // Light red LED if you see a wall
      digitalWrite(redLed, HIGH);
  }
  else {
      digitalWrite(redLed, LOW);
  }


  // Right Hand Follow Code:
  if (f_distance <= distThresh_f && l_distance <= distThresh_l && r_distance <= distThresh_r){
    turnAround();
  }
  else if (r_distance > distThresh_r){
//      Serial.println("Right turn!");
//      Serial.print("forward Sensor: ");
//      Serial.println(f_distance);
//      Serial.print("Right   Sensor: ");
//      Serial.println(r_distance);    
      turnRight();
  }
  else if (f_distance <= distThresh_f){
//      Serial.println("Left turn");
//      Serial.print("forward Sensor: ");
//      Serial.println(f_distance);
//      Serial.print("Right   Sensor: ");
//      Serial.println(r_distance);    
      turnLeft();
  } 
  else {
    moveForward();  
  }

}

void turnAround() {
  Serial.println("TURN AROUND!!");
  rWheel.write(180);
  lWheel.write(180);
  delay(1350);
}

void turnRight() {
  //move forward a little bit before turning to prevent overturn

    int i = 0;
  while(i<10){
    moveForward();
    i++;
  }
  rWheel.write(90);
  lWheel.write(90);
  delay(200);

  lWheel.write(180);
  rWheel.write(95);
  delay(675);
}

void turnLeft() { 
  int i = 0;
  while(i<10){
    moveForward();
    i++;

  }
  rWheel.write(90);
  lWheel.write(90);
  delay(200);

  rWheel.write(0);
  lWheel.write(85);
  delay(675);
}


bool isLeftWhite() {
  return digitalRead(llinesen) == 1;
}

bool isRightWhite() {
  return digitalRead(rlinesen) == 1;
}

/*this function is for the robot to readjust itself <br>
if it goes off the white line.*/
void lineFollow() {
  irSense();
  if (isLeftWhite() && (!isRightWhite())){
    rWheel.write(85);
    lWheel.write(85);
    delay(100);
  }
  else if ((!isLeftWhite()) && isRightWhite()){
    rWheel.write(95);
    lWheel.write(95);
    delay(100);
  } else if (isLeftWhite() && isRightWhite()){
    rWheel.write(90);
    lWheel.write(90);
    distCheck();
    digitalWrite(blueLed, LOW);
    digitalWrite(redLed, LOW);
    delay(300);
  }  else {    
    Serial.println("forward");
    moveForward();  
  }
}

void irSense() {
  int val=analogRead(irSensorFront);  //variable to store values from the photodiode  
  Serial.println(val);          // prints the values from the sensor in serial monitor  
  if(val <= irThresh)           //If obstacle is nearer than the Threshold range  
  {  
   digitalWrite(blueLed,HIGH);// turn blue LED on, another robot is in front of Gerald
     //turn around
  //rWheel.write(180);
  //lWheel.write(180);
  //delay(1500);
  }  
  else         //If obstacle is not in Threshold range  
  {  
   digitalWrite(blueLed,LOW);     // turn blue LED off
   //Do nothing - continue as you were
  }  
}

void testIrSen(bool cond) {
  while(cond){
    irSense();
  }
}

void testWallSen(bool cond) {
  while(cond){
    rWheel.write(90);
    lWheel.write(90);

    float f_volts = analogRead(wallSensorFront)*0.0048828125;  // value from sensor * (5/1024)
    float f_distance = 13*pow(f_volts, -1); // worked out from datasheet graph
    
    float r_volts = analogRead(wallSensorRight)*0.0048828125;  // value from sensor * (5/1024)
    int r_distance = 13*pow(r_volts, -1); // worked out from datasheet graph

    float l_volts = analogRead(wallSensorLeft)*0.0048828125;  // value from sensor * (5/1024)
    int l_distance = 13*pow(l_volts, -1); // worked out from datasheet graph

      Serial.print("forward Sensor: ");
      Serial.println(f_distance);
    
      Serial.print("Right   Sensor: ");
      Serial.println(r_distance);
 
      Serial.print("Left    Sensor: ");
      Serial.println(l_distance);

      if (f_distance <= distThresh_f || r_distance <= distThresh_r || l_distance <= distThresh_l){
          digitalWrite(redLed, HIGH);
          Serial.print("ON!!!!");

          delay(400);
          digitalWrite(redLed, LOW);
         }
      delay(400);
}
}

void tests() {
  testWallSen(true);
  testIrSen(false);
  distCheck();
  }
