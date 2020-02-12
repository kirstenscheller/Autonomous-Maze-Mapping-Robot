// Sharp IR GP2Y0A41SK0F Distance Test
// http://tinkcore.com/sharp-ir-gp2y0a41-skf/
// check that 15's are printed when no walls are detected
#include <Servo.h>
#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft

#include <FFT.h> // include the library

#define wallSensorFront A2 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)
#define wallSensorRight A5
#define wallSensorLeft A1
#define irSensorFront A3       //Readings from IR sensor to analog pin

typedef enum {
              NORTH = 1,
              EAST  = 2,
              SOUTH = 4,
              WEST  = 8,
              } DIRECTION;

typedef enum {
              FORWARD = 1,
              LEFT  = 2,
              RIGHT = 4,
              BACK = 8,
              } TURN;
              
Servo rWheel;
Servo lWheel;

int llinesen = 5;   //line sensor on the left
int rlinesen = 4;   //line sensor on the right
int rWheelPin = 10; //right wheelb
int lWheelPin = 9;  //left wheel
int blueLed = 11;
int greenLed = 12;
int redLed = 8;
int irLed = 2;             
               
//thresholds
int irThresh =100;              //Other Robot Distance
int lineThresh = 750;           //Line Sensor
float distThresh_f = 5;         //Forward Wall
int distThresh_r = 7;          //Right Wall
int distThresh_l = 8;          //Left Wall

//int visitedBox = 0;

byte robotX = 0;
byte robotY = 8;

byte pastPosX = NULL;
byte pastPosY = NULL;

byte robotOrientation = NORTH;  //CHANGE IF NOT STARTING AT BOTTOM LEFT

byte xleft=robotX;
byte yleft=robotY;
byte xfront=robotX;
byte yfront=robotY;
byte xright=robotX;
byte yright=robotY;
byte xback=robotX;
byte yback=robotY;

//bool accessGranted = false;
//bool doturnAround  = true;

byte maze[9][9] =  {{0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0}};
                       
//the matrix that the robot is going to transmit            
//word makeSend [9][9];

//the stack that keeps track of the positions
//the stack that keeps track of the turns for backtrack
byte turnStack[81];


int stackIndex=0;

//set stackindex to -1  then increment stack as push --> otherwise stack index is the next index instead of the right one
//alternatively pop at stackindex - 1

void setup() {
  Serial.begin(9600); // start the serial port
  pinMode(rWheelPin, OUTPUT);
  pinMode(lWheelPin, OUTPUT);
  pinMode(rlinesen, INPUT);
  pinMode(blueLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  digitalWrite(irLed,HIGH);       //supply 5 volts to photodiode  
  
////================FFT stuff==========================
////  Serial.begin(115200); // use the serial port
//  TIMSK0 = 0; // turn off timer0 for lower jitter
//  ADCSRA = 0xe5; // set the adc to free running mode
//  ADMUX = 0x40; // use adc0
//  DIDR0 = 0x01; // turn off the digital input for adc0
////================FFT stuff==========================

  rWheel.attach(rWheelPin);
  lWheel.attach(lWheelPin);
  rWheel.write(90);
  lWheel.write(90);
}

void loop() {
  tests();        //check for testmode
//  waitForSignal(); //wait for sound signal of 950 Hz
//  irSense();      //check for other robots
  lineFollow();  
}

void dfs_traverse() {
    neighbourIndex();   //set front back left right indexes correctly based on current orientation
    updateWalls();
    Serial.print("Facing: ");
    Serial.println(robotOrientation);
//    printMazeState();

    //if there's no front wall and front is a valid non-visited
    if (!isFrontWall() && ((maze[yfront][xfront] == 0 && validMove(xfront, yfront)))) {
      moveForward();      
      if (maze[robotY][robotX] ==0) {turnStack[stackIndex] = FORWARD;} 
      robotX = xfront;
      robotY = yfront;
    else if (!isLeftWall() && ((maze[yleft][xleft] == 0 && validMove(xleft, yleft)))) {
      if (maze[robotY][robotX] ==0) {turnStack[stackIndex] = RIGHT;}
      turnLeft();
      //update visited to reflect new current position and set the old current position to 1
      Serial.println("turn left");
      turnLeft();
      turnStack[stackIndex] = RIGHT;
      stackIndex++;
    } 
    else if (!isRightWall() && maze[yright][xright] == 0 && validMove(xright, yright)) {
      if (maze[robotY][robotX] == 0) {turnStack[stackIndex] = LEFT;}
      turnRight();
      stackIndex++;
    } 
    else {
      turnAround();
      neighbourIndex();
      backtrack();
      printStack();
      printMazeState();
    }
  }

void updateWalls() {
  maze[robotY][robotX] = wallCheck();
  return;
}

bool isFrontWall() { 
  float f_volts = analogRead(wallSensorFront)*0.0048828125;  // value from sensor * (5/1024)
  return 13*pow(f_volts, -1) <= distThresh_f; 
}

bool isLeftWall() {
  float l_volts = analogRead(wallSensorLeft)*0.0048828125;  // value from sensor * (5/1024)
  return 13*pow(l_volts, -1) <= distThresh_l; // worked out from datasheet graph
}

bool isRightWall() {
  float r_volts = analogRead(wallSensorRight)*0.0048828125;  // value from sensor * (5/1024)
  return 13*pow(r_volts, -1) <= distThresh_r; // worked out from datasheet graph
}
//Check the walls, then set corresponding bit to 1 if there is a wall 
byte wallCheck() {

  byte wall = 0;
  
  switch (robotOrientation) {
    Serial.print("Facing: ");
    Serial.println(robotOrientation);
    case(NORTH):
      if(isFrontWall()){  wall |= NORTH;}
      if(isLeftWall()) {  wall |= WEST; }
      if(isRightWall()){  wall |= EAST; }
      break;
    case(EAST):
      if(isFrontWall()){  wall |= EAST; }
      if(isLeftWall()) {  wall |= NORTH;}
      if(isRightWall()){  wall |= SOUTH;} 
      break;
    case (SOUTH):
      if(isFrontWall()){  wall |= SOUTH;}
      if(isLeftWall()) {  wall |= EAST; }
      if(isRightWall()){  wall |= WEST; } 
      break;
    case (WEST):
      if(isFrontWall()){  wall |= WEST; }
      if(isLeftWall()) {  wall |= SOUTH;}
      if(isRightWall()){  wall |= NORTH;} 
      break;
  }
  if(wall == 0) { wall = 15; }
  return wall;  
}

//void stack_push(byte x, byte y) {
//  posStack[stackIndex][0] = x;
//  posStack[stackIndex][1] = y;
//  stackIndex++;
//  return;
//}

void backtrack(){
  Serial.print("Stack Index:");
  Serial.println(stackIndex);
  //while there is not unvisited neighbor 
  while (true) {
    if (isLeftWhite() && (!isRightWhite())){         // if robot is off to the right
      rWheel.write(85);
      lWheel.write(85);
      delay(100);
    }
  else if ((!isLeftWhite()) && isRightWhite()){   // if robot is off to the left
    rWheel.write(95);
    lWheel.write(95);
    delay(100);
  } 
  else if (isLeftWhite() && isRightWhite()){      // if robot is at intersection
    if (!((isLeftWall() || maze[yleft][xleft]>0) && (isRightWall() || maze[yright][xright]>0) 
        && (isFrontWall()|| maze[yfront][xfront]>0))) { updateTurnStack(); break;}
    byte dir = stack_pop();
    switch (dir){
      case FORWARD:
       Serial.println("Go Straight");
        moveForward();
        robotX = xfront;
        robotY = yfront;
        neighbourIndex();
        break;
      case RIGHT:
        Serial.println("Turn Right");
        turnRight();
        neighbourIndex();
        break;
      case LEFT:
        Serial.println("Turn left");
        turnLeft();
        neighbourIndex();
        break;
      default:
        Serial.println("Impossible"); break;
    }
    Serial.print("Facing: ");
    Serial.println(robotOrientation);
//    printCoordinates();
    moveForward();
  }  else {    
    moveForward();                                // if no lines, just go straight
  }
  }
  Serial.print("Stack Index:");
  Serial.println(stackIndex);
  return;
}

void updateTurnStack() {
    switch(turnStack[stackIndex-1]) {
      case LEFT:
        turnStack[stackIndex-1] = RIGHT;
        break;
      case RIGHT:
        turnStack[stackIndex-1] = LEFT;
        break;
      default:
        break;
    } 
}

byte stack_pop() {
  if (stackIndex != 0) {
    stackIndex--;
    byte b = turnStack[stackIndex];
    while(b == BACK){
      stackIndex--;
      b = turnStack[stackIndex];
    }
    return b;
  }
}

void turnAround() {
  rWheel.write(180);
  lWheel.write(180);
  bool flag = true;
  delay(800);
  while(flag) {
    if (isRightWhite()) {
      flag = false;
    }   
  }
  orientRight();
  orientRight();
//  maze[yback][xback] = 6;
//  maze[robotY][robotX] = 1;
  robotX = xback;
  robotY = yback;
  return;
}

void turnRight() {
  //move forward a little bit before turning to prevent overturn
  byte i = 0;
  while(i<10){
    moveForward();
    i++;
  }
  rWheel.write(90);
  lWheel.write(90);
  delay(200);

  lWheel.write(180);
  rWheel.write(95);
  delay(925);
  
  orientRight();
//  maze[yright][xright] = 6;
//  maze[robotY][robotX] = 1;
  robotX = xright;
  robotY = yright;
  lWheel.write(90);
  rWheel.write(90);
  return;
}

void moveForward() {
  rWheel.write(80);   //87   slow  
  lWheel.write(100);   //93   slow
  delay(50);
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
  delay(925);

  orientLeft();
//  maze[yleft][xleft] = 6;
//  maze[robotY][robotX] = 1;
  robotX = xleft;
  robotY = yleft;
  lWheel.write(90);
  rWheel.write(90);
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
  if (isLeftWhite() && (!isRightWhite())){         // if robot is off to the right
    rWheel.write(85);
    lWheel.write(85);
    delay(100);
  }
  else if ((!isLeftWhite()) && isRightWhite()){   // if robot is off to the left
    rWheel.write(95);
    lWheel.write(95);
    delay(100);
  } 
  else if (isLeftWhite() && isRightWhite()){      // if robot is at intersection
    rWheel.write(90);                             //stop
    lWheel.write(90);
    delay(200);
    //    right_hand_follow();
//    dfs_traverse();                               //use DFS to determine next move
    
    digitalWrite(blueLed, LOW);
    digitalWrite(redLed, LOW);
    delay(300);
  }  else {    
    moveForward();                                // if no lines, just go straight
  }
}

void irSense() {
//  Serial.println("irSense");
//  int val=analogRead(irSensorFront);  //variable to store values from the photodiode  
////  Serial.println(val);          // prints the values from the sensor in serial monitor  
//  if(val <= irThresh)           //If obstacle is nearer than the Threshold range  
//  {  
//   digitalWrite(blueLed,HIGH);     // turn blue LED on, another robot is in front of Gerald
//  }  
//  else         //If obstacle is not in Threshold range  
//  {  
//   digitalWrite(blueLed,LOW);     // turn blue LED off
//  }  
}


void orientRight(){
  switch (robotOrientation) {
  case NORTH: robotOrientation = EAST;  break;
  case EAST:  robotOrientation = SOUTH; break;
  case SOUTH: robotOrientation = WEST;  break;
  case WEST:  robotOrientation = NORTH; break;
 }
}

void orientLeft(){
  switch(robotOrientation){
  case NORTH: robotOrientation = WEST;  break;
  case WEST:  robotOrientation = SOUTH; break;
  case SOUTH: robotOrientation = EAST;  break;
  case EAST:  robotOrientation = NORTH; break;
 }
}

bool validMove(int xCoord, int yCoord) {
  return !(xCoord > 8) && !(xCoord<0) && !(yCoord > 8) && !(yCoord<0);
}

void neighbourIndex(){
  switch(robotOrientation){
    case NORTH:           //if robot is facing North
       xleft=robotX-1; 
       yleft=robotY;
       xfront=robotX;
       yfront=robotY-1;
       xright=robotX+1;
       yright=robotY;
       xback=robotX;
       yback=robotY+1;
      break;
    case EAST:            //if robot is facing East
       xleft=robotX;
       yleft=robotY-1;
       xfront=robotX+1;
       yfront=robotY;
       xright=robotX;
       yright=robotY+1;
       xback=robotX-1;
       yback=robotY;
      break; 
    case SOUTH:           //if robot is facing South
       xleft=robotX+1;
       yleft=robotY;
       xfront=robotX;
       yfront=robotY+1;
       xright=robotX-1;
       yright=robotY;
       xback=robotX;
       yback=robotY-1;
      break;
    case WEST:            //if robot is facing West
       xleft=robotX;
       yleft=robotY+1;
       xfront=robotX-1;
       yfront=robotY;
       xright=robotX;
       yright=robotY-1;
       xback=robotX+1;
       yback=robotY;
      break;
  //unsure about bottom
//  if (robotX==NULL  && robotY==NULL){
//    xfront=8; //figure out what this part means
//    yfront=8;
//  }
 }
}

// add push button override
void waitForSignal() {
   while(1) { // reduces jitter
//    cli();  // UDRE interrupt slows this way down on arduino1.0
//    for (int i = 0 ; i < 512 ; i += 2) { // save 256 samples
//      while(!(ADCSRA & 0x10)); // wait for adc to be ready
//      ADCSRA = 0xf7; // restart adc
//      byte m = ADCL; // fetch adc data
//      byte j = ADCH;
//      int k = (j << 8) | m; // form into an int
//      k -= 0x0200; // form into a signed int
//      k <<= 6; // form into a 16b signed int
//      fft_input[i] = k; // put real data into even bins
//      fft_input[i+1] = 0; // set odd bins to 0
//    }
//    fft_window(); // window the data for better frequency response
//    fft_reorder(); // reorder the data before doing the fft
//    fft_run(); // process the data in the fft
//    fft_mag_log(); // take the output of the fft
//    sei(); 
//    Serial.println("start");
//    for (byte i = 0 ; i < FFT_N/2 ; i++) { 
//      Serial.println(fft_log_out[i]); // send out the data
//      if(fft_log_out[25] > 100){
//        Serial.println("950 detected");
//        return;
//      } else{
//        Serial.println("not detected");
//       }
//     }
  }
}

void printMazeState() {
  Serial.println("Walls Detected: ");
  for(int i=0; i <9; i++){
    for(int j=0; j <9; j++){
      Serial.print(maze[i][j]);
    }
    Serial.print("\n");
  }
}

void printStack(){
  Serial.print("Stack: [");
  for(int i = 0; i <= stackIndex; i++){
    Serial.print(turnStack[i]);
    Serial.print(",");
  }
  Serial.println("]");
}
void printCoordinates() {
  Serial.print("Front: ");
  Serial.print(xfront); Serial.print(", ");   Serial.println(yfront); 
  Serial.print("Coord: ");
  Serial.print(robotX); Serial.print(", ");  Serial.println(robotY); 
    Serial.print("Back: ");
  Serial.print(xback); Serial.print(", ");  Serial.println(yback); 
    Serial.print("Left: ");
  Serial.print(xfront ); Serial.print(", ");  Serial.println(yleft); 
    Serial.print("Right: ");
  Serial.print(xright ); Serial.print(", "); Serial.println(yright); 
}

void testIrSen(bool cond) {
//  while(cond){
//    irSense();
//  }
}


void testWallSen(bool cond) {
  while(cond){
    Serial.println("Test Walls");
    rWheel.write(90);
    lWheel.write(90);
    Serial.println(analogRead(wallSensorLeft));
    float f_volts = analogRead(wallSensorFront)*0.0048828125;  // value from sensor * (5/1024)
    float f_distance = 13*pow(f_volts, -1); // worked out from datasheet graph
    float l_volts = analogRead(wallSensorLeft)*0.0048828125;  // value from sensor * (5/1024)
    float l_distance = 13*pow(l_volts, -1); // worked out from datasheet graph
    float r_volts = analogRead(wallSensorRight)*0.0048828125;  // value from sensor * (5/1024)
    float r_distance = 13*pow(r_volts, -1); // worked out from datasheet graph

      Serial.print("forward Sensor: ");
      Serial.println(f_distance);
    
      Serial.print("Right   Sensor: ");
      Serial.println(r_distance);
 
      Serial.print("Left    Sensor: ");
      Serial.println(l_distance);

      if (isFrontWall()|| isRightWall() || isLeftWall()){
          digitalWrite(redLed, HIGH);
          Serial.print("ON!!!!");

          delay(400);
          digitalWrite(redLed, LOW);
         }
      delay(400);
 }
}
 
void tests() {
  testWallSen(false);
  testIrSen(false);
//  right_hand_follow();
 }
