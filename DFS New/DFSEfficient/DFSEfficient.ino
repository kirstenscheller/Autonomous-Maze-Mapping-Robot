#include <Servo.h>
#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft

#include <FFT.h> // include the library

#define wallSensorFront A2 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)
#define wallSensorRight A5
#define wallSensorLeft A1
#define irSensorFront A3       //Readings from IR sensor to analog pin
#define rlinesen A4
#define llinesen A0

typedef enum {
              NORTH = 1,
              EAST  = 2,
              SOUTH = 4,
              WEST  = 8,
              } DIRECTION;
              
Servo rWheel;
Servo lWheel;


//byte llinesen = 8; //line sensor on the left
//byte rlinesen = 7; //line sensor on the right

byte rWheelPin = 5; //right wheel
byte lWheelPin = 4; //left wheel
byte robotLed = 6; // Indicates robot sensed
byte goalLed = 2;
byte pushButton = 0;

             
//ir steps: 1. check for robots 2. if no cnt if yes turn slightly, turn back forward, check again and cnt if robot is gone else turn around

//================== Thresholds ======================//

int irThresh=750;            //Other Robot Distance
int lineThresh = 750;           //Line Sensor
float distThresh_f = 6.5;         //Forward Wall
byte distThresh_r = 7;           //Right Wall
byte distThresh_l = 13;          //Left Wall
//================== Thresholds ======================//

byte robotX = 0; 
byte robotY = 8;
//byte robotCoords = 8; //can do this so robotX and Y are in 1 byte since we send it as one anyway (if we need space)

byte AVAILABLE = 255;
byte robotOrientation = NORTH; 
byte nextDir = robotOrientation;
bool inBacktrack = false;
bool robotBacktrack = false;
bool done = false;

//change this to 1 byte per x,y pair or get rid of to free up space
byte xleft=robotX;
byte yleft=robotY;
byte xfront=robotX;
byte yfront=robotY;
byte xright=robotX;
byte yright=robotY;
byte xback=robotX;
byte yback=robotY;

byte maze[9][9] =  {{0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0},
                     {240,0,0,0,0,0,0,0,0}};

void setup() {
  Serial.begin(9600); // start the serial port
  pinMode(rWheelPin, OUTPUT);
  pinMode(lWheelPin, OUTPUT);
//  pinMode(rlinesen, INPUT);
  pinMode(robotLed, OUTPUT);
  pinMode(goalLed, OUTPUT);
  pinMode(pushButton, INPUT);
  
////================FFT stuff==========================
// // Serial.begin(115200); // use the serial port
  TIMSK0 = 0; // turn off timer0 for lower jitter
  ADCSRA = 0xe5; // set the adc to free running mode
  ADMUX = 0x40; // use adc0
  DIDR0 = 0x01; // turn off the digital input for adc0
////================FFT stuff==========================

  rWheel.attach(rWheelPin);
  lWheel.attach(lWheelPin);

  digitalWrite(robotLed, LOW);
  digitalWrite(goalLed, LOW);
  
  // right now, waitForSignal is programmed for pushbutton override
  tests();   
  waitForSignal();
  setInitialNode();
}

void loop() {
  if (digitalRead(pushbutton)){
  tests();        //check for testmode
//waitForSignal(); //wait for sound signal of 950 Hz
//  irSense();      //check for other robots
  lineFollow();  
  //test_servo();
  }
}

void test_servo(){
  rWheel.write(90);
  lWheel.write(90);
}
/*this function is for the robot to readjust itself <br>
if it goes off the white line.*/
    if (digitalRead(pushButton)) { while(!digitalRead(pushButton)) { halt(); }}
    irSense();      //check for other robots
    lineFollow(); 
}

void halt() {
  rWheel.write(90);
  lWheel.write(90);
}

void setInitialNode() {
  neighbourIndex(); 
  if (!isFrontWall()) {
    maze[yfront][xfront] = 255;
    maze[yright][xright] = isRightWall() ? 0 : 255;
    goFNode();
  }
//  else if (isRightWall()){
//    digitalWrite(goalLed,HIGH);
//  }
  else {
    maze[yright][xright] = 255;
    orientRight();
    robotOrientation = nextDir;

    robotX = xright;
    robotY = yright; 

    lWheel.write(180);
    rWheel.write(95);
    delay(300);
    bool flag = true;
    while(flag) {
      if (isRightWhite()) {
        flag = false;
        delay(300);
      }   
    }
    halt(); 
    }
  setNodeDir(); 
  neighbourIndex(); 
}


void lineFollow() {
  digitalWrite(robotLed,LOW); //comment out just for testing

  if (isLeftWhite() && (!isRightWhite())){         // robot off to right
    rWheel.write(0);
    lWheel.write(90);
    delay(100);
  }
  else if ((!isLeftWhite()) && isRightWhite()){   // robot off to the left
    rWheel.write(90);
    lWheel.write(180);
    delay(100);
  } 
  else if (isLeftWhite() && isRightWhite()){      // intersection
    halt();
    digitalWrite(robotLed,HIGH); // comment out just for testing
//    delay(200); //if random weeird bugs uncomment this
//    Serial.println("DFS!\n\n\n");
    dfs_traverse();                               //use DFS to determine next move

    digitalWrite(blueLed, LOW);
    digitalWrite(redLed, LOW);
    delay(300);
  }  else {  
    Serial.println("move Forward");  
    moveForward();                                // if no lines, just go straight

//    digitalWrite(robotLed, LOW);
//    delay(300); //if random weeird bugs uncomment this
  }  else {    
    moveForward();                               

  }
}

void dfs_traverse() {
    neighbourIndex();   //set front back left right indexes correctly based on current orientation
    byte nodeWalls = wallCheck();
  
    setAvailableNodes(nodeWalls);
    
    //sendIR();
    //sendWalls(); 
    if (inBacktrack) { 
        setCumulDir();
      }
    else if (robotBacktrack) {
      maze[robotY][robotX] = 255;
    }
    else {
      setNodeDir(); 
    }
//    printMazeState();
    checkGoal();
    
    if (!isFrontWall() && validMove(xfront, yfront) && maze[yfront][xfront] == 255) {
      inBacktrack = false; robotBacktrack = false;
      goFNode();
    } 
    else if (!isLeftWall() && (validMove(xleft, yleft) &&  maze[yleft][xleft] == 255)) {
      inBacktrack = false; robotBacktrack = false;
      goLNode();
    } 
    else if (!isRightWall() && (validMove(xright, yright) && maze[yright][xright] == 255)) {
      inBacktrack = false; robotBacktrack = false;
      goRNode(); 
    } 
    else { 
      if (!robotBacktrack) {
        inBacktrack = true;
      }
      else {
        //but what if you see a robot in regular backtrack ? what's the protocol?
//            Serial.println("robotBacktrack");
      }
      goBack();
    }
   robotOrientation = nextDir;
  }

void checkGoal() {
  for(byte i = 0; i < 9; i++){
    for(byte j = 0; j < 9; j++) {
      if(maze[i][j] == AVAILABLE) {
        return;
      }
    }
  }
  done = true;
  digitalWrite(goalLed, HIGH);
}

void setAvailableNodes(byte walls) {
  byte northCoords = getRelativeNorth();
  byte southCoords = getRelativeSouth();
  byte eastCoords = getRelativeEast();
  byte westCoords= getRelativeWest();
  
  if ((!(walls & NORTH)) && canSetAsAvailable(northCoords)) {
    maze[northCoords & 15][northCoords >> 4] = AVAILABLE;
  } 
  if ((!(walls & SOUTH)) && canSetAsAvailable(southCoords)) {
    maze[southCoords & 15][southCoords >> 4] = AVAILABLE;
  } 
   if ((!(walls & EAST)) && canSetAsAvailable(eastCoords)) {
    maze[eastCoords & 15][eastCoords >> 4] = AVAILABLE;
  } 
   if ((!(walls & WEST)) && canSetAsAvailable(westCoords)) {
    maze[westCoords & 15][westCoords >> 4] = AVAILABLE;
  } 
}

bool canSetAsAvailable(byte m) {
  byte x = m >> 4;
  byte y = m & 15;
  
  return validMove(x,y) && (maze[y][x] == 0);
}

/* returns a byte repr the north coordinates: <xval> <yval>*/
byte getRelativeNorth() {
  switch(robotOrientation) {
    case(NORTH):
      return (xfront << 4) | yfront;
    case(SOUTH):
      return (xback << 4) | yback;
    case(EAST):
      return (xleft << 4) | yleft;
    case(WEST):
      return (xright << 4) | yright;
  }
}

byte getRelativeSouth() {
  switch(robotOrientation) {
    case(SOUTH):
      return (xfront << 4) | yfront;
    case(NORTH):
      return (xback << 4) | yback;
    case(WEST):
      return (xleft << 4) | yleft;
    case(EAST):
      return (xright << 4) | yright;
  }
}

byte getRelativeEast() {
  switch(robotOrientation) {
    case(EAST):
      return (xfront << 4) | yfront;
    case(WEST):
      return (xback << 4) | yback;
    case(SOUTH):
      return (xleft << 4) | yleft;
    case(NORTH):
      return (xright << 4) | yright;
  }
}

byte getRelativeWest() {
  switch(robotOrientation) {
    case(WEST):
      return (xfront << 4) | yfront;
    case(EAST):
      return (xback << 4) | yback;
    case(NORTH):
      return (xleft << 4) | yleft;
    case(SOUTH):
      return (xright << 4) | yright;
  }
}

void setCumulDir() {
   maze[robotY][robotX] |= robotOrientation;
}

void goBack() {
  byte comp = (maze[robotY][robotX] & 240) | robotOrientation;
  switch (comp) {     // <prev dir> <orientation>
    case (NORTH | (NORTH << 4)):   
      orientBack();     
      robotX = xback;
      robotY = yback; 
      turnAround(); 
      moveForward(); 
      break; 
    case (NORTH | (EAST << 4)):  goLNode();  break;
    case (NORTH | (SOUTH << 4)): goFNode(); break;
    case (NORTH | (WEST << 4)): goRNode();  break;
    case (EAST | (NORTH << 4)):  goRNode(); break;
    case (EAST | (EAST << 4)):
      orientBack();       
      robotX = xback;
      robotY = yback; 
      turnAround(); 
      moveForward(); 
      break;
    case (EAST | (SOUTH << 4)): goLNode();   break;
    case (EAST | (WEST << 4)):  goFNode(); break;
    case (SOUTH | (NORTH << 4)):  goFNode(); break;
    case (SOUTH | (EAST << 4)):  goRNode(); break;
    case (SOUTH | (SOUTH << 4)): 
      orientBack();       
      robotX = xback;
      robotY = yback; 
      turnAround(); 
      moveForward(); 
      break;
    case (SOUTH | (WEST << 4)):  goLNode();  break;
    case (WEST | (NORTH << 4)): goLNode();   break;
    case (WEST | (EAST << 4)): goFNode(); break;
    case (WEST | (SOUTH << 4)): goRNode();  break;
    case (WEST | (WEST << 4)): 
      orientBack();  
      robotX = xback;
      robotY = yback; 
      turnAround(); 
      moveForward(); 
      break;
    default: Serial.println(comp); Serial.println("take out"); while(1) { halt(); } break;
 }
}

void goFNode() {
  robotX = xfront;
  robotY = yfront;
  longerMoveForward();
}

void goLNode() {
  orientLeft();
  robotX = xleft;
  robotY = yleft;
  turnLeft();
}

void goRNode() {
  orientRight();
  robotX = xright;
  robotY = yright; 
  turnRight(); 
}

void setNodeDir() {
  maze[robotY][robotX] = (maze[robotY][robotX] == AVAILABLE) ? 0 : maze[robotY][robotX] &= 15;
  setCumulDir();
  maze[robotY][robotX] |= (robotOrientation << 4);
}

bool isFrontWall() { 
//  float f_volts = ;  // value from sensor * (5/1024)
  return 13*pow(analogRead(wallSensorFront)*0.0048828125, -1) <= distThresh_f; 
}

bool isLeftWall() {
//  float l_volts = analogRead(wallSensorLeft)*0.0048828125;  // value from sensor * (5/1024)
  return 13*pow(analogRead(wallSensorLeft)*0.0048828125, -1) <= distThresh_l; // worked out from datasheet graph
}

bool isRightWall() {
//  float r_volts = analogRead(wallSensorRight)*0.0048828125;  // value from sensor * (5/1024)
  return 13*pow(analogRead(wallSensorRight)*0.0048828125, -1) <= distThresh_r; // worked out from datasheet graph
}

//Check the walls, then set corresponding bit to 1 if there is a wall 
byte wallCheck() {
  byte wall = 0;
  
  switch (robotOrientation) {
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
//  if(wall == 0) { wall = 15; } // I think this is for when we stored walls
  return wall;  
}

void turnAround() {
  rWheel.write(180);
  lWheel.write(180);
  bool flag = true;
  delay(1000);
  while(flag) {
    if (isRightWhite()) {
      flag = false;
      delay(100);
    }   
  }
  return;
}

void turnRight() {
  longerMoveForward();
  moveForward();
  moveForward();

  rWheel.write(90);
  lWheel.write(90);

  lWheel.write(180);
  rWheel.write(95);
  delay(200);
  bool flag = true;
  while(flag) {
    if (isRightWhite()) {
      flag = false;
    }   
  }
  
  moveForward();
  return;
}

void moveForward() {

  rWheel.write(70);    
  lWheel.write(100);   //account for servos not being exactly same

  delay(100);
}

void longerMoveForward() {
  moveForward();moveForward();moveForward();moveForward();
}

void turnLeft() { 
  longerMoveForward();
  moveForward();
  moveForward();

  rWheel.write(90);
  lWheel.write(90);

  lWheel.write(85);
  rWheel.write(0);
  delay(200);
  bool flag = true;
  while(flag) {
    if (isLeftWhite()) {
      flag = false;
    }   
  }
  
  moveForward();
  return;
}


bool isLeftWhite() {
//  return digitalRead(llinesen) == 1;
return analogRead(llinesen) < lineThresh;
}

bool isRightWhite() {
//  return digitalRead(rlinesen) == 1;
return analogRead(rlinesen) < lineThresh;
}


void irSense() {
  return; //fix
  if(analogRead(irSensorFront) <= irThresh)           //If obstacle is nearer than the Threshold range  
  {  
   digitalWrite(robotLed,HIGH);     // turn blue LED on, another robot is in front of Gerald
   turnAround();
   turnAround();
   if(analogRead(irSensorFront) <= irThresh) {
    robotBacktrack = true;
    orientBack();     
    robotX = xback;
    robotY = yback; 
    turnAround(); 
    moveForward(); 
    
    //revert to last entrance(before prev) of node and go to next available node
    //if there's nowhere to go then try to go straight again
   }
   //reverse() until no robot or hit double white
   //if hit double white then 
   
  }  
  else         //If obstacle is not in Threshold range  
  {  
   digitalWrite(robotLed,LOW);     // turn blue LED off
  }  
}

void orientRight(){
  switch (nextDir) {
  case NORTH: nextDir = EAST;  break;
  case EAST:  nextDir = SOUTH; break;
  case SOUTH: nextDir = WEST;  break;
  case WEST:  nextDir = NORTH; break;
 }
}

void orientBack() {
  orientRight();
  orientRight();
}

void orientLeft(){
  switch(nextDir){
  case NORTH: nextDir = WEST;  break;
  case WEST:  nextDir = SOUTH; break;
  case SOUTH: nextDir = EAST;  break;
  case EAST:  nextDir = NORTH; break;
 }
}

bool validMove(byte xCoord, byte yCoord) {
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
 }
}

void waitForSignal() {

   while(1) { // reduces jitter
    cli();  // UDRE interrupt slows this way down on arduino1.0
    for (int i = 0 ; i < 512 ; i += 2) { // save 256 samples
      while(!(ADCSRA & 0x10)); // wait for adc to be ready
      ADCSRA = 0xf7; // restart adc
      byte m = ADCL; // fetch adc data
      byte j = ADCH;
      int k = (j << 8) | m; // form into an int
      k -= 0x0200; // form into a signed int
      k <<= 6; // form into a 16b signed int
      fft_input[i] = k; // put real data into even bins
      fft_input[i+1] = 0; // set odd bins to 0
    }
    fft_window(); // window the data for better frequency response
    fft_reorder(); // reorder the data before doing the fft
    fft_run(); // process the data in the fft
    fft_mag_log(); // take the output of the fft
    sei(); 
    Serial.println("start");
    for (byte i = 0 ; i < FFT_N/2 ; i++) { 
      Serial.println(fft_log_out[i]); // send out the data
      if(fft_log_out[25] > 100){
        Serial.println("950 detected");
        return;
      } else{
        Serial.println("not detected");
       }
     }
  }

  // add push button override
    while(!digitalRead(pushButton)){
      halt();
    } 
  
//   while(1) { // reduces jitter
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
//  }

}

void printMazeState() {
  for(byte i=0; i <9; i++){
    for(byte j=0; j <9; j++){
      Serial.print(maze[i][j]);
    }
    Serial.print("\n");
  }
}

void testIrSen(bool cond) {
  while(cond){
    Serial.println("Test IR");
    rWheel.write(90);
    lWheel.write(90);

      Serial.print("ir sensor read: "); 
      Serial.println(analogRead(irSensorFront));

      if (analogRead(irSensorFront) <= irThresh){
          digitalWrite(robotLed, HIGH);
         } else
      {
            digitalWrite(robotLed, LOW);
      }
  }
}


void testWallSen(bool cond) {
  while(cond){
    Serial.println("Test Walls");
    rWheel.write(90);
    lWheel.write(90);
//    float f_volts = analogRead(wallSensorFront)*0.0048828125;  // value from sensor * (5/1024)
//    float f_distance = 13*pow(analogRead(wallSensorFront)*0.0048828125, -1); // worked out from datasheet graph
//    float l_volts = analogRead(wallSensorLeft)*0.0048828125;  // value from sensor * (5/1024)
//    float l_distance = 13*pow(analogRead(wallSensorLeft)*0.0048828125, -1); // worked out from datasheet graph
//    float r_volts = analogRead(wallSensorRight)*0.0048828125;  // value from sensor * (5/1024)
//    float r_distance = 13*pow(analogRead(wallSensorRight)*0.0048828125, -1); // worked out from datasheet graph

      Serial.print("forward Sensor: ");
      Serial.println(13*pow(analogRead(wallSensorFront)*0.0048828125, -1));
    
      Serial.print("Right   Sensor: ");
      Serial.println(13*pow(analogRead(wallSensorRight)*0.0048828125, -1));
 
      Serial.print("Left    Sensor: ");
      Serial.println(13*pow(analogRead(wallSensorLeft)*0.0048828125, -1));

      if (isFrontWall()|| isRightWall() || isLeftWall()){
          Serial.print("ON!!!!");

          delay(400);
         }
      delay(400);
 }
}

void testLineSen(bool cond) {
  if (cond) { Serial.println("Test LineSense"); waitForSignal(); }

  while (cond) {
    Serial.print("Right Sensor: ");
    Serial.print(isRightWhite());
    Serial.print(" Left Sensor:  ");
    Serial.println(isLeftWhite());
    
  }
}
 
void tests() {
  testWallSen(false);
  testIrSen(false);
  testLineSen(false);
  }
