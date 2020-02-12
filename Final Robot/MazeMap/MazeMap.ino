#include <Servo.h>
#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft

#include <FFT.h> // include the library
#include <SPI.h>
//#include <FFT.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"


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
byte pushButton = 3;
unsigned long timer;

//======================== Radio ============================//
  // Set up nRF24L01 radio on SPI bus plus pins 9 & 10
  RF24 radio(9,10);
  
  // Topology
  // Radio pipe addresses for the 2 nodes to communicate.
  const uint64_t pipes[2] = { 0x0000000018LL, 0x0000000019LL };
  
  // Role management: Set up role.  This sketch uses the same software for all the nodes
  // in this system.  Doing so greatly simplifies testing.
  // The various roles supported by this sketch
  typedef enum { role_ping_out = 1, role_pong_back } role_e;
  
  // The debug-friendly names of those roles
  const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};
  
  // The role of the current running sketch
  role_e role = role_ping_out;
//================== Radio ======================//

//ir steps: 1. check for robots 2. if no cnt if yes turn slightly, turn back forward, check again and cnt if robot is gone else turn around

//================== Thresholds ======================//
  int irThresh=600;            //Other Robot Distance
  int lineThresh = 650;           //Line Sensor
  float distThresh_f = 7;         //Forward Wall
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
                     {252,0,0,0,0,0,0,0,0}};

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
//  TIMSK0 = 0; // turn off timer0 for lower jitter
//  ADCSRA = 0xe5; // set the adc to free running mode
//  ADMUX = 0x40; // use adc0
//  DIDR0 = 0x01; // turn off the digital input for adc0
////================FFT stuff==========================

  rWheel.attach(rWheelPin);
  lWheel.attach(lWheelPin);

  digitalWrite(robotLed, LOW);
  digitalWrite(goalLed, LOW);
  
  // right now, waitForSignal is programmed for pushbutton override
  tests();   
    
  printf_begin();
  radio.begin();
  radio.setRetries(15,15);
  radio.setAutoAck(true);
  radio.setChannel(0x50);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);


  if ( role == role_ping_out )
  {
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
  }
//  else
//  {
//    radio.openWritingPipe(pipes[1]);
//    radio.openReadingPipe(1,pipes[0]);
//  }
  
  radio.startListening();
   while(!digitalRead(pushButton)){}; // needed so walls arent detected before its in maze
   byte initial_walls = setInitialNode();
   waitForSignal();   //FFT to wait for correct signal (has push button override too)
   initial_move(initial_walls);
}

void loop() {
    irSense();      //check for other robots
    lineFollow(); 
}

void initial_move(byte walls) {
  if (!(walls & NORTH == NORTH)) {
        goFNode();
   } else {
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
   timer = millis();
   }
   
void halt() {
  rWheel.write(90);
  lWheel.write(90);
}

byte setInitialNode() {
  byte walls = WEST | SOUTH;;
  
  neighbourIndex(); 
  if (!isFrontWall()) {
    maze[yfront][xfront] = 255;
    if (isRightWall()) {
      walls |= EAST;
    } else {
      maze[yright][xright] = 255;
    }
  }
  else {
    walls |= NORTH;
    maze[yright][xright] = 255;
    }
  setNodeDir();
  sendData(0, 8, walls);
 
//  neighbourIndex();  //maybe comment out ?
  halt();
  return walls;
}

void lineFollow() {
  digitalWrite(robotLed,LOW); //comment out just for testing
  if (isLeftWhite() && (!isRightWhite())){         // robot off to right
    rWheel.write(0);
    lWheel.write(88);
    delay(100);
  }
  else if ((!isLeftWhite()) && isRightWhite()){   // robot off to the left
    rWheel.write(92);
    lWheel.write(180);
    delay(100);
  } 
  else if (millis() - timer > 2000) {
    while(!isLeftWhite() || !isRightWhite()) {
    rWheel.write(110);
    lWheel.write(70);
    }
    rWheel.write(90);
    lWheel.write(90);
    dfs_traverse();
    timer = millis();
  }
  else if (isLeftWhite() && isRightWhite()){      // intersection
    halt();
    digitalWrite(robotLed,HIGH); // comment out just for testing
    if (done && robotX ==0 && robotY == 8) { while(!digitalRead(pushButton)) {halt(); } }
//    Serial.println("DFS!\n\n\n");
    dfs_traverse();                               //use DFS to determine next move
//    digitalWrite(robotLed, LOW);
    timer = millis();
  }  else {    
    moveForward();                               
  }
  
}

void dfs_traverse() {
    neighbourIndex();   //set front back left right indexes correctly based on current orientation
    byte nodeWalls = wallCheck();
  
    setAvailableNodes(nodeWalls);
    
    // or if this is buggy, figure out how to put   right after Go<d>Node so that it computs while it drives
//    moveForward(); 
    if (!inBacktrack && !robotBacktrack) {sendData(robotX, robotY, nodeWalls); }
    
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
//   neighbourIndex(); 
//   sendData(xback, yback, nodeWalls);
   robotOrientation = nextDir;
  }

void sendData(byte xCoord, byte yCoord, byte walls) {
  byte robot_loc = (xCoord << 4) | (yCoord);
  int data = robot_loc | (walls << 8);
  
  if (role == role_ping_out)
  {
    // First, stop listening so we can talk.
    radio.stopListening();

    // Take the time, and send it.  This will block until complete
    //unsigned long time = millis();
    unsigned int send_data1 = data; //WSEN,x,y
//    printf("Now sending %u...",send_data1);
    bool ok = radio.write( &send_data1, sizeof(unsigned int) );

//    if (ok)
//      printf("ok...");
//    else
//      printf("failed.\n\r");

    // Now, continue listening
    radio.startListening();

    // Wait here until we get a response, or timeout (250ms)
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout )
      if (millis() - started_waiting_at > 200 )
        timeout = true;

    // Describe the results
    if ( timeout )
    {
//      printf("Failed, response timed out.\n\r");
    }
    else
    {
      // Grab the response, compare, and send to debugging spew
      unsigned int got_data1;
      radio.read( &got_data1, sizeof(unsigned int) );
//      printf("Got response1 %u\n\r",got_data1);
    }
    // Try again 1s later
    delay(200);
  }

//  if ( role == role_pong_back )
//  {
//    // if there is data ready
//    if ( radio.available() )
//    {
//      // Dump the payloads until we've gotten everything
//      unsigned int got_data1;
//      bool done = false;
//      while (!done)
//      {
//        // Fetch the payload, and see if this was the last one.
//        done = radio.read( &got_data1, sizeof(unsigned int) );
//              // send to fpga
//        digitalWrite(2,bitRead(got_data1,0));
//        digitalWrite(3,bitRead(got_data1,1));
//        digitalWrite(4,bitRead(got_data1,2));
//        digitalWrite(19,bitRead(got_data1,3));
//        digitalWrite(18,bitRead(got_data1,4));
//        digitalWrite(17,bitRead(got_data1,5));
//        digitalWrite(16,bitRead(got_data1,6));
//        digitalWrite(15,bitRead(got_data1,7));
//       digitalWrite(5,bitRead(got_data1,8));
//       digitalWrite(6,bitRead(got_data1,9));
//       digitalWrite(7,bitRead(got_data1,10));
//       digitalWrite(8,bitRead(got_data1,11));
//        // Spew it
//        printf("%u..",bitRead(got_data1,11));
//        printf("%u..",bitRead(got_data1,10));
//        printf("%u..",bitRead(got_data1,9));
//        printf("%u..",bitRead(got_data1,8));
//        printf("%u..",bitRead(got_data1,7));
//        printf("%u..",bitRead(got_data1,6));
//        printf("%u..",bitRead(got_data1,5));
//        printf("%u..",bitRead(got_data1,4));
//        printf("%u..",bitRead(got_data1,3));
//        printf("%u..",bitRead(got_data1,2));
//        printf("%u..",bitRead(got_data1,1));
//        printf("%u..",bitRead(got_data1,0));
//        printf("Got payload %u...",got_data1);
//        // Delay just a little bit to let the other unit
//        // make the transition to receiver
//        delay(20);
//      }
//
//      // First, stop listening so we can talk
//      radio.stopListening();
//
//      // Send the final one back.
//      radio.write( &got_data1, sizeof(unsigned int) );
//      printf("Sent response.\n\r");
//
//      // Now, resume listening so we catch the next packets.
//      radio.startListening();
//    }
//  }

  if (Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'T' && role == role_pong_back )
    {
//      printf("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK\n\r");

      // Become the primary transmitter (ping out)
      role = role_ping_out;
      radio.openWritingPipe(pipes[0]);
      radio.openReadingPipe(1,pipes[1]);
    }
    else if ( c == 'R' && role == role_ping_out )
    {
//      printf("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK\n\r");

      // Become the primary receiver (pong back)
      role = role_pong_back;
      radio.openWritingPipe(pipes[1]);
      radio.openReadingPipe(1,pipes[0]);
    }
  }
}

void checkGoal() {
  for(byte i = 0; i < 9; i++){
    for(byte j = 0; j < 9; j++) {
      if(maze[i][j] == AVAILABLE) {
        return;
      }
    }
  }
  digitalWrite(goalLed, HIGH);

  done = true;
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
    default: Serial.println(comp); Serial.println("take out"); while(!(digitalRead(pushButton))) { halt(); } break;
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

  rWheel.write(90);
  lWheel.write(90);

  lWheel.write(180);
  rWheel.write(95);
  delay(200);
  bool flag = true;
  while(flag) {
    if (isRightWhite()) {
      flag = false;
      delay(150);
    }   
  }

  moveForward();
  return;
}

void moveForward() {
  rWheel.write(60);    
  lWheel.write(120);   //account for servos not being exactly same
  delay(100);
}

void longerMoveForward() {
  moveForward();moveForward();moveForward();moveForward();
}

void turnLeft() { 
  longerMoveForward();

  rWheel.write(90);
  lWheel.write(90);

  lWheel.write(85);
  rWheel.write(0);
  delay(200); //send wall data here??
  bool flag = true;
  while(flag) {
    if (isLeftWhite()) {
      flag = false;
      delay(150);
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
   //fix
  if(analogRead(irSensorFront) <= irThresh)           //If obstacle is nearer than the Threshold range  
  {  
   digitalWrite(robotLed,HIGH);     // turn blue LED on, another robot is in front of Gerald
   turnAround();
   turnAround();
   if(analogRead(irSensorFront) <= irThresh) {
//    robotBacktrack = true;
//    orientBack();     
//    robotX = xback;
//    robotY = yback; 
//    turnAround(); 
//    moveForward(); 
    while (!isLeftWhite() || !isRightWhite()) {
      rWheel.write(100);
      lWheel.write(80);
     }
      rWheel.write(90);
      lWheel.write(90);
      if (!inBacktrack && validMove(robotX,robotY)) {maze[robotY][robotX] = AVAILABLE;}
    }
    //revert to last entrance(before prev) of node and go to next available node
    //if there's nowhere to go then try to go straight again
   //reverse() until no robot or hit double white
   //if hit double white then 
   timer = millis();
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
    case NORTH:          
       xleft=robotX-1; 
       yleft=robotY;
       xfront=robotX;
       yfront=robotY-1;
       xright=robotX+1;
       yright=robotY;
       xback=robotX;
       yback=robotY+1;
      break;
    case EAST:
       xleft=robotX;
       yleft=robotY-1;
       xfront=robotX+1;
       yfront=robotY;
       xright=robotX;
       yright=robotY+1;
       xback=robotX-1;
       yback=robotY;
      break; 
    case SOUTH:         
       xleft=robotX+1;
       yleft=robotY;
       xfront=robotX;
       yfront=robotY+1;
       xright=robotX-1;
       yright=robotY;
       xback=robotX;
       yback=robotY-1;
      break;
    case WEST:
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
  // push button override
  
   while(!digitalRead(pushButton)) { // reduces jitter
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
  if (cond) { Serial.println("Test LineSense"); waitForSignal(); halt();}

  while (cond) {
    Serial.print("Right Sensor: ");
    Serial.print(isRightWhite());
        Serial.print("   ");
    Serial.print(analogRead(rlinesen));
    Serial.print(" Left Sensor:  ");
    Serial.print(isLeftWhite());
        Serial.print("   ");
    Serial.println(analogRead(llinesen));

  }
}
 
void tests() {
  testWallSen(false);
  testIrSen(false);
  testLineSen(false);
  }
