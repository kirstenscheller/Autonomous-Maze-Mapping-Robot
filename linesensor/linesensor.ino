#include <Servo.h>

int llinesen = A1; //line sensor on the left
int rlinesen = 4; //line sensor on the right
int rWheelPin = 10; //right wheel
int lWheelPin = 9; //left wheel
int Dpin4 = 4;

int counter = 0;  //counter to keep tract of where the robot is in figure 8
int threshold = 750;
Servo rWheel;
Servo lWheel;


void setup() {
  Serial.begin(9600);
  pinMode(rWheelPin, OUTPUT);
  pinMode(lWheelPin, OUTPUT);
  pinMode(Dpin4, INPUT);
  //pinMode(Dpin4, OUTPUT);
  rWheel.attach(rWheelPin);
  lWheel.attach(lWheelPin);
}

void loop() {

 //Uncomment the following line for testing line sensor values
  test();
  
  
  rWheel.write(90);
  lWheel.write(90);
  
  if (isLeftWhite() && isRightWhite()){
    Serial.println(analogRead(rlinesen));
    Serial.println(analogRead(llinesen));

    rWheel.write(90);
    lWheel.write(90);
    delay(300);
    Serial.println(counter);
  
    switch (counter) {
      case 0:
      case 5:
        if(isLeftWhite() && isRightWhite()){
          turnRight();
          moveForward();
          moveForward();
        } else {
          counter--;
        }
        break;
      case 6:
        if(isLeftWhite() && isRightWhite()){
          turnRight();
          moveForward();
          moveForward();
        } else {
          counter--;
        }
        break;     
      case 1 ... 4:
        if(isLeftWhite() && isRightWhite()){
          turnLeft();
          moveForward();
        } else {
          counter --;
        }
        break;
      default: 
        break;
    } 
    counter ++;
  }
  
  lineFollow();
}

void moveForward() {
  Serial.println("Moving Forward!");
  rWheel.write(87);      
  lWheel.write(93);
  delay(50);
}

void turnRight() {
  Serial.println("Turning Right!");
  //move forward a little bit before turning to prevent overturn
  moveForward();
  moveForward();
  //stop and then turn
  rWheel.write(90);
  lWheel.write(90);
  delay(200);
  rWheel.write(95);
  lWheel.write(180);
  delay(675);
}

void turnLeft() {
  Serial.println("Turning Left!");
  moveForward();
  moveForward();
  rWheel.write(90);
  lWheel.write(90);
  delay(200);

  rWheel.write(0);
  lWheel.write(180);
  delay(675);
}

/*this function is for the robot to readjust itself <br>
if it goes off the white line.*/
void lineFollow() {
  if (isLeftWhite() && (!isRightWhite())){
    Serial.println("Correct line left");
    rWheel.write(85);
    lWheel.write(85);
    delay(100);
  }
  else if ((!isLeftWhite()) && isRightWhite()){
    Serial.println("Correct line right");
    rWheel.write(95);
    lWheel.write(95);
    delay(100);
  } 
  else {    
    moveForward();   
  }
  
}

bool isLeftWhite() {
  return analogRead(llinesen) < threshold;
}

bool isRightWhite() {
  return analogRead(rlinesen) < threshold;
}


void test() {
  while(true){
    rWheel.write(90);
    lWheel.write(90);
    //Serial.print("left  Sensor: ");
    //Serial.println(analogRead(llinesen));
    Serial.print("Right Sensor: ");
    Serial.println(digitalRead(rlinesen));
  }
}
