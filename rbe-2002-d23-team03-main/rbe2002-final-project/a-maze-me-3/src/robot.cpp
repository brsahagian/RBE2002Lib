/**
 * Here you will find the handler functions we used in main.cpp
 * as well as the checker functions
 * We also initialize the rangefinder and IR_PIN
 * We have our function that makes the 38kHz in here
*/
#include <robot.h>
#include <Chassis.h>
#include <HC-SR04.h>
#include <ir_codes.h>
#include <Arduino.h>
#include <Wire.h>
#include <openmv.h>
#include <IRdecoder.h>

HC_SR04 hc_sr04(17, 4);
OpenMV camera;

#define IR_PIN 14
IRDecoder decoder(IR_PIN);

void ISR_HC_SR04(void)
{
  hc_sr04.ISR_echo();
}

float distanceSR04 = 0;
float lastTime = 0.0;
String serString1;

ROBOT_STATE robotState = ROBOT_IDLE;
Chassis chassis;
bool tooFar = true;
float TARGETDISTANCECLOSE = 25;
float TARGETDISTANCEFAR = 40;
float Kp = 1.5;
float Kd = .1;
float lastError;

uint8_t X = 0;
uint8_t Y = 0;
uint8_t direction = 0;
int givenX;
int givenY;
int givenOrientation = 5;
uint8_t checkSum = 0;
uint32_t XORcombination;

void initialize(void)
{
  chassis.init();

  hc_sr04.init(ISR_HC_SR04);

  decoder.init();

  pinMode(A0, INPUT);
  pinMode(A3, INPUT);
}

bool handleEstop()
{
  if (decoder.getKeyCode() == DOWN_ARROW)
  {
    return true;
  }
  return false;
}

void showState(void)
{
  Serial.println(robotState);
}

int16_t returnState(void)
{
  return robotState;
}

void sendMessage(const String &topic, const String &message)
{
  Serial1.println(topic + String(':') + message);
}

bool checkSerial1(void)
{
  while (Serial1.available())
  {
    char c = Serial1.read();
    serString1 += c;

    if (c == '\n')
    {
      return true;
    }
  }

  return false;
}

void handleMQTT(){ //updating SerString1 to be ""
    Serial.print("Rec'd:\t");
    Serial.print(serString1);
    char *token;
    char charArray[20];

    if (serString1.indexOf("xVal")>=0) { 
      serString1.toCharArray(charArray, serString1.length()+1);
      token = strtok(charArray,":"); //before :
      token = strtok(NULL,":");
      givenX = atoi(token);

    } 
    if (serString1.indexOf("yVal")>=0) { 
      serString1.toCharArray(charArray, serString1.length()+1);
      token = strtok(charArray,":"); //before :
      token = strtok(NULL,":");
      givenY = atoi(token);
    } 

    if (serString1.indexOf("tation")>0) { 
      serString1.toCharArray(charArray, serString1.length()+1);
      token = strtok(charArray,":"); //before :
      token = strtok(NULL,":");
      givenOrientation = atoi(token);
    } 

    if (serString1.indexOf("coordinate")>0) { //meaning it's in the string
      serString1.toCharArray(charArray, serString1.length()+1);
      token = strtok(charArray,":"); //before :
      token = strtok(NULL,":"); // after : which should be which should be number,number, direction
      token = strtok(token,",");
      X = int(token[0]);
      token = strtok(NULL,",");
      Y = int(token[0]);
      token = strtok(NULL,",");
      direction = int(token[0]);

      checkSum = X ^ Y ^ direction;
      XORcombination = ( X<< 8) | Y;
      XORcombination = (XORcombination << 8) | direction;
      XORcombination = (XORcombination << 8) | checkSum;

    }
    serString1 = "";
}

int returnXAprilTag() {
  return givenX;
}

int returnYAprilTag() {
  return givenY;
}

int returnOriAprilTag() {
  return givenOrientation;
}

int returnXcoordinate(){
  return X;
}
int returnYcoordinate(){
  return Y;
}
int returnOrientation() {
  return direction;
}


void sendSignal() {
  //start pulse
  TCCR1A |= 0b00001000; //on
  delayMicroseconds(9000);
  TCCR1A &= 0b11110111; //off
  delayMicroseconds(4500);

  for (int i =0;i<32;i++) {

    if (bitRead(XORcombination,i)==1) {
    TCCR1A |= 0b00001000; //on
    delayMicroseconds(562.5);
    TCCR1A &= 0b11110111; //off
    delayMicroseconds(1687.5);
    } else {
    TCCR1A |= 0b00001000; //on
    delayMicroseconds(562.5);
    TCCR1A &= 0b11110111; //off
    delayMicroseconds(562.5);
    }
  }
  //end pulse
  TCCR1A |= 0b00001000; //on
  delayMicroseconds(562.5);
  TCCR1A &= 0b11110111; //off

}


//////////////////////////////////////////////////////////////////
// state machine code below

bool checkIDLE(void)
{
  if (robotState == ROBOT_IDLE)
  {
    if (decoder.getKeyCode() == UP_ARROW)
    {
      return true;
    }
  }
  return false;
}

void handleJ(void){
  linefollowState();
  //set linefollow in main
}

void handleI(void){
  handleRightTurn(650);
}

int targetEncoderCountRT = 0;
void handleRightTurn(int turnticks)
{
  chassis.setTwist(0, -5);
  targetEncoderCountRT = chassis.getLeftEncoder() + turnticks;
  robotState = ROBOT_RIGHTTURN;
}

bool checkRIGHTTURN(void)
{
  if (robotState == ROBOT_RIGHTTURN)
  {
    if (chassis.getLeftEncoder() >= targetEncoderCountRT)
    {
      chassis.setTwist(0, 0);
      targetEncoderCountRT = 0;
      return true;
    }
  }
  return false;
}

int targetEncoderCountFD = 0;
void handleCentering(int driveTicks)
{
  // set line follow off
  chassis.setTwist(10, 0);
  targetEncoderCountFD = chassis.getLeftEncoder() + driveTicks;
  robotState = ROBOT_CENTERING;
}

bool checkCENTER(void)
{
  if (robotState == ROBOT_CENTERING)
  {
    if (chassis.getLeftEncoder() >= targetEncoderCountFD)
    {
      chassis.setTwist(0, 0);
      targetEncoderCountFD = 0;
      return true;
    }
  }
  return false;
}

void linefollowState(void)
{
  // set line follow on
  robotState = ROBOT_DRIVE_LINE;
}

bool checkINTERSECT(void)
{
  if (robotState == ROBOT_DRIVE_LINE)
  {
    int line_det = analogRead(A3);
    if (line_det < 400)
    {
      chassis.setTwist(0, 0);
      return true;
    }
  }
  return false;
}

int targetEncoderCountLF = 0;
void handleLEFTTURN(int turnticks)
{
  chassis.setTwist(0, 5);
  targetEncoderCountLF = chassis.getRightEncoder() + turnticks;
  robotState = ROBOT_LEFTTURN;
}

bool checkLEFTTURN(void)
{
  if (robotState == ROBOT_LEFTTURN)
  {
    if (chassis.getRightEncoder() >= targetEncoderCountLF)
    {
      chassis.setTwist(0, 0);
      targetEncoderCountLF = 0;
      return true;
    }
  }
  return false;
}

void handleatDoorState(void){
  robotState = ROBOT_AT_DOOR;
}


void idle(void)
{
  Serial.println("Idling!");
  chassis.setMotorEfforts(0, 0);
  robotState = ROBOT_IDLE;
}

//////////////////////////////////////////////////////////////////////////////////////

void updateDrive(bool drive)
{
  if (drive)
  {
    linefollow();
  }
}

void linefollow(void)
{
  int line_err = analogRead(A0) - 385; // get err to the white line, robot should be on the left of the line
  chassis.setTwist(10, -line_err * 0.01); // P control line follow
}


float distanceReading = 0;
int16_t returnDis(void)
{ 
  return distanceReading;//to make it compile without warnings
}

void updatedDis(void){
  bool hasNewReading = hc_sr04.getDistance(distanceReading);
  if (hasNewReading){
    //Serial.print("Ultra sonic updated");
  }
}