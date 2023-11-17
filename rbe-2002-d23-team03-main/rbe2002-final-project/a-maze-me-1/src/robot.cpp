/**
 * Here you will find the handler functions we used in main.cpp
 * as well as the checker functions
 * We also initialize the rangefinder and IR_PIN
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

ROBOT_STATE robotState = ROBOT_IDLE;
Chassis chassis;
bool tooFar = true;
float TARGETDISTANCECLOSE = 25;
float TARGETDISTANCEFAR = 40;
float Kp = 1.5;
float Kd = .1;
float lastError;
void initialize(void)
{
  chassis.init();

  hc_sr04.init(ISR_HC_SR04);

  Wire.setClock(100000ul);

  decoder.init();

  pinMode(A0, INPUT); //line follower pin for linefollowing
  pinMode(A3, INPUT); //line follower pin for line detect

  pinMode(13, OUTPUT);
}

bool handleEstop()
{ 
   if (decoder.getKeyCode() == DOWN_ARROW)
   {
     return true;
   }
  return false;
}

void showState(void){
  Serial.println(robotState);
}

int16_t returnState(void){
  return robotState;
}

void sendMessage(const String &topic, const String &message)
{
  Serial1.println(topic + String(':') + message);
}


//////////////////////////////////////////////////////////////////
//state machine code below

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
  //set line follow off
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
  //set line follow on in main
  robotState = ROBOT_DRIVE_LINE;
}

bool checkINTERSECT(void)
{
  if (robotState == ROBOT_DRIVE_LINE)
  {
    int line_det = analogRead(A3);
    if (line_det < 400) //detect on white
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

void handlecheckState(void)
{
  chassis.setTwist(0, 0);
  robotState = ROBOT_CHECK_RAMP;
}

void handlethirdcol(void){
  robotState = ROBOT_APPROACH_RAMP;
}

bool checkRamp(void)
{
  int disFront = returnDis();
  if (disFront > 50) //check front dis
  {
    return true;
  }
  
  return false;
}

bool handleRAMP()
{
  if (robotState == ROBOT_CHECK_RAMP){
    robotState = ROBOT_APPROACH_RAMP;
    return true;
  }
  return false;
}

bool handleBLOCK(){
  if (robotState == ROBOT_CHECK_RAMP){
    handleRightTurn(700);
    return true;
  }
  return false;
}
  
bool checkClimbingRAMP()
{
    if (chassis.estimatedPitchAngle < -8.0)
    {
      return true;
    }
  
  return false;
}

bool checkDeadEnd(){
  int disFront = returnDis();
  if (disFront < 15)
  {
    return true;
  }
  return false;
}

int targetEncoderCount180 = 0;
void handleDeadEnd(int turnticks){
  if(robotState == ROBOT_APPROACH_RAMP){
    chassis.setTwist(0,5);
    targetEncoderCount180 = chassis.getRightEncoder() + turnticks;
    robotState = ROBOT_180TURN;
  }
}

bool check180turn(void){
  if(robotState == ROBOT_180TURN){
    if(chassis.getRightEncoder() >= targetEncoderCount180){
      chassis.setTwist(0,0);
      targetEncoderCount180 = 0;
      return true;
    }
  }
  return false;
}

void handleBacktoY0(void){
  robotState = ROBOT_BACK_TO_Y0;
}

bool checkY0(void){
  if(robotState == ROBOT_BACK_TO_Y0){
    int line_det = analogRead(A3);
    if (line_det < 400)
    {
      return true;
    }
  }
  return false;
}

void handleClimbingRAMP()
{
  if (robotState == ROBOT_APPROACH_RAMP)
  {
  digitalWrite(13, HIGH);
  robotState = ROBOT_CLIMB_RAMP;
  }
}

bool checkONRAMP()
{
  if (robotState == ROBOT_CLIMB_RAMP)
  {
    if (chassis.estimatedPitchAngle >= -1.0)
    {
      return true;
    }
  }
  return false;
}

int targetEncoderCountTAG = 0;
void handleONRAMP(int driveticks)
{
  // set line follow off
  chassis.setTwist(5, 0);
  digitalWrite(13, LOW);
  targetEncoderCountTAG = chassis.getLeftEncoder() + driveticks;
  robotState = ROBOT_APPROACH_TAG;
}

bool checkApproachTag()
{
  if (robotState == ROBOT_APPROACH_TAG)
  {
    if (chassis.getLeftEncoder() >= targetEncoderCountTAG)
    {
      targetEncoderCountTAG = 0;
      return true;
    }
  }
  return false;
}

void handleApproachTag()
{
  chassis.setTwist(0, 0);
  robotState = ROBOT_CHECK_TAG;
}

bool checkTag()
{
  if (robotState == ROBOT_CHECK_TAG)
  {
    if (FindAprilTags())
    {
      return true;
    }
  }
  return false;
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

int idVal;
int rotVal;
uint8_t FindAprilTags(void)
{
  uint8_t tagCount = camera.getTagCount();
  if (tagCount)
  {
    Serial.println(tagCount);
    AprilTagDatum tag;
    if (camera.readTag(tag))
    {
      Serial.print(F("Tag [cx="));
      Serial.print(tag.cx);
      Serial.print(F(", cy="));
      Serial.print(tag.cy);
      Serial.print(F(", w="));
      Serial.print(tag.w);
      Serial.print(F(", h="));
      Serial.print(tag.h);
      Serial.print(F(", id="));
      Serial.print(tag.id);
      Serial.print(F(", rot="));
      Serial.print(tag.rot);
      Serial.println(F("]"));
      idVal = tag.id;
      rotVal = tag.rot;
      Serial.print(idVal);
      Serial.print("\t");
      Serial.println(rotVal);
    }
  }

  return tagCount;
}

int returnID(void)
{
  return idVal;
}

int returnRot(void)
{
  return rotVal;
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
