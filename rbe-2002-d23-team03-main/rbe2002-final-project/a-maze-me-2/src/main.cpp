/**
 * Here you will find our states we used for our second robot
 * Our second robot finds the button and presses it
 * our robot starts off in idle
 * Once the UP_ARROW is pressed it goes into detect
*/

#include <Arduino.h>
#include <robot.h>

/**
 * Most of the accessories, we put in robot.h/.cpp. We put the IR remote here because it's
 * responsible for overall command and control -- it's not something that the robot uses directly
 * for control
 */
#include <ir_codes.h>
#include <IRdecoder.h>
#define IR_PIN 14
IRDecoder decoder(IR_PIN);
enum CHALLENGE_STATE
{
  IDLE,
  TURN,
  DRIVE,
  DETECTING,
  CENTERING,
  CHECK_FOR_IR,
  APPROACH_SENSOR,
  PUSH_BUTTON,
};
CHALLENGE_STATE challengeState = IDLE;

enum ORIENTATION
{
  NORTH,
  EAST,
  SOUTH,
  WEST,
};

ORIENTATION myOrientation = NORTH;

int turntimes = 0;
float lastEntered = 0.0;
int x = 0;
int y = 0;
int location[2] = {x, y};
bool flag = false;
int turnTime2 = 0;

void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println("setup()");

  Serial1.begin(115200);

  decoder.init();

  initialize();
  chassis.init();

  pinMode(13, OUTPUT);

  Serial.println("/setup()");
}

bool irPressed = false;
bool rampReached = false;

float disfront = 0;
int16_t irDist = (int16_t)0;
bool linefollowing = false;

void sendMessage(const String &topic, const String &message)
{
  Serial1.println(topic + String(':') + message);
}

void handleRemote(int16_t keyCode)
{
  Serial.println(keyCode);

  if (keyCode == ENTER_SAVE)
    idle();

  switch (keyCode)
  {
  case UP_ARROW:
    challengeState = DETECTING;
    break;
  case DOWN_ARROW:
    challengeState = IDLE;
    break;
  }
}
void dealWithTurnOrientation()
{
  switch (turnTime2 % 4)
  {
  case 0:
    myOrientation = NORTH;
    break;
  case 1:
    myOrientation = EAST;
    break;
  case 2:
    myOrientation = SOUTH;
    break;
  case 3:
    myOrientation = WEST;
    break;
  }
}
void dealWithLocation()
{
  switch (turntimes % 4)
  {
  case 0:
    turntimes = 0;
    myOrientation = NORTH;
    y++;
    break;
  case 1:
    myOrientation = EAST;
    x++;
    break;
  case 2:
    myOrientation = SOUTH;
    y--;
    break;
  case 3:
    myOrientation = WEST;
    x--;
    break;
  }
}

int targetencoderticks360 = 0; // encoder ticks count for 360 turns
int targetencoderticksRT = 0;  // encoder ticks count for right turn
int targetencoderticksCT = 0;  // encoder ticks count for centering
int targetencoderticksPushButton = 0; // encoder ticks count for pushing button

void loop()
{
  /**
   * Chassis::loop() returns true when the motor control loop fires. We can use that timer to trigger
   * any number of processes that we want to run on the same schedule, for example, the line following
   * controller.
   */
  chassis.loop();

  updateIR();
  irDist = returnIR();
  updatedDis(); //update distance front using ultrasonic
  disfront = returnDis(); //assign distance front

  int16_t keyCode = decoder.getKeyCode();
  if (keyCode != -1)
  {
    handleRemote(keyCode);
  }

  if (challengeState == IDLE)
  {
    linefollowing = false;
    chassis.setTwist(0, 0);
  }


  if (challengeState == DETECTING)
  {
    chassis.setMotorTargetSpeeds(0, 0);
    if (disfront != 0)
    {
      if (disfront <= 15)
      {
        turntimes++;
        turnTime2++;
        dealWithTurnOrientation();
        chassis.setMotorTargetSpeeds(5, -5);
        targetencoderticksRT = chassis.getLeftEncoder() + 800;
        challengeState = TURN;
      }
      else if (x != 0)
      {
        chassis.setMotorTargetSpeeds(5, -5);
        targetencoderticks360 = chassis.getLeftEncoder() + 2850;
        challengeState = CHECK_FOR_IR;
      }
      else
      {
        chassis.setMotorTargetSpeeds(0, 0);
        linefollowing = true;
        challengeState = DRIVE;
      }
    }
  }
  

  if (challengeState == CHECK_FOR_IR)
  {
    chassis.checkMod90(targetencoderticks360);
    if (irDist != (int16_t)1023 && irDist != (int16_t)0)
    {

      if (chassis.getMod90())
      {
        turnTime2++;
        dealWithTurnOrientation();
        flag = false;
        linefollowing = true;
        challengeState = APPROACH_SENSOR;
      }
    }

    if (chassis.getLeftEncoder() >= targetencoderticks360)
    {
      chassis.setMotorTargetSpeeds(0, 0);
      targetencoderticks360 = 0;
      linefollowing = true;
      challengeState = DRIVE;
    }
  }

  if (challengeState == TURN)
  {
    if (chassis.getLeftEncoder() >= targetencoderticksRT)
    {
      targetencoderticksRT = 0;
      chassis.setMotorTargetSpeeds(0, 0);
      challengeState = DETECTING;
    }
  }

  if (challengeState == APPROACH_SENSOR)
  {
    if (checkINTERSECT() && (millis() - lastEntered > 500))
    {
      dealWithLocation();
      String location = String(x) + "," + String(y);
      sendMessage("location", location);
      lastEntered = millis();
    }
    if (disfront <= 5)
    {
      flag = true;
      chassis.setMotorTargetSpeeds(0, 0);
    }
    if (flag == true)
    {
      challengeState = PUSH_BUTTON;
      targetencoderticksPushButton = chassis.getLeftEncoder() + 450;
      chassis.setMotorTargetSpeeds(5,5);
    }
  }

  if (challengeState == PUSH_BUTTON){
    if(chassis.getLeftEncoder() >= targetencoderticksPushButton){
      targetencoderticksPushButton = 0;
      chassis.setTwist(0,0);
      String location = String(x) + "," + String(y) + "," + String(myOrientation);
      sendMessage("escape coordinate", location);
      flag = false;
      challengeState = IDLE;
    }
  }

  if (challengeState == DRIVE)
  {
    if (checkINTERSECT())
    {
      linefollowing = false;
      chassis.setTwist(0, 0);
      dealWithLocation();
      String location = String(x) + "," + String(y);
      sendMessage("location", location);
      chassis.setTwist(8,0);
      targetencoderticksCT = chassis.getLeftEncoder() + 550;
      challengeState = CENTERING;
    }
  }

  if (challengeState == CENTERING)
  {
    if (chassis.getLeftEncoder() >= targetencoderticksCT)
    {
      targetencoderticksCT = 0;
      challengeState = DETECTING;
    }
  }

  updateDrive(linefollowing);
}