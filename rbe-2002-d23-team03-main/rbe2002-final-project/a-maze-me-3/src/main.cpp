/**
 * Here you will find our checkers and handlers we used for our third robot
 * Our third robot was the one escaping from the maze
 * In ROBOT_IDLE our robot is reading from MQTT and seeing if we have the readings 
 * it desires that gives it the location and code to flash to the escape door
 * Once it recieves those readings we then press the UP_ARROW to get it into checkIDLE state 
 * which starts the rest of the state machine
 * 
*/

#include <Arduino.h>
#include <robot.h>

/**
 * Most of the accessories, we put in robot.h/.cpp. We put the IR remote here because it's
 * responsible for overall command and control -- it's not something that the robot uses directly
 * for control
 */
#include <ir_codes.h>

enum ORIENTATION
{
  NORTH,
  EAST,
  SOUTH,
  WEST,
};

ORIENTATION myOrientation = NORTH;

int turntimes = 4;
int x = 0;
int y = 0;
int location[2] = {x, y};
float val = 0;

void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println("setup()");

  Serial1.begin(115200);

  initialize();

  chassis.init();

  Serial.println("/setup()");
}

void dealWithLocation()
{
  switch (turntimes % 4)
  {
  case 0:
    turntimes = 4;
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

unsigned long int lastMillEstop = 0;

bool linefollowing = false;

int disFront = 0;
bool onJnow = false;
bool turnfinished = false;
bool ongate = false;
bool signalon = false;

int valueX = 0;
int valueY = 0;
int gateOrien = 5;

void handleDoorOri()
{
  ongate = true;
  handleatDoorState();
  linefollowing = false;
  chassis.setTwist(0,0);
  if (gateOrien == 1 || gateOrien == 2)
  {
    if (gateOrien == 1)
    {
      signalon = true;
    }
    else
    {
      handleRightTurn(700);
    }
  }
  else if(gateOrien == 0 || gateOrien == 3)
  {
    if(gateOrien == 0){
      signalon = true;
    }else{
      handleLEFTTURN(800);
    }
  }
}

void loop()
{
  /**
   * Chassis::loop() returns true when the motor control loop fires. We can use that timer to trigger
   * any number of processes that we want to run on the same schedule, for example, the line following
   * controller.
   */

  updatedDis();
  disFront = returnDis();
  chassis.loop();

  if ((millis() - lastMillEstop) > 100)
  {
    if (handleEstop())
    {
      linefollowing = false;
      idle();
      x = 0;
      y = 0;
      valueX = 0;
      valueY = 0;
    }
    if(signalon == true){
      sendSignal();
    }
    lastMillEstop = millis();
  }

  if(returnState() == ROBOT_IDLE){
    if(checkSerial1()){
      handleMQTT();
    }

    gateOrien = returnOriAprilTag();

    if(gateOrien != 5){
      valueX = returnXAprilTag();
      valueY = returnYAprilTag();
    }
  }

  if (checkIDLE())
  {
    //signalon = true;
    if (gateOrien == 1 || gateOrien == 2)
    {
      handleJ();
      linefollowing = true;
      onJnow = true;
    }
    else if (gateOrien == 0 || gateOrien == 3)
    {
      handleI();
      onJnow = false;
    }
  }

  if (checkINTERSECT())
  {
    linefollowing = false;
    handleCentering(450);
    if(onJnow == true){
      y++;
    }else{
      x++;
    }
  }

  if (checkCENTER() && onJnow == true && ongate == false)
  {
    if(y != valueY)
    {
      handleJ();
      linefollowing = true;
    }
    else
    {
      if (turnfinished != true)
      {
        handleI();
        turnfinished = true;
        onJnow = false;
      }
      else
      {
        handleDoorOri();
      }
    }
  }

  if (checkRIGHTTURN() && onJnow == false && ongate == false)
  {
    turntimes++;
    linefollowing = true;
    linefollowState();
  }

  if (checkCENTER() && onJnow == false && ongate == false)
  {
    if(x != valueX)
    {
      handleI();
      linefollowing = true;
    }
    else
    {
      if (turnfinished != true)
      {
        handleLEFTTURN(900);
        turnfinished = true;
        onJnow = true;
      }
      else
      {
        handleDoorOri();
      }
    }
  }

  if (checkLEFTTURN() && onJnow == true && ongate == false)
  {
    turntimes--;
    handleJ();
    linefollowing = true;
  }

  if (checkRIGHTTURN() && ongate == true){
    signalon = true;
    handleatDoorState();
  }

  if (checkLEFTTURN() && ongate == true){
    signalon = true;
    handleatDoorState();
  } 

  if (disFront > 10 && ongate == true && signalon == true){
    linefollowing = true;
  }

  if (disFront < 5 && ongate == true && signalon == true && linefollowing == true){
    linefollowing = false;
    chassis.setTwist(0,0);
    idle();
  }

  chassis.updatePitch();
  updateDrive(linefollowing);
}