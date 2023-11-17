/**
 * Here you will find our checkers and handlers we used for our first robot
 * Our first robot finds and goes up the ramp and waits until it can read the april tag
 * our robot starts off in idle
 * Once the UP_ARROW is pressed it goes into checkRIGHTTURN which 
 * starts off the rest of the state machine
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

int turntimes = 0;

int x = 0;
int y = 0;
int location[2] = {x, y};

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

bool irPressed = false;
bool rampReached = false;

float val = 0;

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

int AprID = 0;
int AprRot = 0;
String idwithrot;
unsigned long int lastMillMain = 0;
unsigned long int lastMillEstop = 0;
bool linefollowing = false; //flag for setting up linefollowing
bool thirdCol = false; //flag for whether third column has the ramp
bool deadEnd = false; //flag for whether deadend
int disFront = 0;

void loop()
{
  /**
   * Chassis::loop() returns true when the motor control loop fires. We can use that timer to trigger
   * any number of processes that we want to run on the same schedule, for example, the line following
   * controller.
   */
  
  updatedDis(); //update ultrasonic distance reading
  disFront = returnDis(); //assign ultrasonic distance reading

  chassis.loop();
  
  if((millis() - lastMillEstop) > 100){
    if(handleEstop()){
      linefollowing = false;
      idle();
    }
    lastMillEstop = millis();   
  }

  if(checkIDLE()){
    handleRightTurn(700);
  }

  if(checkRIGHTTURN()){
    linefollowState();
    linefollowing = true;
  }

  if(checkINTERSECT()){
    linefollowing = false;
    handleCentering(470);
  }

  if(checkCENTER() && deadEnd == false){
    handleLEFTTURN(940);
  }

  if(checkLEFTTURN() && deadEnd == false){
    if(thirdCol == false){
      handlecheckState();
    }else{
      handlethirdcol();
      linefollowing = true;
    }
  }

  if(disFront > 55){
    if(handleRAMP()){
      linefollowing = true;
    }
    
  }else{
    if(handleBLOCK()){
      thirdCol = true;
    }
  }

  if(checkClimbingRAMP()){
    handleClimbingRAMP();
  }

  if(disFront < 18){
    if(returnState() == ROBOT_APPROACH_RAMP){
      linefollowing = false;
    }
    handleDeadEnd(1730); //180 degree turn
  }

  if (check180turn()){
    handleBacktoY0();
    deadEnd = true;
    thirdCol = true;
    linefollowing = true;
  }

  if(checkY0()){
    linefollowing = false;
    handleCentering(450);
    
  }

  if(checkCENTER() && deadEnd == true){
    if(disFront <= 15){
      handleLEFTTURN(940);
      linefollowing = false;
    }else{
      handleBacktoY0();
      linefollowing = true;
    }
  }

  if(checkLEFTTURN() && deadEnd == true){
    linefollowState();
    linefollowing = true;
    deadEnd = false;
  }

  if(checkONRAMP()){
    linefollowing = false;
    handleONRAMP(550);
  }

  if(checkApproachTag()){
    handleApproachTag();
  }

  if(checkTag()){
    AprID = returnID();
    AprRot = returnRot();
    idle();
  }

  chassis.updatePitch();
  updateDrive(linefollowing); //update whether linefollowing

  if (AprID != 0)
  {
    if ((millis() - lastMillMain) > 50)
    {
      int xVal = AprID / 10;
      int yVal = AprID % 10;
      if ((AprRot >= 0 && AprRot < 30) || (AprRot > 330 && AprRot <= 360)){
        sendMessage("orientation", String(0));//facing north
      }
      if (AprRot > 240 && AprRot < 300){
        sendMessage("orientation", String(1));//facing east
      }
      if (AprRot > 150 && AprRot < 210){
        sendMessage("orientation", String(2));//facing south
      }
      if (AprRot > 60 && AprRot < 120){
        sendMessage("orientation", String(3));//facing west
      }
      idwithrot = String(AprID) + "," +String(AprRot); //used to double check we have the correct reading
      //sendMessage("idwithrot", idwithrot);
      sendMessage("xVal", String(xVal));
      sendMessage("yVal", String(yVal));
      lastMillMain = millis();
    }
  }
}