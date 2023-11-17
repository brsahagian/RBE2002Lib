/**
 * Here you will find the handler functions we used in main.cpp
 * as well as the checker functions
 * We also initialize the rangefinder and IR_PIN
*/

#include <robot.h>
#include <Chassis.h>
#include <HC-SR04.h>
#include <ir_codes.h>
#include <IRDirectionFinder.h>

IRDirectionFinder irFinder;
HC_SR04 hc_sr04(17, 4);

void ISR_HC_SR04(void)
{
    hc_sr04.ISR_echo();
}

float distanceSR04 = 0;
float lastTime = 0.0;

ROBOT_STATE robotState = ROBOT_IDLE;
Chassis chassis;
bool tooFar = true;
float TARGETDISTANCECLOSE =25;
float TARGETDISTANCEFAR =40;
float Kp = 1.5;
float Kd = .1;
float lastError;
void initialize(void)
{
    chassis.init();

    hc_sr04.init(ISR_HC_SR04);
    irFinder.begin();

    pinMode(A0, INPUT);
    pinMode(A3, INPUT);

}

void idle(void)
{
    Serial.println("Idling!");
    chassis.setMotorEfforts(0, 0);
    robotState = ROBOT_IDLE;
}

String keyCodeString; // this may come in handy later

Point point;
int16_t returnIR(){
  return point.x;
}

void updateIR(void){
  static uint32_t lastIRread = 0;
  if(millis() - lastIRread > 50)
  {
    irFinder.requestPosition();
    lastIRread = millis();
  } 
  if (irFinder.available()) {
      for (size_t i = 0; i < 4; i++)
      {
        point = irFinder.ReadPoint(i);
      }
  }
}

void linefollow(void) {
  int line_err = analogRead(A0) - 385; //get err to the white line, robot should be on the left of the line
  chassis.setTwist(10, -line_err*0.01);
}

void updateDrive(bool drive)
{
  if (drive)
  {
    linefollow();
  }
}

bool checkINTERSECT(void)
{
    int line_det = analogRead(A3);
    if (line_det < 400) // hit white line
    {
      chassis.setTwist(0, 0);
      return true;
    }
  return false;
}

float checkDistancefront(void){
    float distanceReading = 0;
    hc_sr04.getDistance(distanceReading);
    return distanceReading;
    /*if (hasNewReading){
      //Serial.println(distanceReading);
      if (distanceReading < 15){
        //Serial.println('yes');
        return true;
      }else{
        //Serial.println('no');
        return false;
      }
    }*/
}

void processDistanceSensors(void)
{
    /** Check the distance sensor.
     * We return true only if there is a new reading, which is passed by reference.
     * It hardly needs to be done this way, but passing by reference is a useful tool,
     * so we start with a 'lightweight' example here.
     */
    float distanceReading = 0;
    bool hasNewReading = hc_sr04.getDistance(distanceReading);
    if(hasNewReading){
      distanceSR04 = checkDistancefront();
    }
}

float distanceReading = 0;
int16_t returnDis(void){
    return distanceReading;
}

void updatedDis(void){
  bool hasNewReading = hc_sr04.getDistance(distanceReading);
  if (hasNewReading){
    //Serial.print("Ultra sonic updated");
  }
}

void handleNewDistanceReading(float distanceReading)
{
#ifdef __DEBUG_RANGEFINDER__
    Serial.println(distanceReading);
#endif
    if(robotState == ROBOT_STANDOFF)
    {
      /**
       * TODO: Add bang-bang control with hysteresis
      */
     
    if (tooFar){
      if (distanceReading > 25) {
        float error = distanceReading - TARGETDISTANCECLOSE;
        float errord = error - lastError;
        chassis.setWheelTargetSpeeds(error*Kp+errord*Kd,error*Kp+errord*Kd);
        lastError = error;
      } else {
        tooFar = false;
      }
      } else {
        if (distanceReading < 40) {
          float error = distanceReading - TARGETDISTANCEFAR;
          float errord = error - lastError;
        chassis.setWheelTargetSpeeds(error*Kp+errord*Kd,error*Kp+errord*Kd);
        lastError = error;
        } else {
          tooFar = true;
        }
      }
      
    }
}