/**
 * Here we declare all of our functions used in robot.cpp
 * We have checker and handler declared
*/
#pragma once

#include <Arduino.h>
#include <Chassis.h>
#include <HC-SR04.h>

enum ROBOT_STATE
{   
    ROBOT_IDLE, 
    ROBOT_RIGHTTURN, 
    ROBOT_LEFTTURN,
    ROBOT_CENTERING,
    ROBOT_DRIVE_LINE,
    ROBOT_CHECK_RAMP,
    ROBOT_APPROACH_RAMP,
    ROBOT_CLIMB_RAMP,
    ROBOT_APPROACH_TAG,
    ROBOT_CHECK_TAG,    //for centering on an intersection
    ROBOT_BACK_TO_Y0,
    ROBOT_180TURN
};

void initialize(void);

void idle(void);

void handleKeyCode(int16_t keyCode);

void processDistanceSensors(void);
void handleNewDistanceReading(float distanceReading);
float checkDistancefront(void);
int16_t returnDis(void);
void updatedDis(void);
int16_t returnState(void);
uint8_t FindAprilTags(void);
int returnID(void);
int returnRot(void);


void linefollow(void);
void updateDrive(bool drive);
bool handleEstop(void);
void sendMessage(const String &topic, const String &message);


bool checkIDLE(void);
void handleRightTurn(int turnticks);
bool checkRIGHTTURN(void);
void handleCentering(int driveTicks);
bool checkCENTER(void);
void linefollowState(void);
bool checkINTERSECT(void);
void handleLEFTTURN(int turnticks);
bool checkLEFTTURN(void);
void handlecheckState(void);
bool checkRamp(void);
void handlethirdcol(void);
bool handleRAMP(void);
bool handleBLOCK(void);
bool checkClimbingRAMP(void);
void handleClimbingRAMP(void);
bool checkONRAMP(void);
void handleONRAMP(int driveticks);
bool checkApproachTag(void);
void handleApproachTag(void);
bool checkTag(void);

bool checkClimbingRAMP(void);
bool checkDeadEnd(void);
void handleDeadEnd(int turnticks);
bool check180turn(void);
void handleBacktoY0(void);
bool checkY0(void);

void showState(void);