/**
 * Here we declare all of our functions used in robot.cpp
 * We have checker and handler declared
 * Some useful functions for getting April Tag and X,Y, orientation
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

    ROBOT_AT_DOOR
};

void initialize(void);

void idle(void);

int16_t returnDis(void);
void updatedDis(void);
int16_t returnState(void);


void linefollow(void);
void updateDrive(bool drive);
bool handleEstop(void);
void sendMessage(const String &topic, const String &message);

void handleMQTT();

int returnXAprilTag();
int returnYAprilTag();
int returnOriAprilTag();

int returnXcoordinate();
int returnYcoordinate();
int returnOrientation();
void sendSignal();
bool checkSerial1(void);

bool checkIDLE(void);
void handleIDLE(void);
void handleJ(void);
void handleI(void);

void handleRightTurn(int turnticks);
bool checkRIGHTTURN(void);
void handleCentering(int driveTicks);
bool checkCENTER(void);
void linefollowState(void);
bool checkINTERSECT(void);
void handleLEFTTURN(int turnticks);
bool checkLEFTTURN(void);
void handleatDoorState(void);

void showState(void);