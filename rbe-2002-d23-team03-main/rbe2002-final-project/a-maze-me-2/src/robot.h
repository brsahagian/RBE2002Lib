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
    ROBOT_STANDOFF, 
    ROBOT_LEFT,
    ROBOT_RIGHT,
    ROBOT_DRIVING,
    ROBOT_LINING,
    ROBOT_CENTERING,    //for centering on an intersection
    ROBOT_WAITING,      //waiting for an observation
};

void initialize(void);

void idle(void);

void processDistanceSensors(void);
void handleNewDistanceReading(float distanceReading);
int16_t returnIR();
void updateIR(void);

void linefollow(void);
bool checkINTERSECT(void);
float checkDistancefront(void);
int16_t returnDis(void);
void updatedDis(void);

void updateDrive(bool drive);