#pragma once

#include <Arduino.h>
#include <Romi32U4Motors.h>

class Chassis
{
protected:
    // Kinematic parameters -- TODO: These are grossly wrong!!!
    float wheel_track = 14.2; //cm
    float wheel_diam = 7; //cm
    float ticks_per_rotation = 1440; // from the datasheet 1440
    float cmPerEncoderTick = 3.1416 * wheel_diam / ticks_per_rotation;
    float robotRadius = wheel_track / 2.0;
    float currPoseTheta = 0.0;
    float prevPoseTheta = PI/2.0;//cause starting north
    float currPoseThetaAvg = 0.0;

public:
    uint8_t readyToPID = 0;
    bool firsttime = true;

    Chassis(void);

    void init(void);
    bool loop(void);
    void update(void);
    bool updatePitch(void);
    void updatePose(void);

    bool checkForNewIMUData();

    void setMotorEfforts(int16_t left, int16_t right) 
        {leftMotor.setMotorEffort(left); rightMotor.setMotorEffort(right);}

    void setMotorTargetSpeeds(float leftTicksPerInterval, float rightTicksPerInterval);

    void setWheelTargetSpeeds(float leftLinearSpeed, float rightLinearSpeed);

    void setTwist(float fowardSpeed, float turningSpeed);

    void driveFor(float forwardDistance, float forwardSpeed, bool block = false);
    void turnFor(float turnTicks, float turnSpeed, bool dir);
    bool driveForTicks(float forwardTicks, float forwardSpeed);
    bool turnForTicks(float turnTicks, float forwardSpeed, bool right = true);

    bool checkForStop(void);
    bool turnRight(float ticks);
    bool turnLeft(float ticks);
    bool encoderMove(float ticks);
    bool inIRrange(float ticks);
    bool atMod90 = false;
    bool getMod90();
    void checkMod90(int targetTicks);
    bool checkMotionComplete(void);

    void resetfirstTime(void){firsttime = true;}
    float estimatedPitchAngle = 0;


    int16_t getLeftEncoder(void);
    int16_t getRightEncoder(void);

    float currPoseX = 0.0;
    float currPoseY = 0.0;

};

extern Chassis chassis;