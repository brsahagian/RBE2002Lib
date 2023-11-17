#include <Arduino.h>
#include <Chassis.h>
#include <Romi32U4Motors.h>
#include <LSM6.h>
#include <Wire.h>

// We'll declare motors as global to make the ISRs happier, but we'll put them in Chassis.cpp
// to keep things organized

LeftMotor leftMotor;
RightMotor rightMotor;
LSM6 imu;

Chassis::Chassis(void) {}

void Chassis::init(void)
{
    noInterrupts(); // disable interupts while we mess with the Timer4 registers

    // sets up timer 4
    TCCR4A = 0x00; // disable some functionality -- no need to worry about this
    TCCR4B = 0x0A; // sets the prescaler -- look in the handout for values
    TCCR4C = 0x04; // toggles pin 6 at the timer frequency
    TCCR4D = 0x00; // normal mode

    /**
     * Here we do a little trick to allow full 10-bit register access.
     * We have 2 bytes in TC4H that we can use to add capacity to TOP.
     * In the end, this sets TOP = 2 * 256 + 112 = 624
     */
    TC4H = 2;
    OCR4C = 112;

    TIMSK4 = 0x04; // enable overflow interrupt

    interrupts(); // re-enable interrupts

    // init the motors
    Romi32U4Motor::init();
    Wire.begin();

    // init the IMU
    imu.init();
    // imu.setFullScaleAcc(imu.ACC_FS4);
    imu.setFullScaleGyro(imu.GYRO_FS500);
    imu.setAccDataOutputRate(imu.ODR52);
    imu.setGyroDataOutputRate(imu.ODR52);

    imu.init();

    // pinMode(6, OUTPUT); //COMMENT THIS OUT TO SHUT UP THE PIEZO!!!
}

bool Chassis::checkForNewIMUData()
{
    return (imu.getStatus() & 0x01);
}

bool Chassis::loop(void)
{
    bool retVal = false;
    if (readyToPID)
    {
        if (readyToPID > 1)
            Serial.println("Missed update in Chassis::loop()");

        update();
        readyToPID = 0;
        retVal = true;
    }

    if (checkForNewIMUData())
    {
        updatePitch();
        //checkForStop();
        //Serial.println(checkForStop());
    }

    return retVal;
}

void Chassis::update(void)
{
    leftMotor.update();
    rightMotor.update();

    updatePose();
    
#ifdef __MOTOR_DEBUG__
    Serial.print('\n');
#endif
}

void Chassis::setMotorTargetSpeeds(float leftTicksPerInterval, float rightTicksPerInterval)
{
    leftMotor.setTargetSpeed(leftTicksPerInterval);
    rightMotor.setTargetSpeed(rightTicksPerInterval);
}

void Chassis::setWheelTargetSpeeds(float leftLinearSpeed, float rightLinearSpeed)
{
    leftMotor.setTargetSpeed(leftLinearSpeed * 1.3);
    rightMotor.setTargetSpeed(rightLinearSpeed * 1.3);
}

void Chassis::setTwist(float forwardSpeed, float turningSpeed)
{
    setWheelTargetSpeeds(forwardSpeed - turningSpeed, forwardSpeed + turningSpeed);
}

void Chassis::driveFor(float forwardDistance, float forwardSpeed, bool block)
{
    // ensure the speed and distance are in the same direction
    forwardSpeed = forwardDistance > 0 ? fabs(forwardSpeed) : -fabs(forwardSpeed);
    setTwist(forwardSpeed, 0); // sets the speeds

    // calculate the total motion in encoder ticks
    int16_t delta = forwardDistance / cmPerEncoderTick;

    // set both wheels to move the same amount
    leftMotor.moveFor(delta);
    rightMotor.moveFor(delta);

}

//true for right turn, false for left turn
//720 ticks for 90 turn
void Chassis::turnFor(float turnTicks, float turnSpeed, bool dir)
{
    if (dir) 
    {
        setTwist(0, -turnSpeed);
        leftMotor.moveFor(turnTicks);
        rightMotor.moveFor(-turnTicks);
    }else{
        setTwist(0, turnSpeed);
        leftMotor.moveFor(-turnTicks);
        rightMotor.moveFor(turnTicks);
    }
}

bool Chassis::checkMotionComplete(void)
{
    bool complete = leftMotor.checkComplete() && rightMotor.checkComplete();
    return complete;
}


int initialEncoderValue;  // variable to store initial encoder value
int targetEncoderValue;   // variable to store target encoder value
int currentEncoderValue;  // variable to store current encoder value

static bool initialized = false;  // flag to track initialization

bool Chassis::driveForTicks(float forwardTics, float forwardSpeed)
{   
    if(!initialized){
        initialEncoderValue = leftMotor.getCount();
        targetEncoderValue = initialEncoderValue + forwardTics;
        initialized = true;
    }
    //Serial.println("============");
    setTwist(forwardSpeed,0);
    currentEncoderValue = leftMotor.getCount();

    if(currentEncoderValue >= targetEncoderValue){
        setTwist(0,0);
        initialized = false;
        return true;
    }

    return false;
}

int initialEncoderValueTurn;  // variable to store initial encoder value
int targetEncoderValueTurn;   // variable to store target encoder value
int currentEncoderValueTurn;  // variable to store current encoder value

bool initializedTurn = false;  // flag to track initialization
float currentPos;
float targetPos;
bool initialize = false; 

bool Chassis::getMod90() {
    return atMod90;
}

void Chassis::checkMod90(int targetTicks){
    int inRange = leftMotor.getCount() - targetTicks;
    if(abs(inRange) % 670 <= 100){
        atMod90 = true;
    }else{
        atMod90 = false;
    }
}

bool Chassis::turnRight(float ticks){
    if (!initialize) {
        currentPos = leftMotor.getCount();
        targetPos = currentPos + ticks;
        initialize = true;
        atMod90 = false;
    }
    
    int inRange = leftMotor.getCount() - targetPos;
    if(abs(inRange) % 670 <= 100){
        atMod90 = true;
    }else{
        atMod90 = false;
    }
    
    if (leftMotor.getCount() <= targetPos){
        setMotorTargetSpeeds(5,-5);
        return false;
    } else {
        initialize = false;
        atMod90 = false;
        return true;
    }
}

bool initializeTurnLeft = false;
bool Chassis::turnLeft(float ticks){
    if (!initializeTurnLeft) {
        currentPos = leftMotor.getCount();
        targetPos = currentPos + ticks;
        initializeTurnLeft = true;
    }
    
    if (leftMotor.getCount() <= targetPos){
        setMotorTargetSpeeds(-5,5);
        return false;
    } else {
        initializeTurnLeft = false;
        return true;
    }
}

bool initializeEncoderMove = false; 

bool Chassis::encoderMove(float ticks){
    if (!initializeEncoderMove) {
        currentPos = leftMotor.getCount();
        targetPos = currentPos + ticks;
        initializeEncoderMove = true;
    }
    
    if (leftMotor.getCount() <= targetPos){
        setMotorTargetSpeeds(5,5);
        return false;
    } else {
        initializeEncoderMove = false;
        return true;
    }
}

bool initializeInIRrange = false;
bool Chassis::inIRrange(float ticks) {
    if (!initializeInIRrange) {
        currentPos = leftMotor.getCount();
        targetPos = currentPos + ticks;
        initializeInIRrange = true;
    }
    int range = leftMotor.getCount() - targetPos;
    if (abs(range)<=250) {
        return true;
    } else {
        return false;
    }
}

int16_t Chassis::getLeftEncoder(void){
    return leftMotor.getCount();
}

int16_t Chassis::getRightEncoder(void){
    return rightMotor.getCount();
}

bool Chassis::turnForTicks(float turnTicks, float forwardSpeed, bool right)
{
    if(!initializedTurn){
        if(right){
            initialEncoderValueTurn = leftMotor.getCount();
        }else{
            initialEncoderValueTurn = rightMotor.getCount();
        }
        targetEncoderValueTurn = initialEncoderValueTurn + turnTicks;
        initializedTurn = true;
    }

    if (right){
        setWheelTargetSpeeds(forwardSpeed, -forwardSpeed);
        currentEncoderValue = leftMotor.getCount();
    }else{
        setWheelTargetSpeeds(-forwardSpeed, forwardSpeed);
        currentEncoderValue = rightMotor.getCount();
    }
    
    if(currentEncoderValue >= targetEncoderValue){
        setTwist(0,0);
        initialized = false;
        return true;
    }

    return false;
}

/*
 * ISR for timing. On overflow, it takes a 'snapshot' of the encoder counts and raises a flag to let
 * the main program it is time to execute the PID calculations.
 */
ISR(TIMER4_OVF_vect)
{
    // Capture a "snapshot" of the encoder counts for later processing
    leftMotor.calcEncoderDelta();
    rightMotor.calcEncoderDelta();

    chassis.readyToPID++;
}

int stopThreshold = 2;
static long prevPosition = 0;
bool Chassis::checkForStop(){
  long currentPosition = leftMotor.getCount();
  if (abs(currentPosition - prevPosition) < stopThreshold) {
    Serial.println("stopped");
    return true;
  } else {
    prevPosition = currentPosition;
    return false;
  }
}


unsigned long int lastMill = 0;
unsigned long int currMill = 0;
float bias = 8.9; // bias for gyro
float angleBias = 0;
float gyroBias = 0.02;

float k = 0.5;

bool Chassis::updatePitch()
{
    if (imu.getStatus() & 0x01)
    {

        // digitalWrite(13, HIGH);
        imu.read();
        // digitalWrite(13, LOW);
        currMill = millis();

        float xmg = imu.a.x / imu.mgPerLSBGL;
        float ymg = imu.a.y / imu.mgPerLSBGL;
        float zmg = imu.a.z / imu.mgPerLSBGL;

        float gy = ((imu.g.y / 1000.0f) - angleBias) * imu.mdpsPerLSBGL; // degree per second
        float deltat = (currMill - lastMill) / 1000.0f;

        float predictPitchAngle = estimatedPitchAngle + gy * deltat; // pitch angle by gyro
        float observedPitchAngle = 180 * (atan2(-xmg, zmg) / M_PI);  // pitch angle by accr
        lastMill = currMill;

        estimatedPitchAngle = predictPitchAngle * (1.0f - k) + observedPitchAngle * k;

        angleBias = angleBias - gyroBias * (observedPitchAngle - predictPitchAngle) / (imu.mdpsPerLSBGL * deltat);

        // float thetaz = 360 * (atan2(-ymg, xmg) / M_PI);
        /*
        Serial.print(millis());
        Serial.print('\t');
        Serial.print(angleBias);
        Serial.print('\t');
        Serial.print(predictPitchAngle);
        Serial.print('\t');
        Serial.print(observedPitchAngle);
        Serial.print('\t');
        Serial.println(estimatedPitchAngle);
        */
    }

    return false;
}


void Chassis::updatePose(void) {
    float dLeft = leftMotor.speed * cmPerEncoderTick;
    float dRight = rightMotor.speed * cmPerEncoderTick;

    float d = (dLeft + dRight)/ 2.0;
    currPoseTheta = prevPoseTheta + (dRight-dLeft)/ wheel_track; //heading
    currPoseThetaAvg = (prevPoseTheta + currPoseTheta)/2.0;
    currPoseX += d * cos(currPoseThetaAvg);
    currPoseY += d * sin(currPoseThetaAvg);
    prevPoseTheta  = currPoseTheta;
}
