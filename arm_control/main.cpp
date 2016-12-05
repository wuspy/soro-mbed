/*
 * Copyright 2016 The University of Oklahoma.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mbed.h"
#include "Servo.h"
#include "armmessage.h"
#include "mbedchannel.h"
#include "enums.h"
#include "constants.h"
#include "util.h"

#include <climits>

//DO NOT CHANGE THESE THE PROGRAM WILL NOT WORK CORRECTLY
#define PI 3.1415926
#define FOREARM_LENGTH 300
#define BICEP_LENGTH 275
/*#define FOREARM_LENGTH 339
#define BICEP_LENGTH 266
#define BUCKET_HEIGHT 146
#define BUCKET_DEPTH 76
#define BUCKET_OFFSET 19
#define SHOULDER_OFFSET 66*/

/*                          BUCKET_OFFSET
                            o   | 
            SPRING_HEIGHT - |\  |
                            | o--O Wrist
                           /--|   \
          BUCKET_HEIGHT - /   |    \
                          \   |     \ - FOREARM_LENGTH
                           \__|      \  
                            |         \
                            |          \
                      BUCKET_DEPTH      O Elbow
                                       /
                                      /
                                     /
                                    / - BICEP_LENGTH
                                   / 
                                  /
                                 O Shoulder
                                 | - SHOULDER_OFFSET
                                -O-
                                Yaw
*/

// 0.46 BOTTOM FOR SHOULDER
// 0.21 UP FOR SHOULDER
// 0.05 BACK FOR SHOULDER

// Elbow low 0.05
// Elbow high 0.3

//values from old arm
/*#define SHOULDER_MIN 0.05
#define SHOULDER_MAX 0.55
#define ELBOW_MIN 0.1
#define ELBOW_MAX 0.60
#define WRIST_MIN 0
#define WRIST_MAX 0.4
#define BUCKET_MIN 0
#define BUCKET_MAX 0.56
#define YAW_MIN 0
#define YAW_MAX 1*/

/************************************
 * These are the limits for the arm *
 ************************************/

#define DUMP_SHOULDER 0.3700
#define DUMP_ELBOW 0.9070
#define DUMP_YAW 0.1020

#define HOME_SHOULDER 0.4556
#define HOME_YAW 0.4882
#define HOME_ELBOW 1.0000
#define HOME_WRIST 0.6022
#define HOME_BUCKET 0.3066

#define EXTENDED_ELBOW 0.5496
#define EXTENDED_SHOULDER 0.0000
#define UP_WRIST 0.1420
#define DOWN_WRIST 1.0000
#define YAW_FULL_RIGHT 0.0000
#define YAW_FULL_LEFT 1.0000

#define BUCKET_OPEN 0.8000
#define BUCKET_CLOSE 0.2000

#define MIN_YAW YAW_FULL_RIGHT
#define MAX_YAW YAW_FULL_LEFT
#define MIN_BUCKET BUCKET_CLOSE
#define MAX_BUCKET BUCKET_OPEN
#define MIN_WRIST UP_WRIST
#define MAX_WRIST DOWN_WRIST
#define MIN_ELBOW EXTENDED_ELBOW
#define MAX_ELBOW HOME_ELBOW
#define MIN_SHOULDER EXTENDED_SHOULDER
#define MAX_SHOULDER HOME_SHOULDER

#define CRASH_ON_CAGE_SHOULDER 0.2946
#define CRASH_ON_CAGE_YAW_MIN 0.2260
#define CRASH_ON_CAGE_YAW_MAX 0.8068
#define CRASH_ON_FRAME_ELBOW 0.9580

using namespace Soro;

Servo *_yawServo = NULL;
Servo *_shoulderServo = NULL;
Servo *_elbowServo = NULL;
Servo *_wristServo = NULL;
Servo *_bucketServo = NULL;

// default to arm OFF
Servo _powerToggle(p26);

float _yawRangeRatio;
float _shoulderRangeRatio;
float _elbowRangeRatio;
float _wristRangeRatio;
float _bucketRangeRatio;

int _x;
int _y;
float _t;

bool _stowed = false;

bool floatBetween(float value, float range1, float range2) {
    if (range1 > range2) {
        return (value > range2) & (value < range1);
    }
    else {
        return (value > range1) & (value < range2);
    }
}

void clampFloat(float& value, float min, float max) {
    if (value < min) value = min;
    else if (value > max) value = max;
}

/* ONLY use this function to alter the position of any servo, except
 * possibly in a predefined movement sequence where you are very careful
 */
void setPositions(float yaw, float shoulder, float elbow, float wrist, float bucket) {
    clampFloat(yaw, MIN_YAW, MAX_YAW);
    clampFloat(shoulder, MIN_SHOULDER, MAX_SHOULDER);
    clampFloat(elbow, MIN_ELBOW, MAX_ELBOW);
    clampFloat(wrist, MIN_WRIST, MAX_WRIST);
    clampFloat(bucket, MIN_BUCKET, MAX_BUCKET);
    
    // check for arm crashing on cage
    if (floatBetween(yaw, CRASH_ON_CAGE_YAW_MIN, CRASH_ON_CAGE_YAW_MAX)) {
        clampFloat(shoulder, EXTENDED_SHOULDER, CRASH_ON_CAGE_SHOULDER);
        clampFloat(elbow, EXTENDED_ELBOW, CRASH_ON_FRAME_ELBOW);
    }
    
    *_yawServo = yaw;
    *_shoulderServo = shoulder;
    *_elbowServo = elbow;
    *_wristServo = wrist;
    *_bucketServo = bucket;
}

/*void setElbowAngle(int angle){
    angle -= 11;
    float res = (0.1) + (((float) angle) / 360.0f);
    setElbow(res);
}

void setShoulderAngle(int angle){
    angle -= 10;
    float res = (0.46) - (((float) angle) / 360.0f);
    setShoulder(res);
}

void setWristAngle(int angle){
    angle -= 45;
    float res = ((float) angle) / 180;
    setWrist(res);
}

inline float toDegrees(float theta){
    return ((theta * 180.0f) / PI);
}

inline float signum(float x){
    return x < 0 ? -1 : 1;
}

void calcAngles(int x, int y, float t){
    float lengthOne = BICEP_LENGTH;
    float lengthTwo = FOREARM_LENGTH;

    float lengthSq = (x*x) + (y*y);
    float length = sqrt(lengthSq);

    float phiOne = signum(y) * acos(x / length);
    float phiTwo = acos( ((length * length) + (lengthOne * lengthOne) - (lengthTwo * lengthTwo)) / (2 * length * lengthOne) );

    float thetaOne = phiOne + phiTwo;

    float thetaTwo = acos( ((lengthOne * lengthOne) + (lengthTwo * lengthTwo) - (lengthSq)) / (2 * lengthOne * lengthTwo) );



    float wristAngle = (PI - phiTwo - thetaTwo) + ((PI / 2) - phiOne) + t;
    //pc.printf("Wrist angle: %d\r\n", (int) toDegrees(wristAngle));
    //pc.printf("Setting shoulder to: %d\r\n", (int) toDegrees(thetaOne) );
    //pc.printf("Setting elbow to: %d\r\n", (int) toDegrees(thetaTwo) );
    setShoulderAngle((int) toDegrees(thetaOne));
    setElbowAngle((int) toDegrees(thetaTwo));
    setWristAngle((int) toDegrees(wristAngle));
}

float length(int x, int y){
    return sqrt(((float) (x * x)) + ((float) (y * y)));
}

int newY(int newx, int newy){
    int ret = newy;
    if(length(newx, newy) > BICEP_LENGTH + FOREARM_LENGTH){
        //pc.printf("too big %d > %d\r\n", (int) length(newx, newy), (int) BICEP_LENGTH + FOREARM_LENGTH);
        float ang = atan2((float)newy,(float)newx);
        ret = (BICEP_LENGTH + FOREARM_LENGTH - 10) * sin(ang);
    }

    if(ret < -200){
        ret = -200;
    }

    return ret;
}

int newX(int newx, int newy){
    int ret = newx;
    if(length(newx, newy) >= BICEP_LENGTH + FOREARM_LENGTH){
        //pc.printf("too big %d > %d\r\n", (int) length(newx, newy), (int) BICEP_LENGTH + FOREARM_LENGTH);
        float ang = atan2((float)newy,(float)newx);
        ret = (BICEP_LENGTH + FOREARM_LENGTH - 10) * cos(ang);
    }

    if(ret < 30){
        ret = 30;
    }

    return ret;
}*/

/* Sets the arm in the stow position.
 * If 'safe' it will wait until the yaw is positioned before
 * moving any other servos. Otherwise, all servos will be moved at once.
 */
void stow(bool knownPosition) {
    float wait1, wait2;
    if (knownPosition && _yawServo && _shoulderServo) {
        wait1 = abs(*_shoulderServo - CRASH_ON_CAGE_SHOULDER) * 3 + 1;
        wait2 = abs(*_yawServo - HOME_YAW) * 6 + 1;
    }
    else {
        wait1 = 2;
        wait2 = 2;
    }
    //make sure yaw doesn't crash into cage because shoulder is too low
    if (!_shoulderServo) {
        _shoulderServo = new Servo(p22);
    }
    if (!_elbowServo) {
        _elbowServo = new Servo(p21);
    }
    *_shoulderServo = CRASH_ON_CAGE_SHOULDER;
    *_elbowServo = EXTENDED_ELBOW;
    wait(wait1);
    //position yaw and wait to make sure it gets there
    if (!_yawServo) {
        _yawServo = new Servo(p23);
    }
    *_yawServo = HOME_YAW;
    wait(wait2);
    //set shoulder home
    *_shoulderServo = HOME_SHOULDER;
    //set elbow home
    *_elbowServo = HOME_ELBOW;
    if (!_wristServo) {
        _wristServo = new Servo(p24);
    }
    if (!_bucketServo) {
        _bucketServo = new Servo(p25);
    }
    //set wrist
    *_wristServo = HOME_WRIST;
    //set bucket
    *_bucketServo = HOME_BUCKET;
}

/* Listener which receives the ethernet's disconnected
 * event (which triggers a reset). The arm must be stowed
 * before this happens otherwise it may stow improperly
 * when the mbed turns back on.
 */
void preResetListener() {
    stow(true);
}

int main() {
   
    //used to calculate positions in master/slave control
    _yawRangeRatio = (MAX_YAW - MIN_YAW) / (float)USHRT_MAX;
    _shoulderRangeRatio = (MAX_SHOULDER - MIN_SHOULDER) / (float)USHRT_MAX;
    _elbowRangeRatio = (MAX_ELBOW - MIN_ELBOW) / (float)USHRT_MAX;
    _wristRangeRatio = (MAX_WRIST - MIN_WRIST) / (float)USHRT_MAX;
    _bucketRangeRatio = (MAX_BUCKET - MIN_BUCKET) / (float)USHRT_MAX;
    
    MbedChannel ethernet(MBED_ID_ARM, NETWORK_ROVER_ARM_MBED_PORT);   
    ethernet.setResetListener(&preResetListener);
    ethernet.setTimeout(500);
    char buffer[50];
    
    _powerToggle = 1.0;
    stow(false);
    *_shoulderServo = CRASH_ON_CAGE_SHOULDER;
    wait(1);
    
    //Stow the arm. This will end very bad if the arm is not
    //alrady close to stow position, but we have no choice.
    
    while(1) {
        int len = ethernet.read(&buffer[0], 50);
        if (len != -1) {
            unsigned int header = (unsigned int)reinterpret_cast<unsigned char&>(buffer[0]);
            MbedMessageType messageType = reinterpret_cast<MbedMessageType&>(header);
            switch (messageType) {
             /*case MbedMessage_ArmGamepad: ///////////////////////////////////////////
                //TODO - this is very rough. Needs cleaning up and more adding wrist, bucket functionality.
                _x -= (int)(ArmMessage::getGamepadX(buffer) * 8);
                _y -= (int)(ArmMessage::getGamepadY(buffer) * 8);
                setYaw(_yawServo - ((float)ArmMessage::getGamepadYaw(buffer) * 0.008));
                if (ArmMessage::getBucketOpen(buffer)) {
                    setBucket(BUCKET_OPEN);
                }
                else if (ArmMessage::getBucketClose(buffer)) {
                    setBucket(BUCKET_CLOSED);
                }
                if ((_t < 1) & (_t > -1)) {
                    _t = _t - ((float)ArmMessage::getGamepadWrist(buffer) * 0.008);    
                }
                if (ArmMessage::getStow(buffer)) {
                    stow(true);
                }
                else {
                    _x = newX(_x,_y);
                    _y = newY(_x,_y);
                    calcAngles(_x, _y, _t);
                }
                break;*/
            case MbedMessage_ArmMaster: //////////////////////////////////////////
                if (ArmMessage::getStow(buffer)) {
                    if (!_stowed) {
                        stow(true);
                        _stowed = true;
                        wait(3);
                        _powerToggle = 0.0;
                    }
                }
                else if (_stowed) {
                    _powerToggle = 1.0;
                    _stowed = false;
                    *_shoulderServo = CRASH_ON_CAGE_SHOULDER;
                    *_yawServo = HOME_YAW;
                    *_elbowServo = HOME_ELBOW;
                    *_wristServo = HOME_WRIST;
                    wait(1);
                }
                else {
                    float bucket = *_bucketServo;
                    if (ArmMessage::getBucketOpen(buffer)) {
                        bucket = BUCKET_OPEN;
                    }
                    else if (ArmMessage::getBucketClose(buffer)) {
                        bucket = BUCKET_CLOSE;
                    }
                    if (ArmMessage::getDump(buffer)) {
                        setPositions(DUMP_YAW,
                                DUMP_SHOULDER,
                                DUMP_ELBOW,
                                ArmMessage::getMasterWrist(buffer) * _wristRangeRatio + MIN_WRIST,
                                bucket);
                    }
                    else {
                        setPositions(ArmMessage::getMasterYaw(buffer) * _yawRangeRatio + MIN_YAW,
                                ArmMessage::getMasterShoulder(buffer) * _shoulderRangeRatio + MIN_SHOULDER,
                                ArmMessage::getMasterElbow(buffer) * _elbowRangeRatio + MIN_ELBOW,
                                ArmMessage::getMasterWrist(buffer) * _wristRangeRatio + MIN_WRIST,
                                bucket);
                    } 
                }
                break;
            }
        }
    }
}
