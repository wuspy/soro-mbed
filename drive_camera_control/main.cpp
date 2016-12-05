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
#include "Serial.h"
#include "mbedchannel.h"
#include "gamepadutil.h"
#include "drivemessage.h"
#include "enums.h"
#include "constants.h"
#include "gimbalmessage.h"
#include "Servo.h"

#include <cstdio>

Servo Drive_LeftOuter(p21);
Servo Drive_LeftMiddle(p23);
Servo Drive_RightOuter(p22);
Servo Drive_RightMiddle(p24);
Servo Gimbal_Pitch(p25);
Servo Gimbal_Yaw(p26);

Timer _driveEthernetTimer;
Timer _driveSerialTimer;

#define GIMBAL_PITCH_HOME 0.5
#define GIMBAL_YAW_HOME 0.5
#define GIMBAL_PITCH_ARM 0.3
#define GIMBAL_YAW_LEFT 0.0
#define GIMBAL_YAW_RIGHT 1.0

using namespace Soro;


void stopDrive() {
    Drive_LeftOuter.write(0.5);
    Drive_LeftMiddle.write(0.5);
    Drive_RightOuter.write(0.5);
    Drive_RightMiddle.write(0.5);
}

void setDrive(const char* buffer) {
    float lo = DriveMessage::getLeftOuter(buffer);
    float ro = -DriveMessage::getRightOuter(buffer);
    float ml = DriveMessage::getLeftMiddle(buffer);
    float mr = -DriveMessage::getRightMiddle(buffer);
    
    Drive_LeftOuter.write(((float)lo)/2.0 + 0.5);
    Drive_RightOuter.write(((float)ro)/2.0 + 0.5);
    Drive_LeftMiddle.write(((float)ml)/2.0 + 0.5);
    Drive_RightMiddle.write(((float)mr)/2.0 + 0.5);
}

/* Listener which receives the ethernet's disconnected
 * event (which triggers a reset). The rover should stop
 * if this is the case.
 */
void preResetListener() {
    stopDrive();
}

int main() {
    Gimbal_Pitch = GIMBAL_PITCH_HOME;
    Gimbal_Yaw = GIMBAL_YAW_HOME;
    
    // TESTING CODE, drives one wheel back and forth
    /*while (1) {
        for (float i = 0.5; i < 1; i += 0.01) {
            Drive_LeftOuter.write(i);
            wait_ms(25);
        }
        for (float i = 1; i > 0; i -= 0.01) {
            Drive_LeftOuter.write(i);
            wait_ms(25);
        }
        for (float i = 0; i < 0.5; i += 0.01) {
            Drive_LeftOuter.write(i);
            wait_ms(25);
        }
    }
    
    return;*/
    
    MbedChannel ethernet(MBED_ID_DRIVE_CAMERA, NETWORK_ROVER_DRIVE_MBED_PORT);
    ethernet.setResetListener(&preResetListener);
    ethernet.setTimeout(500); // drive will stop if this timeout is reached
    
    Serial driveSerial(p13, p14);
    Serial dataSerial(p9, p10);
    Serial pc(USBTX, USBRX);
    
    driveSerial.baud(9600);
    dataSerial.baud(9600);
    
    char buffer[500];
    int bufferOffset;
    
    DigitalOut led1(LED1);
    DigitalOut led2(LED2);
    DigitalOut led3(LED3);
    DigitalOut led4(LED4);
    
    _driveEthernetTimer.start();
    
    while(1) {
        bufferOffset = 0;
        // Process any loggable data first
        while (dataSerial.readable()) {
            buffer[bufferOffset] = dataSerial.getc();
            bufferOffset++;
        }
        if (bufferOffset > 0) {
            led1 = 1;
            ethernet.sendMessage(buffer, bufferOffset);
        }
        else {
            led1 = 0;
        }
        
        // See if there is a message waiting on the drive serial port
        while (driveSerial.readable()) {
            _driveSerialTimer.start();
            _driveSerialTimer.reset();
            int c = driveSerial.getc();
            if (c != 255) continue;
            
            for (int i = 1; i < 5; i++) {
                buffer[i] = driveSerial.getc();
                if (buffer[i] > 200) {
                    buffer[i] = 100;
                }
            }
            //pc.printf("Got complete drive command: %u %u %u %u\r\n", buffer[1], buffer[2], buffer[3], buffer[4]);
            
            led2 = 1;
            led3 = 0;
            setDrive(buffer);
        }
        
        int dst = _driveSerialTimer.read_us(); //MICRO second
        if ((dst > 0) && (dst < 1000000)) { // Serial drive overrides ethernet drive for 1 second
            continue;
        }
        _driveSerialTimer.stop();
        _driveSerialTimer.reset();
        
        // No serial drive control, fallback to ethernet control
        led2 = 0;
        led3 = 1;
        //wait_ms(100);
        //continue;
        
        int len = ethernet.read(&buffer[0], 50);
        if (len == -1) {
            stopDrive();
            continue;
        }
        
        unsigned int header = (unsigned int)reinterpret_cast<unsigned char&>(buffer[0]);
        MbedMessageType messageType = reinterpret_cast<MbedMessageType&>(header);
        switch (messageType) {
        case MbedMessage_Drive:
            setDrive(buffer);            
            _driveEthernetTimer.reset();
            break;
        case MbedMessage_Gimbal:
            if (GimbalMessage::getLookHome(buffer)) {
                Gimbal_Pitch = GIMBAL_PITCH_HOME;
                Gimbal_Yaw = GIMBAL_YAW_HOME;
                break;
            }
            else if (GimbalMessage::getLookLeft(buffer)) {
                Gimbal_Pitch = GIMBAL_PITCH_HOME;
                Gimbal_Yaw = GIMBAL_YAW_LEFT;
                break;
            }
            else if (GimbalMessage::getLookRight(buffer)) {
                Gimbal_Pitch = GIMBAL_PITCH_HOME;
                Gimbal_Yaw = GIMBAL_YAW_RIGHT;
                break;
            }
            else if (GimbalMessage::getLookArm(buffer)) {
                Gimbal_Pitch = GIMBAL_PITCH_ARM;
                Gimbal_Yaw = GIMBAL_YAW_HOME;
                break;
            }
            
            Gimbal_Pitch = Gimbal_Pitch + (GimbalMessage::getPitch(buffer) * 0.01);
            Gimbal_Yaw = Gimbal_Yaw + (GimbalMessage::getYaw(buffer) * 0.01);
            break;
        default:
            break; 
        }
        if (_driveEthernetTimer.read_ms() > 500) {
            stopDrive();
        }
    }
}
