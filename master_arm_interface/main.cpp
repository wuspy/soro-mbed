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
#include "rtos.h"

#include "armmessage.h"
#include "mbedchannel.h"

#include <climits>

#define READ_INTERVAL 50

using namespace Soro;
 
AnalogIn yaw(p15);
AnalogIn shoulder(p16);
AnalogIn elbow(p17);
AnalogIn wrist(p18);

InterruptIn bucketSwitch(p5);
InterruptIn deploySwitch(p6);
InterruptIn dumpSwitch(p7);
InterruptIn onSwitch(p8);

PwmOut statusLed(p26);

MbedChannel *ethernet;

enum State {
    ConnectingState,
    OffState,
    StowState,
    OnState,
    IllegalState
};

State currentState;
char buffer[50];

void ledLoop(void const *args) {
    while (1) {
        switch (currentState) {
        case ConnectingState:
            for (int i = 0; i < 5; i++) {
                statusLed = 1;
                wait(0.05);
                statusLed = 0;
                wait(0.05);
            }
            wait(3);
            break;
        case OffState:
            statusLed = 0;
            break;
        case StowState:
            for(float p = 0.0f; p <= 1.0f; p += 0.01f) {
                statusLed = p;
                wait(0.01);
            }
            if (currentState != StowState) continue;
            for(float p = 1.0f; p >= 0.0f; p -= 0.01f) {
                statusLed = p;
                wait(0.01);
            }
            break;
        case OnState:
            statusLed = 1;
            break;
        case IllegalState:
            statusLed = 1;
            wait(0.1);
            statusLed = 0;
            wait(0.1);
            break;
        }
    }
}

void readAndSend(bool overrideDeploySwitch, bool overrideDeployValue) {
    unsigned short shoulderVal, yawVal, elbowVal, wristVal;
    yawVal = yaw.read() * USHRT_MAX;
    shoulderVal = shoulder.read() * USHRT_MAX;
    elbowVal = elbow.read() * USHRT_MAX;
    wristVal = wrist.read() * USHRT_MAX;
    ArmMessage::setMasterArmData(&buffer[0], 
            yawVal, 
            shoulderVal, 
            elbowVal, 
            wristVal, 
            bucketSwitch, 
            (overrideDeploySwitch ? !overrideDeployValue : !deploySwitch), 
            dumpSwitch);
    ethernet->sendMessage(&buffer[0], ArmMessage::RequiredSize_Master);
}

int main() {
    currentState = ConnectingState;
    Thread ledThread(ledLoop);
    
    ethernet = new MbedChannel(MBED_ID_MASTER_ARM, NETWORK_MC_MASTER_ARM_PORT);

    while(1) {
        if (onSwitch) {
            if (!deploySwitch) {
                if (dumpSwitch) {
                    // stowed while the dump switch is on, do not
                    // let the arm deploy until dump switch is turned off
                    currentState = IllegalState;
                    while (dumpSwitch & onSwitch) { 
                        readAndSend(true, false);
                        wait_ms(READ_INTERVAL);
                    }
                    if (!onSwitch) continue;
                }
                else {
                    currentState = StowState;
                }
            }
            else {
                currentState = OnState;
            }
            readAndSend(false, false);
            wait_ms(READ_INTERVAL);
        }
        else {
            currentState = OffState;
        }
    }
}
