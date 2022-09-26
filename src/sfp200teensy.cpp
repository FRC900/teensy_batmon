// Libraries
#include "sfp200teensy.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>

// CAN Constructor
// CAN: Protocol for microcontrollers to communicate w/o needing a host computer (https://en.wikipedia.org/wiki/CAN_bus)
// CAN stands for Controller Area Network
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> candev1;

// Target device
uint32_t deviceId;

// Set the target device
void set_target_device(uint32_t device) {
    deviceId = device;
}

// Send a message with CAN
void sendCANMessage(uint32_t addressId) {
    CAN_message_t canmsg;  // The message
    canmsg.id = deviceId;
    canmsg.flags.extended = 1;
    canmsg.len = 8;
    canmsg.buf[0] = addressId;
    candev1.write(canmsg);
}

// Get the device's name
String getPartName() {
    CAN_message_t partNameMsg;
    String partName = "";

    sendCANMessage(0x01);
    delay(2);

    if(candev1.read(partNameMsg) && partNameMsg.buf[0] == 0x01){
        for(uint8_t i=1; i<partNameMsg.len; i++){
            partName.concat((char)partNameMsg.buf[i]);
        }
    }
    sendCANMessage(0x02);
    delay(2);
    if(candev1.read(partNameMsg) && partNameMsg.buf[0] == 0x02){
        for(uint8_t i=1; i<partNameMsg.len; i++){
            partName.concat((char)partNameMsg.buf[i]);
        }
    }
    sendCANMessage(0x03);
    delay(2);
    if(candev1.read(partNameMsg) && partNameMsg.buf[0] == 0x03){
        for(uint8_t i=1; i<partNameMsg.len; i++){
            partName.concat((char)partNameMsg.buf[i]);
        }
    }
    sendCANMessage(0x04);
    delay(2);
    if(candev1.read(partNameMsg) && partNameMsg.buf[0] == 0x04){
        for(uint8_t i=1; i<partNameMsg.len; i++){
            partName.concat((char)partNameMsg.buf[i]);
        }
    }

    return partName;
}

// Get the device's version info
String getVersion() {
    CAN_message_t versionMsg;
    String version = "";

    sendCANMessage(0x05);
    delay(2);

    if(candev1.read(versionMsg) && versionMsg.buf[0] == 0x05){
        for(uint8_t i=1; i<versionMsg.len; i++){
            version.concat((char)versionMsg.buf[i]);
        }
    }
    sendCANMessage(0x06);
    delay(2);
    if(candev1.read(versionMsg) && versionMsg.buf[0] == 0x06){
        for(uint8_t i=1; i<versionMsg.len; i++){
            version.concat((char)versionMsg.buf[i]);
        }
    }
    sendCANMessage(0x07);
    delay(2);
    if(candev1.read(versionMsg) && versionMsg.buf[0] == 0x07){
        for(uint8_t i=1; i<versionMsg.len; i++){
            version.concat((char)versionMsg.buf[i]);
        }
    }

    return version;
}

String getSerial() {
    CAN_message_t serialMsg;
    String serialString = "";

    sendCANMessage(0x08);
    delay(2);

    if(candev1.read(serialMsg) && serialMsg.buf[0] == 0x08){
        for(uint8_t i=1; i<serialMsg.len; i++){
            serialString.concat(serialMsg.buf[i]);
        }
    }
    sendCANMessage(0x09);
    delay(2);
    if(candev1.read(serialMsg) && serialMsg.buf[0] == 0x09){
        for(uint8_t i=1; i<serialMsg.len; i++){
            serialString.concat(serialMsg.buf[i]);
        }
    }
    sendCANMessage(0x0A);
    delay(2);
    if(candev1.read(serialMsg) && serialMsg.buf[0] == 0x0A){
        for(uint8_t i=1; i<serialMsg.len; i++){
            serialString.concat(serialMsg.buf[i]);
        }
    }
    sendCANMessage(0x0B);
    delay(2);
    if(candev1.read(serialMsg) && serialMsg.buf[0] == 0x0B){
        for(uint8_t i=1; i<serialMsg.len; i++){
            serialString.concat(serialMsg.buf[i]);
        }
    }
    //this will return a giant numerical string that makes no sense 
    //but it's what it is supposed to do
    //Hex: BB AB 9E 2F, 68 5F 26 2E, 51 87 71 FC, 49 79 B8 6E
    return serialString;
}

double getCurrent() {
    CAN_message_t currentMsg;
    double current = 0;
    unsigned int currentRaw = 0;
    
    sendCANMessage(0x20);
    delay(2);

    if(candev1.read(currentMsg) && currentMsg.buf[0] == 0x20){
        for(uint8_t i=1; i<currentMsg.len; i++){
                currentRaw |= currentMsg.buf[i] << (24-(8*(i-1)));
        }
        current = (int)(currentRaw)/1000000.0f;
    }

    return current;
}

void resetCoulombCount() {
    //Get count and reset
    CAN_message_t resetMsg;
    unsigned int coulombCountLowRaw = 0;

    sendCANMessage(0x42);
    delay(2);

    if(candev1.read(resetMsg) && resetMsg.buf[0] == 0x42){
        for(uint8_t i=1; i<resetMsg.len; i++){
            coulombCountLowRaw |= resetMsg.buf[i] << (24-(8*(i-1)));
        }
    }

    //Printing for debug. Could be used for retrieving last count before reboot.
    Serial.println(coulombCountLowRaw);
    //This delay makes sure that we don't end up with weird numbers and an overflow case
    delay(2000);
}

double getCoulombCount() {
    CAN_message_t coulombMsg;
    unsigned int coulombCountLowRaw = 0;
    unsigned int coulombCountHighRaw = 0;
    double coulombs = 0;

    sendCANMessage(0x40);
    delay(2);

    if(candev1.read(coulombMsg) && coulombMsg.buf[0] == 0x40){
        for(uint8_t i=1; i<coulombMsg.len; i++){
            coulombCountLowRaw |= coulombMsg.buf[i] << (24-(8*(i-1)));
        }
    }
    sendCANMessage(0x41);
    delay(2);
    if(candev1.read(coulombMsg) && coulombMsg.buf[0] == 0x41){
        for(uint8_t i=1; i<coulombMsg.len; i++){
            coulombCountHighRaw |= coulombMsg.buf[i] << (24-(8*(i-1)));
        }
    }

    coulombs = ((uint64_t)coulombCountHighRaw << 32 | coulombCountLowRaw)/1000000.0f;
    return coulombs;
}

double getVoltage(int voltageId) {
    CAN_message_t voltageMsg;
    double voltage = 0;
    unsigned int voltageRaw = 0;

    // There are 3 different voltages
    if (voltageId == 1){
        sendCANMessage(0x60);
        delay(2);
        if(candev1.read(voltageMsg) && voltageMsg.buf[0] == 0x60){
            for(uint8_t i=1; i<voltageMsg.len; i++){
                voltageRaw |= voltageMsg.buf[i] << (24-(8*(i-1)));
            }
        }
        voltage = (int)(voltageRaw)/1000000.0f;
    }
    else if (voltageId == 2){
        sendCANMessage(0x61);
        delay(2);
        if(candev1.read(voltageMsg) && voltageMsg.buf[0] == 0x61){
            for(uint8_t i=1; i<voltageMsg.len; i++){
                voltageRaw |= voltageMsg.buf[i] << (24-(8*(i-1)));
            }
        }
        voltage = (int)(voltageRaw)/1000000.0f;
    }
    else if (voltageId == 3){
        sendCANMessage(0x62);
        delay(2);
        if(candev1.read(voltageMsg) && voltageMsg.buf[0] == 0x62){
            for(uint8_t i=1; i<voltageMsg.len; i++){
                voltageRaw |= voltageMsg.buf[i] << (24-(8*(i-1)));
            }
        }
        voltage = (int)(voltageRaw)/1000000.0f;
    }
    else {
        Serial.println("Voltage ID is invalid");
    }
    return voltage;
}

double getTemperature() {
    CAN_message_t temperatureMsg;
    double temperature = 0;
    unsigned int temperatureRaw = 0;

    sendCANMessage(0x80);
    delay(2);

    if(candev1.read(temperatureMsg) && temperatureMsg.buf[0] == 0x80){
        for(uint8_t i=1; i<temperatureMsg.len; i++){
            temperatureRaw |= temperatureMsg.buf[i] << (24-(8*(i-1)));
        }
        temperature = (int)(temperatureRaw)/1000.0f;
    }
    
    return temperature;
}