#ifndef SFP200TEENSY_H /* include guards */
#define SFP200TEENSY_H

// Included to avoid the missing types
#include <Arduino.h>

// Get the Part Name
String getPartName(uint32_t deviceId);

// Get the Version of the part
String getVersion(uint32_t deviceId);

// Get the Serial of the part
String getSerial(uint32_t deviceId);

// Get the current measurement, in milliAmps
double getCurrent(uint32_t deviceId);

// Reset the Coulomb Counterial
void resetCoulombCount(uint32_t deviceId);

// Get the Coulomb count
double getCoulombCount(uint32_t deviceId);

// Get the voltage
double getVoltage(uint32_t deviceId, int voltageId);

// Get the temperature of the device
double getTemperature(uint32_t deviceId);

#endif /* SFP200TEENSY_H */