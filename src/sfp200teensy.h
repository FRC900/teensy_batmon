#ifndef SFP200TEENSY_H /* include guards */
#define SFP200TEENSY_H

// Included to avoid the missing types
#include <Arduino.h>

// Set the target device
void set_target_device(uint32_t device);

// Get the Part Name
String getPartName();

// Get the Version of the part
String getVersion();

// Get the Serial of the part
String getSerial();

// Get the current measurement, in milliAmps
double getCurrent();

// Reset the Coulomb Counterial
void resetCoulombCount();

// Get the Coulomb count
double getCoulombCount();

// Get the voltage
double getVoltage(int voltageId);

// Get the temperature of the device
double getTemperature();

#endif /* SFP200TEENSY_H */