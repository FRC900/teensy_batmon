// !!! Do not use serial monitor while this is running or ROS will not be able to communicate with the teensy !!!

// on ros side run
// rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

// Libraries
#include <TimeLib.h>

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FlexCAN_T4.h>
#include "sfp200teensy.h"
// ROS Libs
#include <ros.h>
#include <std_msgs/String.h>
// custom power message type
#include "Power.h"
#include <tf2_msgs/TFMessage.h>
// Allocate PSRAM
//  If PSRAM has been added, extmem_malloc() may be used to allocate this memory,
//  started immediately after the EXTMEM variables.
EXTMEM char bigBuffer[1000000];

// LED
int ledpin = 13;  // Pin on the onboard LED

// Declare CAN interface
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> myCan;

// And the battery monitor to talk to
const uint32_t battery_monitor = 0x0A100201;

// Declare ROS Stuff
ros::NodeHandle nh;  // Node handler
rosserial_msgs::Power power_msg;
ros::Publisher power_info("power_info", &power_msg);  // The ROS publisher
tf2_msgs::TFMessage time_msg;
double currenttime = 0;
void tf_callback(const tf2_msgs::TFMessage &msg) {
  if (msg.transforms_length >= 1) {
    currenttime = msg.transforms[0].header.stamp.toSec();
  }
}
ros::Subscriber<tf2_msgs::TFMessage> time_sub("/tf", &tf_callback);

String partName, version, serialString;
double voltage1, voltage2, voltage3, current, coulombs, temp;
File dataFile;
int delta;
bool fileOpen = false;
elapsedMillis sinceRead;

void setup() {
  // check if there was a crash teensy
  if (CrashReport) {
    Serial.println("Crash Report:");
    Serial.println(CrashReport);
  }
  
  // Setup Serial
  Serial.begin(9600);

  // WARNING: Serial communication is borked on macOS, so if you're using a mac, comment out this line
  // Otherwise it'll keep trying to setup serial infinitely and infinitely fail
  while(!Serial);

  // Show that the Teensy is online
  Serial.println("Begin Teensy Startup Process");

  // Set the pinmode for the LED
  pinMode(ledpin, OUTPUT);

  // Enable the SD Card or print an error
  if (!SD.begin(BUILTIN_SDCARD)){
    Serial.println ("CHECK SD CARD");
  }
  else {
    Serial.println ("SD CARD WORKS!");
  }

  // Enable and Configure CAN
  Serial.println("Configuring CAN");
  myCan.begin();
  myCan.setBaudRate(500000);

  // Bring up ROS
  nh.initNode();
  nh.advertise(power_info);
  // Start the SFP lib & tell it what device to talk to
  set_target_device(battery_monitor);

  // Reset Coulomb Count
  resetCoulombCount();
}
void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printCharStr(char *str) {
  int len = strlen(str);
  for (int i = 0; i < len; i++) {
    Serial.print(str[i]);
  }
  Serial.println();
}

// wooo! so fun
char* str2chr(String str) {
  char *c = const_cast<char*>(str.c_str());
  return c;
}

void loop() {
  /*
  RESET Counts - assume full battery
  Verify ROS Coms?
  //0-Blink an LED
  //1-Read Battery Barcode
  2-Read Battery Info from CAN
  3-Publish Battery Label and Coulomb Count
  4-Log to SD Card
  5-Log to Serial
  */

  // The Heartbeat (LED blinking)
  //digitalWrite(ledpin, HIGH);
  
  //Serial.println(oldtime);
  
  // Presumably the name version and string will not change during runtime
  // Saves 22ms per loop to not read it every time
  
  if (strlen(str2chr(partName)) == 0) {
    partName = getPartName();
    // REMOVE!
    partName = "Mypartname";
  
    version = getVersion();
    version = "myversion";
    serialString = getSerial();
    //Serial.println("Ran getPartName, getVersion, getSerial");
  }

  if (sinceRead >= 6) {
  // These functions have 14ms delay total so delay is 20 - 14
  voltage1 = getVoltage(1);
  voltage2 = getVoltage(2);
  voltage3 = getVoltage(3);
  current = getCurrent();
  coulombs = getCoulombCount();
  temp = getTemperature();
  delta = sinceRead;
  sinceRead = 0; // reset timer

  //Serial.println(delta);

  // Doesn't work, strings are hard
  power_msg.header.stamp.fromSec(currenttime);
  power_msg.partName = str2chr(partName);
  power_msg.version = str2chr(version);
  power_msg.serialString = str2chr(serialString);
  // Does work
  power_msg.voltage1 = voltage1;
  power_msg.voltage2 = voltage2;
  power_msg.voltage3 = voltage3;
  power_msg.current = current;
  power_msg.coulombs = coulombs;
  power_msg.temp = temp;

  power_info.publish(&power_msg);
  char buffer[256];
  sprintf(buffer, "power_%f.csv", currenttime);
  // would be nice to get the time from ros, but all the header are relative to ros start time not 1970 
  dataFile = SD.open(buffer, FILE_WRITE);
  if (fileOpen == false) {
    dataFile.println("partName,version,serialString,voltage1,voltage2,voltage3,current,coulombs,temp,delta,rostimestamp");
    fileOpen = true;
  }

  // sets time based on seconds from epoch
  //time_t mytime = 1664667679;
  //setTime(mytime);
  //digitalClockDisplay();  
  
  // open the file.
  

  // if the file is available, write to it:
  if (dataFile) {
    // write power msg to file as a csv in one line of code
    dataFile.println(String(partName) + "," + String(version) + "," + String(serialString) + "," + String(voltage1) + "," + String(voltage2) + "," + String(voltage3) + "," + String(current) + "," + String(coulombs) + "," + String(temp) + "," + String(delta) + "," + String(currenttime));
    dataFile.close();
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
  
  // The Heartbeat
  //digitalWrite(ledpin, LOW);
  
  /*
  Serial.println(partName);
  Serial.println(version);
  Serial.println(serialString);
  Serial.println(getVoltage(1));
  Serial.println(getVoltage(2));
  Serial.println(getVoltage(3));
  Serial.println(getCurrent());
  Serial.println(getCoulombCount());
  Serial.println(getTemperature());
  Serial.println("Time");
  Serial.println(currenttime);
  delay(200);
  */
  // ROS Stuff
  nh.spinOnce();
  //Serial.println(millis() - oldtime);

  } // end if sinceRead >= 20
}