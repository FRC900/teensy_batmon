// on ros side run
// rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

// Libraries
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FlexCAN_T4.h>
#include "sfp200teensy.h"
// ROS Libs
#include <ros.h>
#include <std_msgs/String.h>
// custom power message type
#include "rosserial_msgs/Power.h"
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
std_msgs::String str_msg;
rosserial_msgs::Power power_msg;
ros::Publisher chatter("chatter", &str_msg);  // The ROS publisher
ros::Publisher power_info("power_info", &power_msg);  // The ROS publisher

void setup() {
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
  nh.advertise(chatter);
  nh.advertise(power_info);
  // Start the SFP lib & tell it what device to talk to
  set_target_device(battery_monitor);

  // Reset Coulomb Count
  resetCoulombCount();
}

void loop() {
  /*
  RESET Counts - assume full battery
  Verify ROS Coms?
  0-Blink an LED
  //1-Read Battery Barcode
  2-Read Battery Info from CAN
  3-Publish Battery Label and Coulomb Count
  4-Log to SD Card
  5-Log to Serial
  */

  // The Heartbeat (LED blinking)
  digitalWrite(ledpin, HIGH);
  power_msg.partName = "hello";
  power_msg.version = "fix me";
  power_msg.serialString = "hello world!";
  power_msg.voltage1 = 0.56712;
  power_msg.voltage2 = 0.9999;
  power_msg.voltage3 = 2.0;
  power_msg.current = 0.56712;
  power_msg.coulombs = 0.56712;
  power_msg.temp = 99.29;

  power_info.publish(&power_msg);
  chatter.publish( &str_msg );

  
  /*
  // open the file.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(getPartName());
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
  */

  // All if the power information collected
  
  Serial.println(getPartName());
  Serial.println(getVersion());
  Serial.println(getSerial());
  Serial.println(getVoltage(1));
  Serial.println(getVoltage(2));
  Serial.println(getVoltage(3));
  Serial.println(getCurrent());
  Serial.println(getCoulombCount());
  Serial.println(getTemperature());

  // The Heartbeat
  digitalWrite(ledpin, LOW);

  // ROS Stuff
  nh.spinOnce();

  // Delay, for the heartbeat
  delay(1000);
}