// Libraries
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FlexCAN_T4.h>
#include <NativeEthernet.h>
#include "sfp200teensy.h"

// ROS Libs
#define ROSSERIAL_ARDUINO_TCP  // To use the TCP version of rosserial_arduino
#include <ros.h>
#include <std_msgs/String.h>

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

// Ethernet MAC & IP
byte mac[] = {0xde, 0xad, 0xbe, 0xef, 0x13, 0x37};
IPAddress ip(10,9,0,20);

// Declare ROS Stuff
ros::NodeHandle nh;  // Node handler
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);  // The ROS publisher

// The ROSSerial server IP & port
IPAddress server(10,9,0,8);
const uint16_t serverPort = 11411;

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

  // Enable Ethernet:
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
  else {
    Ethernet.begin(mac, ip);
    Serial.println("");
    Serial.println("Ethernet connected");
    Serial.println("IP address: ");
    Serial.println(Ethernet.localIP());
  }

  // Bring up ROS
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertise(chatter);

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
  1-Read Battery Barcode
  2-Read Battery Info from CAN
  3-Publish Battery Label and Coulomb Count
  4-Log to SD Card
  5-Log to Serial
  */

  // The Heartbeat (LED blinking)
  digitalWrite(ledpin, HIGH);
  
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
  str_msg.data = "hello world!";
  chatter.publish( &str_msg );
  nh.spinOnce();

  // Delay, for the heartbeat
  delay(1000);
}