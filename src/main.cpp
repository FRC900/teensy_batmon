// Libraries
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FlexCAN_T4.h>
#include <NativeEthernet.h>
//#include <NativeEthernetServer.h>
//#include <NativeEthernetClient.h>
//#include <Ethernet.h>
//#include <QTRSensors.h>
#include "sfp200teensy.h"

// ROS Libs
//#include <ArduinoTcpHardware.h>
#define ROSSERIAL_ARDUINO_TCP  // To use the TCP version of rosserial_arduino
#include <ros.h>
#include <std_msgs/String.h>

// Allocate PSRAM
//  If PSRAM has been added, extmem_malloc() may be used to allocate this memory,
//  started immediately after the EXTMEM variables.
EXTMEM char bigBuffer[1000000];

// Variables
int ledpin = 13;  // Pin on the onboard LED
int ledState = LOW;  // ledState used to set the LED

// Declare CAN interface
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> myCan;
//static uint8_t hex[17] = "0123456789abcdef";

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

/*
// QTR Sensor Stuff:
QTRSensors qtr;
const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];
*/

void setup() {
  // Setup Serial
  delay(5000);
  Serial.begin(9600);

  // WARNING: Serial communication is borked on macOS, so if you're using a mac, comment out this line
  // Otherwise it'll keep trying to setup serial infinitely and infinitely fail
  while(!Serial) {; };
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

  // Reset Coulomb Count
  resetCoulombCount(0x0A100201);

/*
  //Check for and read the barcode on the battery:
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){3,4,5,6,7,8,9,10}, sensorCount);
  qtr.setEmitterPin(2);
  delay(5);
  qtr.read(sensorValues);//read more than once?
  //values will be 0 to 2500, 0 is max reflectance (white), 2500 is minimum reflectance (black)
  for(uint8_t i = 0; i < sensorCount; i++){
    Serial.println(sensorValues[i]);
  }
*/
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
  if(ledState == LOW){
    ledState = HIGH;
    digitalWrite(ledpin, ledState);
  }
  else{
    ledState = LOW;
    digitalWrite(ledpin, ledState);
  }
  
  // All if the power information collected
  Serial.println(getPartName(0x0A100201));
  Serial.println(getVersion(0x0A100201));
  Serial.println(getSerial(0x0A100201));
  Serial.println(getVoltage(0x0A100201, 1));
  Serial.println(getVoltage(0x0A100201, 2));
  Serial.println(getVoltage(0x0A100201, 3));
  Serial.println(getCurrent(0x0A100201));
  Serial.println(getCoulombCount(0x0A100201));
  Serial.println(getTemperature(0x0A100201));

  // The Heartbeat
  if(ledState == LOW){
    ledState = HIGH;
    digitalWrite(ledpin, ledState);
  }
  else{
    ledState = LOW;
    digitalWrite(ledpin, ledState);
  }

  // ROS Stuff
  str_msg.data = "hello world!";
  chatter.publish( &str_msg );
  nh.spinOnce();

  //delay
  delay(1000);
}