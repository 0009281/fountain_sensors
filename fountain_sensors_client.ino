#include <Wire.h>
#include "SHTSensor.h"
#include "wifi_details.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "AsyncUDP.h"

#include <ArduinoOTA.h>
#include <HTTPUpdate.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include "EEPROM.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define CHECK_FOR_NEW_FIRMWARE_FREQUENCE_SECONDS 30 //check for the new firmware every 24 hours
#define US_ROUNDTRIP_CM 57      // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space.
#define US_ROUNDTRIP_MM 5.7      // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space.
// Ultrasonic sensor
#define trigPin  13
#define echoPin  14
#define NoBlind_UltrasonicConvert(echoTime, conversionFactor) (max((echoTime + conversionFactor / 2) / conversionFactor, (echoTime ? 1 : 0)))
// SHT85 sensor
#define SCL_pin  22
#define SDA_pin  21

#define BUTTON             0 //onboard button
#define LED_PIN            2 //onboard led
#define GARLAND_PIN        4 //external relay control pin
#define SKETCH_VERSION "1.0.33"

#define FLOOD_LEVEL 30 //; if the water level is over 30 mm then it should be then we decide that is flooding

IPAddress esp32_fountain_basement_ip;
SHTSensor sht(SHTSensor::SHT3X);
unsigned int duration, distance;
//unsigned long check_for_the_new_frimware_millis;
uint16_t distance_mm, distance_mm_average, distance_mm_to_monitor;
uint8_t mesurement_count;
bool loadEepromFailed = false, emergency_flooding = false, was_notified = false;
WiFiUDP udp_client;
AsyncUDP udp_server;
const char *  ip_crestron       = "192.168.0.5"; 
const char *  ip_protection_unit   = "192.168.88.2"; 
const int crestron_esp32_port        = 1111;    // the destination port
const int ip_protection_unit_port        = 1111;    // the destination port

const char *  asterisk_ip       = "192.168.0.7"; 
const int     asterisk_port        = 5038;    // the destination port

std::string command_to_realy_din_rail_block="Disable Fountain";
String command_to_run, tmp1;

struct SetupData {
  uint16_t distance_mm_to_monitor;
  uint8_t garland_outlet_last_state;
};

SetupData eeprom_settings;

#define EEPROM_SIZE        sizeof(eeprom_settings)


void flash_led(uint8_t number_times)
{
  for (uint8_t j=0;j<number_times; j++) {
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
  }
}


void notify_protection_module(std::string *data)
{
  if (esp32_fountain_basement_ip.toString() != "0.0.0.0") {
    Serial.print("IP address of the basement node: ");
    Serial.println(esp32_fountain_basement_ip.toString());
    udp_client.beginPacket(esp32_fountain_basement_ip, 1111);
    udp_client.printf("%s", data->c_str());
    udp_client.endPacket();
  }
}



void setup() {
  Serial.begin(115200); // Starts the serial communication
  Serial.print("Current firmware version: ");
  Serial.println(SKETCH_VERSION);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON, INPUT);
  digitalWrite(GARLAND_PIN, LOW);
  pinMode(GARLAND_PIN, OUTPUT);
  //check for EEPROM corruption
  if ((!EEPROM.begin(EEPROM_SIZE))) {
    Serial.println("Failed to initialise EEPROM"); 
    loadEepromFailed = true;
  }
  else {
    Serial.println("OK to initialise EEPROM"); 
    EEPROM.get(0, eeprom_settings);
    distance_mm_to_monitor = eeprom_settings.distance_mm_to_monitor ;
    if (isnan(distance_mm_to_monitor)) loadEepromFailed = true;
    Serial.print("Average distance for monitoring loaded from EEPROM is: ");
    Serial.println(distance_mm_to_monitor);
  }
  digitalWrite(GARLAND_PIN, eeprom_settings.garland_outlet_last_state);  
  // START WIFI INIT  
  // delete old config
  WiFi.disconnect(true);
  // Examples of different ways to register wifi events
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  Serial.println("Wait for WiFi... ");

  //start Bonjour (mDNS) responder
  if (MDNS.begin("esp32")) {
    MDNS.setInstanceName("ESP32 Fountain Sensor Board");
    Serial.println("mDNS responder started");
    MDNS.addService("ESP32_fountain_control","tcp", 1111);
    MDNS.disableArduino();
  }
  else Serial.println("Error setting up MDNS responder!");


  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100))); led_status_green_blink();
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  //wifiServer.begin();
    if(udp_server.listen(1111)) {
        Serial.print("UDP Listening on IP: ");
        Serial.println(WiFi.localIP());
        udp_server.onPacket([](AsyncUDPPacket packet) {
            Serial.print("UDP Packet Type: ");
            Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
            Serial.print(", From: ");
            Serial.print(packet.remoteIP());
            Serial.print(":");
            Serial.print(packet.remotePort());
            Serial.print(", To: ");
            Serial.print(packet.localIP());
            Serial.print(":");
            Serial.print(packet.localPort());
            Serial.print(", Length: ");
            Serial.print(packet.length());
            Serial.print(", Data: ");
            Serial.write(packet.data(), packet.length());
            Serial.println();
            //reply to the client
            //packet.printf("Got %u bytes of data: ", packet.length());
            tmp1 = "";
            for (uint8_t i=0; i<packet.length();i++) tmp1 += (char)packet.data()[i];

            if ((packet.remoteIP() == esp32_fountain_basement_ip) && (tmp1 == "ACK")) 
              flash_led(1);              
            else {
              command_to_run = tmp1;
              Serial.print("Command_to_run: ");
              Serial.println(command_to_run );
              packet.print(command_to_run);
            }
            
        });
    }


  

  Wire.begin(SDA_pin, SCL_pin);

  if (sht.init()) {
    Serial.print("SHT85 init(): success\n");
  } else {
    Serial.print("SHT85 init(): failed\n");
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x



  digitalWrite(LED_PIN, LOW);
}

void loop() {

  ArduinoOTA.handle();
  //use mDNS to acquire the basement node IP
  esp32_fountain_basement_ip = MDNS.queryHost("esp32-30aea422d0e0");
  
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = duration * 0.034 / 2;

  // Prints the distance on the Serial Monitor
  Serial.print("Distance in mm: ");
  //Serial.println(NoBlind_UltrasonicConvert(duration, US_ROUNDTRIP_CM)); // Convert uS to centimeters.);
  distance_mm = (int)(duration + 5.7 / 2)/5.7;
  Serial.println(distance_mm );
  if(deviceConnected) {
    Serial.print("Command to the basement node: ");
    Serial.println(command_to_realy_din_rail_block.c_str());
    if (distance_mm_to_monitor - distance_mm > FLOOD_LEVEL) {
      Serial.println("Emergency!!!!!!!!!!! Flooding!!!!!!"); // Convert uS to centimeters.);
      command_to_realy_din_rail_block = "Disable Fountain";
      notify_protection_module(&command_to_realy_din_rail_block);
      emergency_flooding = true;
      flash_led(3);
      WiFiClient client_asterisk;
      if (!was_notified) {
        if (!client_asterisk.connect(asterisk_ip, asterisk_port)) {
          Serial.println("Connection to Asterisk at port 5038 failed");
        } else {
          client_asterisk.setNoDelay(true); //to be sure that all data below have been sent to the Asterisk server
          client_asterisk.print("Action: login\nUsername: max0\nSecret: 56kil1234567!@\n\n");
          client_asterisk.print("Action: Originate\nChannel: SIP/000\nContext: fountain-warning\nExten: s\nPriority: 1\nCallerid: Fontan !!!\n\n");
 //         client_asterisk.print("Action: Originate\nChannel: PjSIP/100\nContext: fountain-flood\nExten: s\nPriority: 1\nCallerid: Fontan !!!\n\n");
//          client_asterisk.print("Action: Originate\nChannel: PjSIP/103\nContext: fountain-flood\nExten: s\nPriority: 1\nCallerid: Fontan !!!\n\n");
          client_asterisk.stop();
          was_notified = true;
        }
      } 
    } 
 //   else if ( (distance_mm_to_monitor - distance_mm < 25) && emergency_flooding)  { //hysteresis = 5 mm
      //emergency_flooding = false;
    //}
    else { //if (!emergency_flooding) {
      notify_protection_module(&command_to_realy_din_rail_block);
      emergency_flooding = false;
      was_notified = false;
      //flash_led(1);
    }
  
   
  }
  
  if (sht.readSample()) {
    Serial.print("Humidity: "); 
    Serial.print(sht.getHumidity(), 2);
    Serial.print("%  Temperature:  ");
    Serial.println(sht.getTemperature(), 2);
  } else {
    Serial.println("Error in readSample() from the SENSIRION SHT85 Sensor");
  }


  //Ssending status data to the Crestron system
  udp_client.beginPacket(ip_crestron,crestron_esp32_port);
  udp_client.printf("|%04d mm", distance_mm_to_monitor - distance_mm);
  udp_client.printf("|%0.1f C", sht.getTemperature());
  udp_client.printf("|%0.1f %%|", sht.getHumidity());
  udp_client.endPacket();
  

  if (command_to_run=="Enable Fountain") {
    Serial.println("Enable Fountain");
    command_to_realy_din_rail_block = "Enable Fountain";
    command_to_run="";
  }
  else if (command_to_run=="Disable Fountain") {
    Serial.println("Disable Fountain");
    command_to_realy_din_rail_block = "Disable Fountain";
    command_to_run="";
  }
  else if (command_to_run=="Enable Garland") {
    Serial.println("Enable Garland");
    digitalWrite(GARLAND_PIN, HIGH);
    if (!eeprom_settings.garland_outlet_last_state) {
      eeprom_settings.garland_outlet_last_state = 1;
      EEPROM.put(0, eeprom_settings); 
      EEPROM.commit();
    }
    command_to_run="";
  }
  else if (command_to_run=="Disable Garland") {
    Serial.println("Disable Garland");
    digitalWrite(GARLAND_PIN, LOW);
    if (eeprom_settings.garland_outlet_last_state) {      
      eeprom_settings.garland_outlet_last_state = 0;
      EEPROM.put(0, eeprom_settings); 
      EEPROM.commit();
    }
    command_to_run="";
  }


 //calculate average distance and save it to the EEPROM
  if (!digitalRead(BUTTON)) {
    if (mesurement_count<10) {
      distance_mm_average += distance_mm;
      if (mesurement_count == 9) { 
        distance_mm_to_monitor = (int)(distance_mm_average / 10);
        eeprom_settings.distance_mm_to_monitor = distance_mm_to_monitor;
        EEPROM.put(0, eeprom_settings);
        EEPROM.commit();
        delay(3000);
        flash_led(2);
        delay(1000);
        Serial.print("Average distance to monitor: ");
        Serial.println(distance_mm_to_monitor);
      }
      mesurement_count++;
    }
  }
  else {
    mesurement_count = 0;
    distance_mm_average = 0;
  }
 
  delay(1000);
}
