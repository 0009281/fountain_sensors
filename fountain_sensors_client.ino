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
#define EEPROM_SIZE        2
#define SKETCH_VERSION "1.0.33"

SHTSensor sht(SHTSensor::SHT3X);

unsigned int duration, distance;
unsigned long check_for_the_new_frimware_millis;
uint16_t distance_mm, distance_mm_average, distance_mm_to_monitor;
uint8_t i;
bool loadEepromFailed = false, emergency_flooding = false;
WiFiUDP udp_client;
AsyncUDP udp_server;
const char *  ip_crestron       = "192.168.88.2"; 
const int crestron_esp32_port        = 1111;    // the destination port

const char *  asterisk_ip       = "192.168.88.2"; 
const int     asterisk_port        = 5038;    // the destination port


BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
uint32_t value = 0;
//std::string value1="                                              ";
std::string command_to_realy_din_rail_block="Disable Fountain";
String command_to_run;


//WiFiServer wifiServer(1111);

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      digitalWrite(LED_PIN, LOW);
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      digitalWrite(LED_PIN, HIGH);
      deviceConnected = false;
    }
};

void flash_led(uint8_t number_times)
{
  for (uint8_t j=0;j<number_times; j++) {
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
  }
}

/*
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));      
      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);
          command_to_realy_din_rail_block=value;
        Serial.println();
        Serial.println("*********");
      }
    }
};
*/


void setup() {
  Serial.begin(115200); // Starts the serial communication
  Serial.print("Current firmware version: ");
  Serial.println(SKETCH_VERSION);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(GARLAND_PIN, OUTPUT);
  digitalWrite(GARLAND_PIN, LOW);
  //check for EEPROM corruption
  if ((!EEPROM.begin(EEPROM_SIZE))) {
    Serial.println("Failed to initialise EEPROM"); 
    loadEepromFailed = true;
  }
  else {
    Serial.println("OK to initialise EEPROM"); 
    EEPROM.get(0, distance_mm_to_monitor);
    if (isnan(distance_mm_to_monitor)) loadEepromFailed = true;
    Serial.print("Average distance for monitoring loaded from EEPROM is: ");
    Serial.println(distance_mm_to_monitor);
  }

  
  // START WIFI INIT  
  // delete old config
  WiFi.disconnect(true);
  // Examples of different ways to register wifi events
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  Serial.println("Wait for WiFi... ");
  //start Bonjour (mDNS) responder
  if (MDNS.begin("esp32_fountain")) {
    MDNS.setInstanceName("ESP32 Fountain Sensor Board");
    Serial.println("mDNS responder started");
    MDNS.addService("Control","tcp", 1111);
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
            packet.printf("Got %u bytes of data", packet.length());
            //for (uint8_t i=0; i<packet.length();i++) { command_to_run += (char *)packet.data();
              command_to_run = (char *)packet.data();
              Serial.print("Command_to_run: ");
              Serial.println(command_to_run );
            //}
        });
    }


  

  Wire.begin(SDA_pin, SCL_pin);

  if (sht.init()) {
    Serial.print("SHT85 init(): success\n");
  } else {
    Serial.print("SHT85 init(): failed\n");
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x

  

 // Create the BLE Device
  BLEDevice::init("ESP32");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

//  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("Disable Fountain");


  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
  digitalWrite(LED_PIN, HIGH);


}

void loop() {

  ArduinoOTA.handle();

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

    if (distance_mm_to_monitor - distance_mm > 30) {
      Serial.println("Emergency!!!!!!!!!!! Flooding!!!!!!"); // Convert uS to centimeters.);
      command_to_realy_din_rail_block = "Disable Fountain";
      pCharacteristic->setValue(command_to_realy_din_rail_block);
      pCharacteristic->notify();
      emergency_flooding = true;
      flash_led(3);
 
      WiFiClient client_asterisk;
      if (!client_asterisk.connect(asterisk_ip, asterisk_port)) {
        Serial.println("Connection to Asterisk at port 5038 failed.");
      } else {
        client_asterisk.print("Action: Login\nUsername: admin\nSecret: 56kil1234567!@\n\n");
        client_asterisk.print("Action: Originate\nChannel: PjSIP/100\nContext: fountain-flood\nExten: s\nPriority: 1\nCallerid: Fontan\n\n");
        client_asterisk.print("Action: Originate\nChannel: PjSIP/103\nContext: fountain-flood\nExten: s\nPriority: 1\nCallerid: Fontan\n\n");
        client_asterisk.stop();
      }

 
    } else {
      pCharacteristic->setValue(command_to_realy_din_rail_block);
      pCharacteristic->notify();
      emergency_flooding = false;
      flash_led(1);
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


  //Ssending status data to the Crsteron system
  udp_client.beginPacket(ip_crestron,crestron_esp32_port);
  udp_client.printf("|%04d mm", distance_mm_to_monitor - distance_mm);
  udp_client.printf("|%0.1f C", sht.getTemperature());
  udp_client.printf("|%0.1f %|", sht.getHumidity());
  udp_client.endPacket();
  

  if (command_to_run=="Enable Fountain") {
    Serial.println("Enable Fountain");
    command_to_realy_din_rail_block = "Enable Fountain";
  }
  else if (command_to_run=="Disable Fountain") {
    Serial.println("Disable Fountain");
    command_to_realy_din_rail_block = "Disable Fountain";
  }
  else if (command_to_run=="Enable Garland") {
    Serial.println("Enable Garland");
    digitalWrite(GARLAND_PIN, HIGH);
  }
  else if (command_to_run=="Disable Garland") {
    Serial.println("Disable Garland");
    digitalWrite(GARLAND_PIN, LOW);
  }


 //calculate average distance and save it to the EEPROM
  if (!digitalRead(BUTTON)) {
    if (i<10) {
      distance_mm_average += distance_mm;
      if (i==9) { 
        distance_mm_to_monitor = (int)(distance_mm_average / 10);
        EEPROM.put(0, distance_mm_to_monitor);
        EEPROM.commit();
        delay(3000);
        flash_led(2);
        delay(1000);
        Serial.print("Average distance to monitor: ");
        Serial.println(distance_mm_to_monitor);
      }
      i++;
    }
  }
  else {
    i = 0;
    distance_mm_average = 0;
  }
 


  delay(1000);
}
