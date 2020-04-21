#include <Wire.h>
#include "SHTSensor.h"
#include "github.h"
#include "wifi_details.h"
#include <ArduinoOTA.h>
#include <HTTPUpdate.h>

#define US_ROUNDTRIP_CM 57      // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space.
// Ultrasonic sensor
#define trigPin  13
#define echoPin  14
#define NoBlind_UltrasonicConvert(echoTime, conversionFactor) (max((echoTime + conversionFactor / 2) / conversionFactor, (echoTime ? 1 : 0)))
// SHT85 sensor
#define SCL_pin  22
#define SDA_pin  21

#define LED_PIN            2
#define SKETCH_VERSION "1.0.30"

SHTSensor sht(SHTSensor::SHT3X);

WiFiClientSecure Secure_client;
int duration;
int distance;
unsigned long check_for_the_new_frimware_millis;



void setup() {
  Serial.print("Firmware version: ");
  Serial.println(SKETCH_VERSION);
  Serial.begin(115200); // Starts the serial communication
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
    // delete old config
  WiFi.disconnect(true);
  // Examples of different ways to register wifi events
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  Serial.println("Wait for WiFi... ");

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

  
  
  Wire.begin(SDA_pin, SCL_pin);

  if (sht.init()) {
    Serial.print("init(): success\n");
  } else {
    Serial.print("init(): failed\n");
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x
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
distance= duration*0.034/2;

// Prints the distance on the Serial Monitor
Serial.print("Distance: ");
Serial.println(NoBlind_UltrasonicConvert(duration, US_ROUNDTRIP_CM)); // Convert uS to centimeters.);
//Serial.println(distance); // Convert uS to centimeters.);


  if (sht.readSample()) {
      Serial.print("SHT:\n");
      Serial.print("  RH: ");
      Serial.print(sht.getHumidity(), 2);
      Serial.print("\n");
      Serial.print("  T:  ");
      Serial.print(sht.getTemperature(), 2);
      Serial.print("\n");
  } else {
      Serial.print("Error in readSample()\n");
  }



  if (millis()-check_for_the_new_frimware_millis>30000) {
    Secure_client.setCACert(rootCACertificate);
    // Reading data over SSL may be slow, use an adequate timeout
    Secure_client.setTimeout(12000/1000);

    // The line below is optional. It can be used to blink the LED on the board during flashing
    // The LED will be on during download of one buffer of data from the network. The LED will
    // be off during writing that buffer to flash
    // On a good connection the LED should flash regularly. On a bad connection the LED will be
    // on much longer than it will be off. Other pins than LED_BUILTIN may be used. The second
    // value is used to put the LED on. If the LED is on with HIGH, that value should be passed
    httpUpdate.setLedPin(LED_PIN, LOW);
    t_httpUpdate_return ret = httpUpdate.update(Secure_client, "https://raw.githubusercontent.com/0009281/fountain_sensors/master/fountain_sensors_client.ino.nodemcu-32s.bin");
    check_for_the_new_frimware_millis-millis();
  }



delay(1000);
}