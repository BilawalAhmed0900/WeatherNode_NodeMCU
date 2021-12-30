#include <Adafruit_BMP280.h>
#include <DHTesp.h>
#include <ESP8266WiFi.h>
#include <ThingSpeak.h>
#include "TinyGPS++.h"
#include <SoftwareSerial.h>
#include <Wire.h>

#define BMP280_I2C_ADDRESS (0x76)
#define SEALEVELPRESSURE_HPA (1013.25)

/*
 * has following fieds #define'd
 * 
 * THINGSPEAK_WRITE_KEY
 * CHANNEL_NUMBER
 * WIFI_SSID
 * WIFI_PASSWORD
 */
#include "Secrets.h"

Adafruit_BMP280 bmp280;
DHTesp dht;
WiFiClient client;

float temp = 0, pres = 0, humid = 0;

unsigned long previous_time_ms = 0;
unsigned long previous_time_thingspeak_ms = 0;
const unsigned long timer_delay_ms = 2000;
const unsigned long timer_delay_thingspeak_ms = 20000;
volatile bool first_whole = true;
volatile bool first_thingspeak = true;

int result = 0;

void setup() {
  Serial.begin(9600);
  
  dht.setup(D0, DHTesp::DHT11);
  dht.getHumidity();
  
  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);

  if (!bmp280.begin(BMP280_I2C_ADDRESS))
  {
    Serial.println("BMP280 initialization failed");
    while (1)
    {
      delay(500);
    }
  }
  Serial.println("BMP280 initialization successfull");
  // digitalWrite(SYSTEM_STATUS_LED, HIGH);
  // delay(2000);
}

float lm35_getTemperature(int pin);

void loop() {
  if (first_whole || (millis() - previous_time_ms) > timer_delay_ms)
  {
    first_whole = false;
    
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.print("WiFi not connected, connecting to WiFi...");
      while (WiFi.status() != WL_CONNECTED)
      {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        delay(5000);
        Serial.print(".");
      }

      Serial.println(" Connected successfully...");
    }
    
    temp = lm35_getTemperature(A0);
    pres = 0;
    pres = bmp280.readPressure();
    humid = dht.getHumidity();
    Serial.println("Read values from sensors:");
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" *C");
    Serial.print("Pressure: ");
    Serial.print(pres);
    Serial.println(" Pa");
    Serial.print("Humidity: ");
    Serial.print(humid);
    Serial.println(" %");
    if ((!isnan(temp) && !isnan(pres) && !isnan(humid)) && (first_thingspeak || ((millis() - previous_time_thingspeak_ms) > timer_delay_thingspeak_ms)))
    {
      first_thingspeak = false;
      Serial.println("Writting to Thingspeak...");
      ThingSpeak.setField(1, temp);
      ThingSpeak.setField(2, pres);
      ThingSpeak.setField(3, humid);
      result = ThingSpeak.writeFields(CHANNEL_NUMBER, THINGSPEAK_WRITE_KEY);
      if (result != 200)
      {
        Serial.println("ThingSpeak gave back " + String(result) + "...");
      }
      else
      {
        Serial.println("Successfully written to ThingSpeak...");
      }

      previous_time_thingspeak_ms = millis();
    }

    Serial.println();
    result = 0;
    previous_time_ms = millis();
  }
}

float lm35_getTemperature(int pin)
{
  return (((float)analogRead(pin)) / 1024.0) * 3300 / 10;
}
