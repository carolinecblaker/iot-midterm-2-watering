/* 
 * Project Watering System - IOT Midterm 2
 * Author: Caroline C Blaker
 * Date: 11-11-2024
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
// SEEED DUST SENSOR
// SEEED AIR QUALITY SENSOR
#include "Air_Quality_Sensor.h"
// BME 
#include "Adafruit_BME280.h"
// OLED
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
// SOIL MOISTURE SENSOR
// RELAY

const byte BPEADDRESS=0x76;
const byte GFXADDRESS= 0x3C;
const int SOILPIN = A2;
const int RELAYPIN = D2;
const int OLED_RESET=-1;
const int IDEAL_MOISTURE= 1400;
int counter = 0;
int moistureSignal;
int tempF;
int status;
int quality;

SYSTEM_MODE(SEMI_AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

AirQualitySensor sensor(A1);
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 bme;


void water_plant();

// setup() runs once, when the device is first turned on
void setup() {
  // Begins quietly.
  Serial.begin(9600);
  pinMode(RELAYPIN,OUTPUT);
  pinMode(SOILPIN, INPUT);
  display.begin(SSD1306_SWITCHCAPVCC, GFXADDRESS); 
  display.display(); // show splashscreen
   
  // Is this necessary
  digitalWrite(RELAYPIN,LOW);
  for (counter =0;counter < 2; counter++){
    digitalWrite(RELAYPIN, HIGH);
    delay(20);
    digitalWrite(RELAYPIN,LOW);
    delay(1000);
  }
  waitFor(Serial.isConnected,10000);
  status = bme.begin(BPEADDRESS);
  if (status == FALSE) {
    Serial.printf("Bme280 at address 0x%02X failed to start", BPEADDRESS); 
   } else {
    Serial.printf("Bme280 at address 0x%02X online", BPEADDRESS);
   }
  display.clearDisplay();
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.
  quality = sensor.slope();

  Serial.printf("Sensor value: %i ",sensor.getValue());
  tempF = 9*(bme.readTemperature()/5.0)+32; 
  //pressIN = bme.readPressure()/3386.39; 
  //humidRH = bme.readHumidity();
  moistureSignal= analogRead(SOILPIN);
  // ONCE A MINUTE:
  Serial.printf("The value: %i\n",moistureSignal);
  display.clearDisplay();  
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,10);
  display.printf("Mstr: %i\n",moistureSignal);
  display.setCursor(0,20);
  display.printf("Temp: %i\n",tempF);
  display.display();
  delay(1000);
  digitalWrite(RELAYPIN,LOW);
  delay(1000);
}
void water_plant(){
  digitalWrite(RELAYPIN, HIGH);
  delay(600);
  digitalWrite(RELAYPIN,LOW);
}