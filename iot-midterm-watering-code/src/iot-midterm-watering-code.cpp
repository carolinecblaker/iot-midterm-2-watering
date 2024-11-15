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

//Adafruit
#include "credentials.h"
#include "JsonParserGeneratorRK.h"

#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"

// SOIL MOISTURE SENSOR
// RELAY


const byte BPEADDRESS=0x76;
const byte GFXADDRESS= 0x3C;
const int SOILPIN = A5;
const int DUSTPIN=D18;
const int RELAYPIN = D2;
const int AQPIN = A1;
const int OLED_RESET=-1;
const int IDEAL_MOISTURE= 1400;
int counter = 0;
int moistureSignal;
int tempF;
int status;
int quality;
int startTime;
bool waterNow = 0;
unsigned int sampletime_ms = 30000;//sampe 30s ;
String dateTime, timeOnly;
unsigned int lastTime;

unsigned int duration;
unsigned int starttime;
const int interval = 50000;//sampe 30s ;
struct RoomData {
  int particles;
  float temp;
  int moisture;
  int quality;
  int humidity;
};
RoomData pubValue;

SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

AirQualitySensor sensor(AQPIN);
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 bme;

TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plant-parameters");

Adafruit_MQTT_Publish moistureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plant-moisture");
Adafruit_MQTT_Publish humidityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plant-humidity");
Adafruit_MQTT_Publish aqFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plant-air-quality");
Adafruit_MQTT_Publish particlesFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plant-particles");
Adafruit_MQTT_Publish tempFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plant-temperature");



void water_plant();
bool MQTT_ping();
void MQTT_connect();
void publishRoomData();
float readDust();
void createFloatPayLoad(Adafruit_MQTT_Publish, float item);
void createIntPayLoad(Adafruit_MQTT_Publish, int item);
void createEventPayLoad(float particles, float temp, int moisture, int quality, int humidity );
// setup() runs once, when the device is first turned on
void setup() {
  // Begins quietly.
  Serial.begin(9600);
  Time.zone(-7);
  Particle.syncTime();
  pinMode(RELAYPIN,OUTPUT);
  pinMode(SOILPIN, INPUT);
  display.begin(SSD1306_SWITCHCAPVCC, GFXADDRESS); 
  display.display(); // show splashscreen
  startTime = millis();//get the current time;
  // Is this necessary
  digitalWrite(RELAYPIN,LOW);
  waitFor(Serial.isConnected,15000);
      if (sensor.init()) {
        Serial.println("Sensor ready.");
    } else {
        Serial.println("Sensor ERROR!");
    } 
  for (counter =0;counter < 2; counter++){
    // digitalWrite(RELAYPIN, HIGH);
    // delay(20);
    // digitalWrite(RELAYPIN,LOW);
    // delay(1000);
  }
  
  status = bme.begin(BPEADDRESS);
 // delay(5000);
  if (status == FALSE) {
   Serial.printf("Bme280 at address 0x%02X failed to start\n", BPEADDRESS); 
   } else {
    Serial.printf("Bme280 at address 0x%02X online\n", BPEADDRESS);
   }
  display.clearDisplay();
  display.display();
  
}
void loop() {
  dateTime = Time.timeStr();
  timeOnly= dateTime.substring(11,16);
  pubValue.temp = 9*(bme.readTemperature()/5.0)+32; 
  //pressIN = bme.readPressure()/3386.39; 
  pubValue.humidity  = bme.readHumidity();

  pubValue.moisture= analogRead(SOILPIN);
  pubValue.particles = readDust();
  pubValue.quality = sensor.getValue();

  // ONCE A Something:
   if ((millis()-starttime) > interval)
    {
    MQTT_connect();
    MQTT_ping();
    //Serial.printf("Sensor value: %i\n",sensor.getValue());
    //Serial.printf("The value: %i\n",RoomData.moisture);
    display.clearDisplay();  
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,7);
    display.printf("Mstr: %i\n",pubValue.moisture);
    display.setCursor(0,22);
    display.printf("Sensor: %i\n",pubValue.quality);
    display.setCursor(0,36);
    display.printf(" %s\n",timeOnly.c_str());
    display.display();
    //Publish stuff
    if(mqtt.Update()) {
  
     publishRoomData();
      
    }
    
    starttime = millis();
   if (waterNow){
    water_plant();
   }

    }
}
void water_plant(){
  waterNow = 0;
  digitalWrite(RELAYPIN, HIGH);
  delay(600);
  digitalWrite(RELAYPIN,LOW);
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT...\n");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}
void publishRoomData(){
Serial.printf("Publishing Room data \n"); 

  createIntPayLoad(humidityFeed,pubValue.humidity);
  delay(1000);
  createIntPayLoad(aqFeed,pubValue.quality);
  delay(1000);
  createFloatPayLoad(particlesFeed,pubValue.particles);
  delay(1000);
  createIntPayLoad(moistureFeed,pubValue.moisture);
  delay(1000);
  createFloatPayLoad(tempFeed,pubValue.temp);
  delay(1000);
  createEventPayLoad(pubValue.particles,pubValue.temp,pubValue.moisture,pubValue.quality, pubValue.humidity);
}
void createFloatPayLoad(Adafruit_MQTT_Publish feed, float item){
     JsonWriterStatic <256> jw;
      {
      JsonWriterAutoObject obj(&jw);
      jw.insertKeyValue("value", item);
      
      }
   feed.publish(jw.getBuffer());
}

void createIntPayLoad(Adafruit_MQTT_Publish feed, int item){
     JsonWriterStatic <256> jw;
      {
      JsonWriterAutoObject obj(&jw);
      jw.insertKeyValue("value", item);
      
      }
   feed.publish(jw.getBuffer());
}



void createEventPayLoad(float particles, float temp, int moisture, int quality, int humidity ) {
     JsonWriterStatic <256> jw;
      {
      JsonWriterAutoObject obj(&jw);
      Serial.printf("object: pubData.particles %i, %f, %i, %i, %i\n",particles,temp,moisture,quality,humidity);
      jw.insertKeyValue("particles", particles);
      jw.insertKeyValue("temp", temp);
      jw.insertKeyValue("moisture", moisture);
      jw.insertKeyValue("quality", quality);
      jw.insertKeyValue("humidity", humidity);
     // jw.insertKeyValue("lon", lon);
      //jw.insertKeyValue("lat", lat);
      }
   pubFeed.publish(jw.getBuffer());
}
float readDust() {
    duration = pulseIn(DUSTPIN, LOW);
    int lowpulseoccupancy = 0;
    lowpulseoccupancy = lowpulseoccupancy+duration;
    float concentration;
    if ((millis()-starttime) > sampletime_ms)//if the sampel time == 30s
    {
      float  ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
      concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
        Serial.print(lowpulseoccupancy);
        Serial.print(",");
        Serial.print(ratio);
        Serial.print(",");
        Serial.println(concentration);
        lowpulseoccupancy = 0;
        starttime = millis();
    }
    return concentration;
}