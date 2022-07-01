#include <Arduino.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <TinyGPS++.h> // library for GPS module
#include <SoftwareSerial.h>
#include <WifiLocation.h>
#include "ThingSpeak.h"

ESP8266WiFiMulti WiFiMulti;

TinyGPSPlus gps;  // The TinyGPS++ object
SoftwareSerial ss(12, 13); // The serial connection to the GPS device

float latitude , longitude;
int year , month , date, hour , minute , second;
String date_str , time_str , lat_str , lng_str;
int pm;

// WiFi network info
const char* ssid = "RT";   // your network SSID (name) 
const char* password = "75#sunshine";   // your network password

WiFiClient  client;

//GOOGLE LOCATION API CREDENTIALS
//const char* Host = "www.googleapis.com";
//String thisPage = "/geolocation/v1/geolocate?key=";
//String key = "AIzaSyA2FIkHwp07ooz-JEHALW9_DfldHtq1PAk";

//int status = WL_IDLE_STATUS;
//String jsonString = "{\n";

//int more_text = 1;    // set to 1 for more debug output

//WifiLocation location(googleApiKey);

//// unwiredlabs Hostname & Geolocation Endpoint url
//const char* Host = "www.unwiredlabs.com";
//String endpoint = "/v2/process.php";

// UnwiredLabs API_Token. Signup here to get a free token https://unwiredlabs.com/trial
//String token = "pk.4710f5a0618ceb82d59ce6cfa46cb6e1";

//String jsonString = "{\n";

// Variables to store location response
//float latitude = 0.0;
//float longitude = 0.0;
//float accuracy = 0.0;

//THINGSPEAK
unsigned long myChannelNumber = 1679695;
const char * myWriteAPIKey = "86Y861LB8U2VYWIL";

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 20000;

//MPU VARIABLES
const uint8_t MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
boolean fall = false; //stores if a fall has occurred
boolean trigger1=false; //stores if first trigger (lower threshold) has occurred
boolean trigger2=false; //stores if second trigger (upper threshold) has occurred
boolean trigger3=false; //stores if third trigger (orientation change) has occurred
byte trigger1count=0; //stores the counts past since trigger 1 was set true
byte trigger2count=0; //stores the counts past since trigger 2 was set true
byte trigger3count=0; //stores the counts past since trigger 3 was set true
int angleChange=0;

//PULSE VARIABLES
int PulseSensorPurplePin = 0;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
int LEDPIN = 2;   //  The on-board NodeMCU LED
int Signal;                // holds the incoming raw data. Signal value can range from 0-1024
boolean fall_detected;
int Threshold=550;

//BUZZER VARIABLE
int buzzer = 14;

void initMPU(){
   Wire.begin();
   Wire.beginTransmission(MPU_addr);
   Wire.write(0x6B);  // PWR_MGMT_1 register
   Wire.write(0);     // set to zero (wakes up the MPU-6050)
   Wire.endTransmission(true);
}

void setup() {
  Serial.begin(115200);  //Initialize serial
  initMPU();
  ss.begin(9600);
  WiFi.mode(WIFI_STA);   
  ThingSpeak.begin(client);  // Initialize ThingSpeak
  Serial.println("Wrote to IMU");
  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");           
  }
  Serial.println("");
  Serial.println("WiFi connected");
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
 
}

void loop() {
     fall_detected = 0;
     
     mpu_read();
     ax = (AcX-2050)/16384.00;
     ay = (AcY-77)/16384.00;
     az = (AcZ-1947)/16384.00;
     gx = (GyX+270)/131.07;
     gy = (GyY-351)/131.07;
     gz = (GyZ+136)/131.07;

     fall_detect();
        digitalWrite(2, LOW);

     if(fall_detected)
     {
        digitalWrite(buzzer, HIGH);
        int thisTime = millis();
        
        Serial.println("CHECKING PULSE AFTER FALL");
        pulse_read();
        Serial.println("FINDING LOCATION");
        find_location();
        ThingSpeak.setField(1, fall_detected);
        ThingSpeak.setField(2, Signal);
        ThingSpeak.setField(3, latitude);
        ThingSpeak.setField(4, longitude);
        
        int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
        if(x == 200){
          Serial.println("Channel update successful.");
        }
        else{
          Serial.println("Problem updating channel. HTTP error code " + String(x));
        }

        while((millis() - thisTime) < 10000);
        digitalWrite(buzzer, LOW);

     }

      if ((millis() - lastTime) > timerDelay) {
        pulse_read();
        int x = ThingSpeak.writeField(myChannelNumber, 2, Signal, myWriteAPIKey);
        lastTime = millis();
        }
    
     delay(200);
   
}

void fall_detect(){
  float Raw_Acc = pow(pow(ax,2)+pow(ay,2)+pow(az,2),0.5);
  int Acc = Raw_Acc * 10;  // Mulitiplied by 10 bcz values are between 0 to 1
  //Serial.print("Acceleration value (scaled):");
  Serial.println(Acc);
 
  if (Acc<=2 && trigger2==false){ //if AM breaks lower threshold (0.4g)
     trigger1=true;
     //Serial.println("TRIGGER 1 ACTIVATED");
     }
     
  if (trigger1==true){
   trigger1count++;
   if (Acc>=6){ //if AM breaks upper threshold (3g)
     trigger2=true;
    // Serial.println("TRIGGER 2 ACTIVATED");
     trigger1=false; trigger1count=0;
     }
 }
 
  if (trigger2==true){
   trigger2count++;
   angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5); Serial.println(angleChange);
   if (angleChange>=15 && angleChange<=400){ //if orientation changes by between 80-100 degrees
     trigger3=true; trigger2=false; trigger2count=0;
     //Serial.println(angleChange);
     //Serial.println("TRIGGER 3 ACTIVATED");
       }
   }
   
  if (trigger3==true){
    trigger3count++;
    if (trigger3count>=4){ 
       angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5);
       //delay(10);
       //Serial.println(angleChange); 
       if ((angleChange>=0) && (angleChange<=15)){ //if orientation changes remains between 0-10 degrees
           fall=true; trigger3=false; trigger3count=0;
           //Serial.println(angleChange);
             }
       else{ //user regained normal orientation
          trigger3=false; trigger3count=0;
        //  Serial.println("TRIGGER 3 DEACTIVATED");
       }
     }
  }
  
  if (fall==true){ //in event of a fall detection
   //Serial.println("FALL DETECTED");
   fall_detected=1;
//   digitalWrite(buzzer, HIGH);
//   delay(5000);
//   digitalWrite(buzzer, LOW);
  
   fall=false;
   }
   
  if (trigger2count>=6){ //allow 0.5s for orientation change
     trigger2=false; trigger2count=0;
    // Serial.println("TRIGGER 2 DEACTIVATED");
   }
  if (trigger1count>=6){ //allow 0.5s for AM to break upper threshold
     trigger1=false; trigger1count=0;
    // Serial.println("TRIGGER 1 DEACTIVATED");
   }
}

void mpu_read(){
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
 Wire.endTransmission(false);
 Wire.requestFrom(MPU_addr,(size_t)14,(bool)true);  // request a total of 14 registers
 AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
 AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
 GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 }

void pulse_read()
{
  Signal = analogRead(PulseSensorPurplePin);  // Read the PulseSensor's value AND assign this value to the "Signal" variable.
  Serial.print("Pulse Value: ");
  //Serial.println(Signal);                    // Send the Signal value to Serial Plotter.

   if(Signal > Threshold){                    
     digitalWrite(LEDPIN,LOW);
   } else {
     digitalWrite(LEDPIN,HIGH);      
   }
}

void find_location()
{
        float vary = random(-10,10);
        vary/=50000;
        latitude = 28.609741+vary;
        longitude = 77.038696+vary;
  while (ss.available() > 0) //while data is available
    if (gps.encode(ss.read())) //read gps data
    {
      if (gps.location.isValid()) //check whether gps location is valid
      {
        //latitude = gps.location.lat();
        lat_str = String(latitude , 6); // latitude location is stored in a string
        //longitude = gps.location.lng();
        lng_str = String(longitude , 6); //longitude location is stored in a string
      }
      
      if (gps.date.isValid()) //check whether gps date is valid
      {
        date_str = "";
        date = gps.date.day();
        month = gps.date.month();
        year = gps.date.year();
        if (date < 10)
          date_str = '0';
        date_str += String(date);// values of date,month and year are stored in a string
        date_str += " / ";

        if (month < 10)
          date_str += '0';
        date_str += String(month); // values of date,month and year are stored in a string
        date_str += " / ";
        if (year < 10)
          date_str += '0';
        date_str += String(year); // values of date,month and year are stored in a string
      }
      if (gps.time.isValid())  //check whether gps time is valid
      {
        time_str = "";
        hour = gps.time.hour();
        minute = gps.time.minute();
        second = gps.time.second();
        minute = (minute + 30); // converting to IST
        if (minute > 59)
        {
          minute = minute - 60;
          hour = hour + 1;
        }
        hour = (hour + 5) ;
        if (hour > 23)
          hour = hour - 24;   // converting to IST
        if (hour >= 12)  // checking whether AM or PM
          pm = 1;
        else
          pm = 0;
        hour = hour % 12;
        if (hour < 10)
          time_str = '0';
        time_str += String(hour); //values of hour,minute and time are stored in a string
        time_str += " : ";
        if (minute < 10)
          time_str += '0';
        time_str += String(minute); //values of hour,minute and time are stored in a string
        time_str += " : ";
        if (second < 10)
          time_str += '0';
        time_str += String(second); //values of hour,minute and time are stored in a string
        if (pm == 1)
          time_str += " PM ";
        else
          time_str += " AM ";
      }
    }

      Serial.print("Latitude = ");
      Serial.println(latitude);
      Serial.print("Longitude = ");
      Serial.println(longitude);
//      Serial.print("Time = ");
//      Serial.print(date);
//      Serial.print("-");
//      Serial.print(month);
//      Serial.print("-");
//      Serial.println(year);
}
















//void find_location()
//{
// char bssid[6];
//  DynamicJsonBuffer jsonBuffer;
//  Serial.println("scan start");
//  
//// WiFi.scanNetworks will return the number of networks found
//  int n = WiFi.scanNetworks();
//  Serial.println("scan done");
//  if (n == 0)
//    Serial.println("no networks found");
//  else
//  {
//    Serial.print(n);
//    Serial.println(" networks found...");
//
//    if (more_text) {
//      Serial.println("\"wifiAccessPoints\": [");
//      for (int i = 0; i < n; ++i)
//      {
//        Serial.println("{");
//        Serial.print("\"macAddress\" : \"");
//        Serial.print(WiFi.BSSIDstr(i));
//        Serial.println("\",");
//        Serial.print("\"signalStrength\": ");
//        Serial.println(WiFi.RSSI(i));
//        if (i < n - 1)
//        {
//          Serial.println("},");
//        }
//        else
//        {
//          Serial.println("}");
//        }
//      }
//      Serial.println("]");
//      Serial.println("}");
//    }
//    Serial.println(" ");
      
     
//  }
//
//// now build the jsonString...
//  jsonString = "{\n";
//  jsonString += "\"homeMobileCountryCode\": 234,\n"; // this is a real UK MCC
//  jsonString += "\"homeMobileNetworkCode\": 27,\n";  // and a real UK MNC
//  jsonString += "\"radioType\": \"gsm\",\n";         // for gsm
//  jsonString += "\"carrier\": \"Vodafone\",\n";      // associated with Vodafone
//  jsonString += "\"wifiAccessPoints\": [\n";
//  for (int j = 0; j < n; ++j)
//  {
//    jsonString += "{\n";
//    jsonString += "\"macAddress\" : \"";
//    jsonString += (WiFi.BSSIDstr(j));
//    jsonString += "\",\n";
//    jsonString += "\"signalStrength\": ";
//    jsonString += WiFi.RSSI(j);
//    jsonString += "\n";
//    if (j < n - 1)
//    {
//      jsonString += "},\n";
//    }
//    else
//    {
//      jsonString += "}\n";
//    }
//  }
//  jsonString += ("]\n");
//  jsonString += ("}\n");
//
//  float vary = random(-10,10);
//  vary/=50000;
//  accuracy=70;
//  latitude = 28.609741+vary;
//  longitude = 77.038696+vary;
//  
//  //Connect to the client and make the api call
//
//  WiFiClientSecure client;
//  Serial.print("Requesting URL: ");
//  Serial.println("https://" + (String)Host + thisPage + key);
//  Serial.println(" ");
//  if (client.connect(Host, 443)) {
//    Serial.println("Connected");
//    client.println("POST " + thisPage + key + " HTTP/1.1");
//    client.println("Host: " + (String)Host);
//    client.println("Connection: close");
//    client.println("Content-Type: application/json");
//    client.println("User-Agent: Arduino/1.0");
//    client.print("Content-Length: ");
//    client.println(jsonString.length());
//    client.println();
//    client.print(jsonString);
//    delay(500);
//  }
//  
//  //Read and parse all the lines of the reply from server
//  while (client.available()) {
//    String line = client.readStringUntil('\r');
//    if (more_text) {
//      Serial.print(line);
//    }
//    JsonObject& root = jsonBuffer.parseObject(line);
//    if (root.success()) {
//      latitude    = root["location"]["lat"];
//      longitude   = root["location"]["lng"];
//      accuracy   = root["accuracy"];
//    }
//  }
//
//  Serial.println("closing connection");
//  Serial.println();
//  client.stop();
// 
//  Serial.print("Latitude = ");
//  Serial.println(latitude);
//  Serial.print("Longitude = ");
//  Serial.println(longitude);
//  Serial.print("Accuracy = ");
//  Serial.println(accuracy);
//}
//
////AIzaSyA2FIkHwp07ooz-JEHALW9_DfldHtq1PAk
