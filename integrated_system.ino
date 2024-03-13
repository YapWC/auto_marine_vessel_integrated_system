#include <ESP32Servo.h>
#include <algorithm>
#include <QMC5883LCompass.h>
#include <Wire.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// For Bearing Calculation for Current and Destination Coordinate
#define _USE_MATH_DEFINES
#include <math.h>

#define TRIG_PIN 5   // ESP32 pin GPIO23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 12  // ESP32 pin GPIO22 connected to Ultrasonic Sensor's ECHO pin
#define SERVO_PIN 13 // ESP32 pin GPIO32 connected to Servo Motor's pin
#define DISTANCE_THRESHOLD 80 // centimeters

#define PWM_A 23 // Control Motor A Speed 
#define MOTOR_A 26  // Motor A direction pins
#define MOTOR_AA 25
#define PWM_B 33  // Control Motor B Speed
#define MOTOR_B 2 // Motor B direction pins
#define MOTOR_BB 4


QMC5883LCompass compass;

Servo servo; // create servo object to control a servo

static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial softSerial(RXPin, TXPin);


// WiFi parameters
#define WLAN_SSID       "Hilman’s iPhone"
#define WLAN_PASS       "hilman318"
 
 
// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
//Enter the username and key from the Adafruit IO
#define AIO_USERNAME    "mhilmanz"
#define AIO_KEY         "" 
WiFiClient client;
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish GPSLocation = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/GPS Location/csv");
Adafruit_MQTT_Publish Ultrasonic = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Ultrasonic Display/csv");
Adafruit_MQTT_Publish Magnetometer = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Magnetometer Display/csv");
Adafruit_MQTT_Publish Target = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Target Angle/csv");


// variables will change:
float duration_us, distance_cm;
int incomingByte = 0; 

//gps
float speed_mph = 0;
float alltitude = 0;
float lati; //Storing the Latitude
float longi; //Storing the Longitude
float destination_x;
float destination_y;
double bearing;
char gpsdata[120];
int a;

#define ARRAY_SIZE 5 // Size of the array
#define DELAY_BETWEEN_MEASUREMENTS 30 // Delay between ultrasonic measurements in milliseconds

float filterArray[ARRAY_SIZE]; // Array to store data samples from the sensor
float filtered_distance; // Store the filtered distance from the sensor
int  input = 0;
//publish
unsigned long previousMillis = 0;
unsigned long currentMillis;

void setup() {
  Serial.begin(115200);       // initialize serial port
  softSerial.begin(GPSBaud);
  pinMode(TRIG_PIN, OUTPUT); // set ESP32 pin to output mode
  pinMode(ECHO_PIN, INPUT);  // set ESP32 pin to input mode
  servo.attach(SERVO_PIN);   // attaches the servo on pin 9 to the servo object
  servo.write(96);
  compass.setADDR(0x0D);
  compass.init();

  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_AA, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  pinMode(MOTOR_BB, OUTPUT);

  digitalWrite(MOTOR_A, LOW);
  digitalWrite(MOTOR_AA, LOW);
  digitalWrite(MOTOR_B, LOW);
  digitalWrite(MOTOR_BB, LOW);

  // Set the destination coordinate here
  destination_x = 4.3816416;
  destination_y = 100.9668201;

  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
 
  // connect to adafruit io
  connect();
}

// connect to adafruit io via MQTT
void connect() {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }
 
    if(ret >= 0)
      mqtt.disconnect();
 
    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}

void loop() {
  forward();
  // Adafruit MQTT initialization:
  if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }
   
  // Ultrasonic sensor measurements with noise filtering
  // 1. Taking multiple measurements and storing in an array
  for (int sample = 0; sample < ARRAY_SIZE; sample++) {
    filterArray[sample] = ultrasonicMeasure();
    Serial.print("In Array: ");
    Serial.print(filterArray[sample]);
    delay(DELAY_BETWEEN_MEASUREMENTS);
  }

  // 2. Sorting the array in ascending order
  std::sort(filterArray, filterArray + ARRAY_SIZE);

  // 3. Filtering noise and calculating the average distance
  int startIdx = ARRAY_SIZE / 4;       // Start from the 25th percentile
  int endIdx = 3 * ARRAY_SIZE / 4 - 1; // End at the 75th percentile
  double sum = 0;

  for (int sample = startIdx; sample <= endIdx; sample++) {
    sum += filterArray[sample];
  }

  filtered_distance = sum / (endIdx - startIdx + 1);

  // Print the value to Serial Monitor
  Serial.print("Filtered Distance: ");
  Serial.print(filtered_distance);
  Serial.println(" cm");

 /*if (!Ultrasonic.publish(filtered_distance)) {                     //Publish to Adafruit
      Serial.println(F("Ultrasonic Failed"));
    }
    else {
      Serial.println(F("Ultrasonic Sent!"));
    }
    //delay(2000);
    */
  //Magentometer
    
  
  // Read compass values
  compass.read();

  // Return Azimuth reading
  a = compass.getAzimuth();
  a = a - 90
  if (a < 0){
    a = 360 + a; // minus 80 for calibration
  }
  Serial.print("Vessel Angle: ");
  Serial.print(a);
  Serial.println();

   /* if (!Magnetometer.publish(a)) {                     //Publish to Adafruit
      Serial.println(F("Bearing Failed"));
    }
    else {
      Serial.println(F("Bearing Sent!"));
    }
    delay(5000);
    */
//gps
  getCoordinates();
  Serial.print("Lati = ");
  Serial.print(lati,17);
  Serial.print("\tLongi = ");
  Serial.println(longi,17);
/*
    if (!GPSLocation.publish(gpsdata)) {                     //Publish to Adafruit
      Serial.println(F("GPS Failed"));
    }
    else {
      Serial.println(F("GPS Sent!"));
    }
    //delay(2000);
    */
  bearing = calculate_gps_heading(lati, longi, destination_x, destination_y);
  Serial.print("Target Angle = ");
  Serial.println(bearing,17);
  /*
      if (!Target.publish(bearing)) {                     //Publish to Adafruit
      Serial.println(F("Target Angle Failed"));
    }
    else {
      Serial.println(F("Target Angle Sent!"));
    }
    //delay(2000);
*/
// motor
  if (bearing-10 < a && a < bearing+10) {
    // boat remain straight line
    servo.write(96);
    forward();
    delay(1500);
    if (filtered_distance < DISTANCE_THRESHOLD) {
      servo.write(36);
      delay(1000);
    }
  }
  else if (a > bearing+10) {
    //boat need to turn left
    servo.write(126);
    left();
    delay(1500);
    if (filtered_distance < DISTANCE_THRESHOLD) {
      servo.write(156);
      delay(1000);
    }
  }
  else if (a < bearing-10) {
  //boat need to turn right
    servo.write(66);
    right();
    delay(1500);
    if (filtered_distance < DISTANCE_THRESHOLD) {
      servo.write(36);
      delay(1000);
    }
  }


  //motor
  // 1 unit of coordinate is equal to 111.195km
  if ((lati >= destination_x-0.00001 && lati <= destination_x+0.00001) && 
    (longi >= destination_y-0.00001 && longi <= destination_y+0.00001)){
      // if the distance between vessel and target coordinate is less than 1.11m radius
      // then destination is considered reach therefore vessel stop
      stop();
    }
  
   // Check if it's time to publish
  if (currentMillis - previousMillis >= 8000) {
    // Save the last time data was published
    previousMillis = currentMillis;
    // Publish your data here
    publishData(bearing, gpsdata, filtered_distance, a);
    //publishData();
  }
  currentMillis = millis();
}

// Starting from here it is all functions
float ultrasonicMeasure() {
  // Generate a 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure duration of pulse from ECHO pin
  float duration_us = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance
  return 0.017 * duration_us;
}

//motor function
void backward() {          //function of forward 
  digitalWrite(MOTOR_A, HIGH);
  digitalWrite(MOTOR_AA, LOW);
  digitalWrite(MOTOR_B, HIGH);
  digitalWrite(MOTOR_BB, LOW);

  analogWrite(PWM_A, 90);
  analogWrite(PWM_B, 90);
}

void forward() {         //function of backward
  digitalWrite(MOTOR_A, LOW);
  digitalWrite(MOTOR_AA, HIGH);
  digitalWrite(MOTOR_B, LOW);
  digitalWrite(MOTOR_BB, HIGH);

  analogWrite(PWM_A, 150);
  analogWrite(PWM_B, 150);
}

void right() {         //function of backward
  digitalWrite(MOTOR_A, LOW);
  digitalWrite(MOTOR_AA, HIGH);
  digitalWrite(MOTOR_B, LOW);
  digitalWrite(MOTOR_BB, HIGH);

  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 150);
}

void left() {         //function of backward
  digitalWrite(MOTOR_A, LOW);
  digitalWrite(MOTOR_AA, HIGH);
  digitalWrite(MOTOR_B, LOW);
  digitalWrite(MOTOR_BB, HIGH);

  analogWrite(PWM_A, 150);
  analogWrite(PWM_B, 0);
}

void stop() {              //function of stop
  digitalWrite(MOTOR_A, LOW);
  digitalWrite(MOTOR_AA, LOW);
  digitalWrite(MOTOR_B, LOW);
  digitalWrite(MOTOR_BB, LOW);
}

//gps
void getCoordinates()
{
 readGPSData();
 char *p = gpsdata;
 // add speed value
 dtostrf(speed_mph, 2, 17, p);
 p += strlen(p);
 p[0] = ','; p++;
 // concat latitude
 dtostrf(lati, 2, 17, p);
 p += strlen(p);
 p[0] = ','; p++;
 // concat longitude
 dtostrf(longi, 3, 17, p);
 p += strlen(p);
 p[0] = ','; p++;
 // concat altitude
 dtostrf(alltitude, 2, 17, p);
 p += strlen(p);
 // null terminate
 p[0] = 0;
}
void readGPSData()
{
 if(gps.location.isValid()){
 lati = gps.location.lat();
 longi = gps.location.lng();
 Serial.print("Lati: ");
 Serial.print(lati,17);
 Serial.print("\tLongi: ");
 Serial.println(longi,17);
 }
 waitGPS(100);
 if (millis() > 5000 && gps.charsProcessed() < 10)
 Serial.println("Waiting for data...");
}
static void waitGPS(unsigned long ms)
{
 unsigned long start = millis();
 do
 {
 while (softSerial.available())
 gps.encode(softSerial.read());
 } while (millis() - start < ms);
}


// For Bearing Calculation for Current and Destination Coordinate
static inline double to_rad(double theta) {
    return (theta * M_PI) / 180.0;
}

static inline double to_degrees(double theta) {
    return (theta * 180.0) / M_PI;
}

//
// Calculate the heading in decimal degrees between 2 (preferably quite close) locations 
// (lat1, lon1) --> First location
// (lat2, lon2) --> Second location
//
double calculate_gps_heading(double lat1, double lon1, double lat2, double lon2) {
    // Convert degrees to radians
    lat1 = to_rad(lat1);
    lon1 = to_rad(lon1);
    lat2 = to_rad(lat2);
    lon2 = to_rad(lon2);    
        
    double dlon = lon2 - lon1;
    double X = cos(lat2) * sin(dlon);
    double Y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    
    double heading = atan2(X,Y);
    
    // We want heading in degrees, not radians.
    heading = to_degrees(heading);
    if (heading < 0){
      heading = abs(heading)+180;
    }
    
    return heading;
}
void publishData(double bearing, char gpsdata[120], float filtered_distance, int a) {
    if (!Target.publish(bearing)) {                     //Publish to Adafruit
    Serial.println(F("Target Angle Failed"));
    }
    else {
      Serial.println(F("Target Angle Sent!"));
    }
    //delay(2000);

    if (!GPSLocation.publish(gpsdata)) {                     //Publish to Adafruit
      Serial.println(F("GPS Failed"));
    }
    else {
      Serial.println(F("GPS Sent!"));
    }
    //delay(2000);
    if (!Ultrasonic.publish(filtered_distance)) {                     //Publish to Adafruit
      Serial.println(F("Ultrasonic Failed"));
    }
    else {
      Serial.println(F("Ultrasonic Sent!"));
    }
    //delay(2000);
    if (!Magnetometer.publish(a)) {                     //Publish to Adafruit
      Serial.println(F("Bearing Failed"));
    }
    else {
      Serial.println(F("Bearing Sent!"));
    }
    //delay(5000);
}
