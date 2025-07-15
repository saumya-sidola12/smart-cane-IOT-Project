#include <SoftwareSerial.h>
#include <TinyGPS++.h>

//pins for the ultrasonic sensors
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 11;
const int echoPin2 = 12;
const int trigPin3 = 13;
const int echoPin3 = A0;

//pin for the buzzer
const int buzzerPin = 8;

//pins for the Bluetooth module
const int btTxPin = 1;
const int btRxPin = 0;

//pins for the GPS module
const int gpsTxPin = 4;
const int gpsRxPin = 5;

//SoftwareSerial objects for Bluetooth and GPS modules
SoftwareSerial btSerial(btTxPin, btRxPin);
SoftwareSerial gpsSerial(gpsTxPin, gpsRxPin);

//TinyGPS++ object
SoftwareSerial serial_connection(4, 5);
TinyGPSPlus gps;// GPS object to process the NMEA data

void setup() {
  // Initializing serial communication
  Serial.begin(9600);
  btSerial.begin(9600);
  gpsSerial.begin(9600);
  serial_connection.begin(9600);
    //communications to the GPS

  //trigPins as OUTPUT and echoPins as INPUT
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  //buzzer pin as OUTPUT
  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  // Measuring distance for each sensor
  long distance1 = measureDistance(trigPin1, echoPin1);
  long distance2 = measureDistance(trigPin2, echoPin2);
  long distance3 = measureDistance(trigPin3, echoPin3);

  // Printing the distances on the Serial Monitor
  Serial.print("Distance1: ");
  Serial.print(distance1);
  Serial.print(" cm, Distance2: ");
  Serial.print(distance2);
  Serial.print(" cm, Distance3: ");
  Serial.print(distance3);
  Serial.println(" cm");

  // Sending distances via Bluetooth
  btSerial.print("Distance1: ");
  btSerial.print(distance1);
  btSerial.print(" cm, Distance2: ");
  btSerial.print(distance2);
  btSerial.print(" cm, Distance3: ");
  btSerial.print(distance3);
  btSerial.println(" cm");

  // Check if any distance is below a threshold and sound the buzzer
  if (distance1 < 50 || distance2 < 50 || distance3 < 50) {
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }


 while (serial_connection.available()) {
    gps.encode(serial_connection.read());
  }
  if (gps.location.isUpdated()) {
    Serial.print("Satellite Count:");
    Serial.println(gps.satellites.value());
    Serial.print("Latitude:");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude:");
    Serial.println(gps.location.lng(), 6);

    Serial.print("Altitude Feet:");
    Serial.println(gps.altitude.feet());
    Serial.println("");

    Serial.print("Date: ");
    if (gps.date.isValid()) {
      Serial.print(gps.date.month());
      Serial.print("/");
      Serial.print(gps.date.day());
      Serial.print("/");
      Serial.println(gps.date.year());
    }

    Serial.print("Time: ");
    if (gps.time.isValid()) {
      if (gps.time.hour() < 10) Serial.print(F("0"));
      Serial.print(gps.time.hour());
      Serial.print(":");
      if (gps.time.minute() < 10) Serial.print(F("0"));
      Serial.print(gps.time.minute());
      Serial.print(":");
      if (gps.time.second() < 10) Serial.print(F("0"));
      Serial.print(gps.time.second());
      Serial.print(".GMT");
    }
    Serial.println("");
    delay(2000);
  }
}

long measureDistance(int trigPin, int echoPin) {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Set the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance
  long distance = duration * 0.034 / 2;

  return distance;
}