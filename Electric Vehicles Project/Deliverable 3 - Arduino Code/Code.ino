#include "Arduino.h"
#include "uRTCLib.h"
#include "SPI.h"
#include "SD.h"

uRTCLib rtc(0x68);
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const int chipSelect = 4;

const int hallSensorPin = A5;
const int voltageSensorPin = A0;
const int currentSensorPin = A1;

const float wheel_radius = 0.28;
const float current_senstivity = 0.066;
const int numReadings = 10;

float freqReadings[numReadings];
float voltageReadings[numReadings];
float currentReadings[numReadings];

float prevDuration1 = 0.0;
float prevDuration2 = 0.0;

float f_total = 0.0;
float v_total = 0.0;
float i_total = 0.0;

int readIndex = 0;

float get_speed();
float get_voltage();
float get_current();
String get_time();

void setup() {
  Serial.begin(9600);
  URTCLIB_WIRE.begin();
  pinMode(hallSensorPin, INPUT);
  while (!Serial);
  for (int thisReading = 0; thisReading < numReadings; thisReading++){
    freqReadings[thisReading] = 0.0;
    voltageReadings[thisReading] = 0.0;
    currentReadings[thisReading] = 0.0;   
  }

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1);
  }
  Serial.println("card initialized.");

  // For RTC Setup (Comment out once set)
  // rtc.set(second, minute, hour, dayOfWeek, dayOfMonth, month, year)      
  rtc.set(0, 0, 0, 0, 14, 4, 23); 
}

void loop() {
  String dataString = "";
  if (readIndex >= numReadings){
    readIndex = 0;
  }
  float velocity = get_speed();
  float voltage = get_voltage();
  float current = get_current();
  String time = get_time();
  dataString += String(velocity) + " km/h ," + String(voltage) + " V, " + String(current) + " A" + time;
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);    
  }
  else {
    Serial.println("error opening datalog.txt");
  }   
  readIndex += 1;
}

float get_speed(){
  double pulseDuration1 = pulseIn(hallSensorPin, HIGH);
  double pulseDuration2 = pulseIn(hallSensorPin, LOW);
  if (pulseDuration1 != 0.0 && pulseDuration2 != 0.0) {
    if (pulseDuration1 < 50){
      pulseDuration1 = prevDuration1;
    }
    if (pulseDuration2 < 50){
      pulseDuration2 = prevDuration2;
    }
    prevDuration1 = pulseDuration1;
    prevDuration2 = pulseDuration2;
    double timePeriod = pulseDuration1 + pulseDuration2;
    double frequency = 1e6 / timePeriod;
    f_total -= freqReadings[readIndex];
    freqReadings[readIndex] = frequency;
    f_total += freqReadings[readIndex];
    float freq_avg = f_total / float(numReadings);
    float rpm = (2.339 * freq_avg) - 2.505;
    float vel = (rpm/60) * 2*PI * wheel_radius * 3.6;
    return vel;
  }
}

float get_voltage(){
  int value = analogRead(voltageSensorPin);
  double voltage = value * (64.0 /1023.0);
  v_total -= voltageReadings[readIndex];
  voltageReadings[readIndex] = voltage;
  v_total += voltageReadings[readIndex];
  float volt_avg = v_total/ float(numReadings);
  return volt_avg;
}

float get_current(){
  int value = analogRead(voltageSensorPin);
  double current = (value * (5.0 / 1023.0) - 2.56) / current_senstivity;
  i_total -= currentReadings[readIndex];
  currentReadings[readIndex] = current;
  i_total += currentReadings[readIndex];
  float current_avg = i_total/ float(numReadings);
  return current_avg;  
}

String get_time(){
  rtc.refresh();
  String hour = String(rtc.hour());
  String minute = String(rtc.minute());
  String second = String(rtc.second());
  return hour + ":" + minute + ":" + second;  
}
