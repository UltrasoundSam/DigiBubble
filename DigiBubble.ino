// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
#define LIS3DH_CS 10

// Create LIS3DH (orientation) Object
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// Create VL53L0X (distance) object
VL53L0X IRToF;

// Defining a struct for saving roll and pitch
struct orient {
  float roll;
  float pitch;
};

// Define some constants
const float radtoang = 180 / 3.14159265;
const int numaves = 16;
struct orient level;

// Define variables to store values
// Full array of values
int distance[numaves];
float roll[numaves];
float pitch[numaves];

// Running total of values
float tot_dist = 0;
float tot_pitch = 0;
float tot_roll = 0;

// Average values
float ave_pitch;
float ave_roll;
float ave_dist;

// Array to latest pitch, roll and dist values
float outputs[3];
const char *OutputMsg[] = {"Distance: ", "Pitch: ", "Roll: "};

// Loop counter variable
int readIndex = 0;

// Defining constants for tap detection
#define CLICKTHRESHHOLD 80
const byte interruptPin = 2;
volatile int measchoice = 2;

void setup(void) {
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Wire.begin();
  Serial.println("LIS3DH test!");
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!

  // Set up Distance meter
  IRToF.init();
  IRToF.setTimeout(500);
  IRToF.setMeasurementTimingBudget(2000);

  // Set up tap detection
  // 0 = turn off click detection & interrupt
  // 1 = single click only interrupt output
  // 2 = double click only interrupt output, detect single click
  // Adjust threshhold, higher numbers are less sensitive
  lis.setClick(2, CLICKTHRESHHOLD);

  // Attach interrupt to tap
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), changemeasure, RISING);

  // Initialise all measurements to zero
  for (int thisReading = 0; thisReading < numaves; thisReading++) {
    distance[thisReading] = 0;
    roll[thisReading] = 0;
    pitch[thisReading] = 0;
  }
}

// Function for turning data into pitch and roll values
struct orient alignments(float accx, float accy, float accz){
  struct orient alignment;
  alignment.roll = asin(accy / accz) * radtoang;
  alignment.pitch = asin(accx / accz) * radtoang;
  return alignment;
}


// ISR to change what measurement value is displayed to screen
void changemeasure() {
  measchoice++;
  measchoice = measchoice % 3;
}

void loop() {
  lis.read();      // get X Y and Z data at once

  /* Get a new sensor event, normalized */ 
  sensors_event_t event; 
  lis.getEvent(&event);
  level = alignments(event.acceleration.x, event.acceleration.y, event.acceleration.z);
  
  // Update Orientation values
  tot_pitch = tot_pitch - pitch[readIndex];
  pitch[readIndex] = level.pitch;
  tot_pitch = tot_pitch + pitch[readIndex];
  tot_roll = tot_roll - roll[readIndex];
  roll[readIndex] = level.roll;
  tot_roll = tot_roll + roll[readIndex];

  // Re-calculate Averages of orientation
  ave_pitch = tot_pitch / numaves;
  ave_roll = tot_roll / numaves;

  // Update distance measurement
  tot_dist = tot_dist - distance[readIndex];
  distance[readIndex] = IRToF.readRangeSingleMillimeters();
  tot_dist = tot_dist + distance[readIndex];
  ave_dist = tot_dist / numaves;

  // Now advance the counter
  readIndex ++;
  // If we're at end of array, roll back round
  if (readIndex >= numaves) {
    readIndex = 0;
  }
  
  outputs[0] = ave_dist;
  outputs[1] = ave_pitch;
  outputs[2] = ave_roll;

  Serial.print(OutputMsg[measchoice]); Serial.print(outputs[measchoice]);
//  Serial.print(" \tRoll: "); Serial.print(ave_pitch);
//  Serial.print(" \tDist: "); Serial.println(ave_dist);
  Serial.println();
}
