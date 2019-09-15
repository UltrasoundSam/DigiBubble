/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
int total;
int numaves = 32;
float average;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);
  
  // Set timing budget to 2 ms
  sensor.setMeasurementTimingBudget(2000);
}

void loop()
{
  
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  total = 0;
  for (int i = 0; i < numaves; i++){
    total += sensor.readRangeSingleMillimeters();
  }
  average = (float)total / numaves;
  Serial.println(average);
}
