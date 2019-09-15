// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

// software SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);

// Defining a struct for saving roll and pitch
struct orient {
  float roll;
  float pitch;
};

// Define some constants
const float radtoang = 180 / 3.14159265;
const int numaves = 16;
struct orient level;
float total;

void setup(void) {
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Serial.println("LIS3DH test!");
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");
}

struct orient alignments(float accx, float accy, float accz){
  struct orient alignment;
  alignment.roll = asin(accy / accz) * radtoang;
  alignment.pitch = asin(accx / accz) * radtoang;
  return alignment;
}

void loop() {
  lis.read();      // get X Y and Z data at once

  /* Or....get a new sensor event, normalized */ 
  sensors_event_t event; 
  lis.getEvent(&event);
  
  total = 0;
  for (int i = 0; i < numaves; i++){
    level = alignments(event.acceleration.x, event.acceleration.y, event.acceleration.z);
    total += level.pitch;
    delay(50);
  }
  total /= numaves;
  Serial.print(" \tRoll: "); Serial.println(total);
  Serial.println();
 
   
}
