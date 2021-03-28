/*
This example shows how to take simple range measurements with the VL53L1X. The
range readings are in units of mm.
*/

#include <Wire.h>
#include <VL53L1X.h>

#include "vl53l1_api.h"
VL53L1X sensor;

VL53L1_Dev_t                   dev;
VL53L1_DEV                     Dev = &dev;

int statusint, i, x, y, distance[16];
int left = 0, right = 0, cnt = 0, oldcnt;
volatile int LightON = 0, OLED_dimmed = 0, OLED_OFF_timeout = 10000;
long timeMark = 0, DisplayUpdateTime = 0;

//VL53L1_UserRoi_t  roiConfig[16]; 



void setup()
{

  i = 0;
  for (y = 0; y < 4; y++) {
    for (x = 0; x < 4; x++) {
      roiConfig[i] = {4*x, (15-4*y), (4*x+3), (15-4*y-3)};
      i++;
    }
  }

}

void loop()
{


}
