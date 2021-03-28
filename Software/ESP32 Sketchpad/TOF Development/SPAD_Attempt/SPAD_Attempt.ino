/*
This example shows how to take simple range measurements with the VL53L1X. The
range readings are in units of mm.

History
March 28, 2021
- try to use polulu libraries to get a single roi, and print continously


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

VL53L1_UserRoi_t  roiConfig[16]; 
static VL53L1_RangingMeasurementData_t RangingData;


int matrix_distance[16];

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);
}

void loop()
{
  statusint = VL53L1_SetUserROI(Dev, &roiConfig[0]); // just check one roi

  if(statusint == 0)
  {
    statusint = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
    
  }

  VL53L1_clear_interrupt_and_enable_next_range(Dev, VL53L1_DEVICEMEASUREMENTMODE_SINGLESHOT);

  if(statusint == 0)
  {
    
    matrix_distance[0] = RangingData.RangeMilliMeter;
  }

  
  Serial.print(matrix_distance[0]);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
}
