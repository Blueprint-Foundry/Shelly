/*
This example shows how to take simple range measurements with the VL53L1X. The
range readings are in units of mm.

History
March 28, 2021a
- try to use polulu libraries to get a single roi, and print continously
- "sensor.startContinuous(50);" function doesn't work with SPAD library, replacing them with Polulu driver functions
- note: Pimoroni Ltd PIM373 VL53L1X tof sensor has address of 0x29  
- just getting zeros when using all polulu libraries :(


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

char device_i2c_address = 0;

void setup()
{
  delay(2000);
  
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  delay(2000);

  sensor.setTimeout(500);
  if (!sensor.init(true))
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  device_i2c_address = sensor.getAddress(); 

  Serial.printf("Device address:  %d \n",device_i2c_address);

  Dev->I2cDevAddr = device_i2c_address; // 0x29 address for the Pimoroni Ltd PIM373 VL53L1X 

  Serial.printf("Device data:  \n");
  checkDev(Dev);

  delay(1000);

  Serial.printf("pre status: %d\n",statusint);

  //statusint = VL53L1_WaitDeviceBooted(Dev);

  //Serial.printf("post VL53L1_WaitDeviceBooted: %d\n",statusint);
  
  statusint = VL53L1_DataInit(Dev);

  Serial.printf("post VL53L1_DataInit: %d\n",statusint);
  
  statusint = VL53L1_StaticInit(Dev);

  Serial.printf("post VL53L1_StaticInit: %d\n",statusint);
  
  statusint = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_SHORT);

  Serial.printf("post VL53L1_SetDistanceMode: %d\n",statusint);

  statusint = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 10000);  // 73Hz

  Serial.printf("post VL53L1_SetMeasurementTimingBudgetMicroSeconds: %d\n",statusint);
    
  statusint = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 15); 

  Serial.printf("post VL53L1_SetInterMeasurementPeriodMilliSeconds: %d\n",statusint);
  
  if (statusint) 
  {
    Serial.printf("StartMeasurement failed status: %d \n",statusint);
  }
  
  VL53L1_StartMeasurement(Dev);

  Serial.printf("happens #2 \n");


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
  //Serial.printf("happens #3 \n");
  
  statusint = VL53L1_SetUserROI(Dev, &roiConfig[0]); // just check one roi

  if(statusint == 0)
  {
    statusint = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
    
  }
  else
  {
    Serial.printf("VL53L1_SetUserROI status: %d \n",statusint);
    
  }
  

  VL53L1_clear_interrupt_and_enable_next_range(Dev, VL53L1_DEVICEMEASUREMENTMODE_SINGLESHOT);

  if(statusint == 0)
  {
    
    matrix_distance[0] = RangingData.RangeMilliMeter;
  }


  if(matrix_distance[0] == 0)
  {
    Serial.printf("bad :( \n");
    delay(2000);
  
  }
  else
  {
  
    Serial.printf("[0]:%d ",matrix_distance[0]);
  }
  
  //if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  //Serial.println();

  //Serial.printf("happens #4 \n");
  
}


void checkDev(VL53L1_DEV Dev) {
  uint16_t wordData;
  VL53L1_RdWord(Dev, 0x010F, &wordData);
  Serial.printf("DevAddr: 0x%X VL53L1X: 0x%X\n\r", Dev->I2cDevAddr, wordData);

}


/* old code
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
  */
