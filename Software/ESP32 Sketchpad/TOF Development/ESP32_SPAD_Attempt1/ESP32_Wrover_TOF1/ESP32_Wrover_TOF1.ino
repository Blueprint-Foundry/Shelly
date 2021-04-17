/* The true ESP32 chip ID is essentially its MAC address.
This sketch provides an alternate chip ID that matches 
the output of the ESP.getChipId() function on ESP8266 
(i.e. a 32-bit integer matching the last 3 bytes of 
the MAC address. This is less unique than the 
MAC address chip ID, but is helpful when you need 
an identifier that can be no more than a 32-bit integer 
(like for switch...case).

created 2020-06-07 by cweinhofer
with help from Cicicok */
	
uint32_t chipId = 0;

#include <Wire.h>
#include "vl53l1_api.h"

// Timing budget set through VL53L1_SetMeasurementTimingBudgetMicroSeconds().
#define MEASUREMENT_BUDGET_MS 50

// Interval between measurements, set through
// VL53L1_SetInterMeasurementPeriodMilliSeconds(). According to the API user
// manual (rev 2), "the minimum inter-measurement period must be longer than the
// timing budget + 4 ms." The STM32Cube example from ST uses 500 ms, but we
// reduce this to 55 ms to allow faster readings.
#define INTER_MEASUREMENT_PERIOD_MS 55

VL53L1_Dev_t                   dev;
VL53L1_DEV                     Dev = &dev;

// SPAD stuff
VL53L1_UserRoi_t ROI_OriginalSettings;
VL53L1_CalibrationData_t CalibrationData;
int status_int, i, x, y, distance[16];
// 16 ROI configurations
VL53L1_UserRoi_t  roiConfig[16];

int status;
int print_delay_flag = 0;
int print_delay_counter = 0;

void setup() {

  uint8_t byteData = 0;
  uint16_t wordData = 0;

  delay(5000);


  Wire.begin(25, 23); //Join I2C bus, set SDA and SCL pins respectively
  Wire.setClock(400000);

  
	Serial.begin(115200);

 // This is the default 8-bit slave address (including R/W as the least
  // significant bit) as expected by the API. Note that the Arduino Wire library
  // uses a 7-bit address without the R/W bit instead (0x29 or 0b0101001).
  Dev->I2cDevAddr = 0x52;

  VL53L1_software_reset(Dev);

  VL53L1_RdByte(Dev, 0x010F, &byteData);
  Serial.print(F("VL53L1X Model_ID: "));
  Serial.println(byteData, HEX);
  VL53L1_RdByte(Dev, 0x0110, &byteData);
  Serial.print(F("VL53L1X Module_Type: "));
  Serial.println(byteData, HEX);
  VL53L1_RdWord(Dev, 0x010F, &wordData);
  Serial.print(F("VL53L1X: "));
  Serial.println(wordData, HEX);

  Serial.println(F("Autonomous Ranging Test"));
  status = VL53L1_WaitDeviceBooted(Dev);               // need to do first
  if (!status) status = VL53L1_DataInit(Dev);          // need to do second
  if (!status) status = VL53L1_StaticInit(Dev);        // need to do third

  // find default ROI and center co-ordinates (debug)

  status_int = VL53L1_GetUserROI(Dev, &ROI_OriginalSettings);
  if (!status_int) // it worked
  {
    Serial.printf("orig top leftX: %d \n", ROI_OriginalSettings.TopLeftX);
    Serial.printf("orig top leftY: %d \n", ROI_OriginalSettings.TopLeftY);
    Serial.printf("orig bot rightX: %d \n", ROI_OriginalSettings.BotRightX);
    Serial.printf("orig bot rightY: %d \n", ROI_OriginalSettings.BotRightY);

  }
  else  // it failed
  {
    Serial.printf("GetUserROI status failure: %d \n", status_int);

  }


  status_int = VL53L1_GetCalibrationData(Dev, &CalibrationData);
  if (!status_int) // it worked
  {
    Serial.printf("optical_centre.x_centre: %d \n", CalibrationData.optical_centre.x_centre);
    Serial.printf("optical_centre.y_centre: %d \n", CalibrationData.optical_centre.y_centre);

  }
  else  // it failed
  {
    Serial.printf("VL53L1_GetCalibrationData status failure: %d \n", status_int);

  }

  // Creating 16 ROI definition
  i = 0;
  for (y = 0; y < 4; y++) {
    for (x = 0; x < 4; x++) {
      roiConfig[i] = {4 * x, (15 - 4 * y), (4 * x + 3), (15 - 4 * y - 3)};
      Serial.printf("rc[%d].TopLeftX = %d, rc.TopLeftY = %d, rc.BotRightX = %d, rc.BotRightY = %d | x = %d, y = %d \n", i, roiConfig[i].TopLeftX, roiConfig[i].TopLeftY, roiConfig[i].BotRightX, roiConfig[i].BotRightY, x, y);
      i++;
    }
  }

  // optional polling driver initiation
  if (!status) status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
  if (!status) status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, (uint32_t)MEASUREMENT_BUDGET_MS * 1000);
  if (!status) status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, INTER_MEASUREMENT_PERIOD_MS);
  if (!status) status = VL53L1_StartMeasurement(Dev);

  if (status)
  {
    Serial.println(F("VL53L1_StartMeasurement failed"));
    while (1);
  }

 
}

void loop() 
{

  static VL53L1_RangingMeasurementData_t RangingData;

  for (i = 0; i < 16; i++)
  {
    // switching ROI configs
    status_int = VL53L1_SetUserROI(Dev, &roiConfig[i]);

    if (status_int)
    {
      Serial.print(F("VL53L1_SetUserROI failed"));
    }

    status_int = VL53L1_WaitMeasurementDataReady(Dev);


    if (status_int)
    {
      Serial.print(F("VL53L1_WaitMeasurementDataReady failed"));
    }


    if (!status_int)
    {
      status_int = VL53L1_GetRangingMeasurementData(Dev, &RangingData);  //4mS
      VL53L1_clear_interrupt_and_enable_next_range(Dev, VL53L1_DEVICEMEASUREMENTMODE_SINGLESHOT); //2mS

      if (status_int)
      {
        Serial.print(F("VL53L1_GetRangingMeasurementData"));
      }
      else  // the ROI SPAD distance was successfully captured for the current SPAD
      {
        distance[i] = RangingData.RangeMilliMeter;
        Serial.printf("d[%i]=%d | ", i, distance[i]);
      }
    }

  }

  Serial.println();


}


void printRangingData()
{
  static VL53L1_RangingMeasurementData_t RangingData;

  status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);         // driver polling mode third step
  if (!status)
  {
    if (print_delay_flag == 0)
    {
      Serial.print(RangingData.RangeStatus);
      Serial.print(F(",??"));
      Serial.print(RangingData.RangeMilliMeter);
      Serial.print(F(","));
      Serial.print(RangingData.SignalRateRtnMegaCps / 65536.0);
      Serial.print(F(","));
      Serial.println(RangingData.AmbientRateRtnMegaCps / 65336.0);
      print_delay_flag = 1;
    }
    else
    {
      print_delay_counter++;
      if (print_delay_counter > 25)
      {
        print_delay_flag = 0;
        print_delay_counter = 0;
      }
    }

  }
}
