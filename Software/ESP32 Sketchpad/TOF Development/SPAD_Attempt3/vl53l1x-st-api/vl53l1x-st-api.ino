/*******************************************************************************

This sketch file is derived from an example program
(Projects\Multi\Examples\VL53L1X\SimpleRangingExamples\Src\main.c) in the
X-CUBE-53L1A1 Long Distance Ranging sensor software expansion for STM32Cube
from ST, available here:

http://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32cube-expansion-software/stm32-ode-sense-sw/x-cube-53l1a1.html

The rest of the files in this sketch are from the STSW-IMG007 VL53L1X API from
ST, available here:

http://www.st.com/content/st_com/en/products/embedded-software/proximity-sensors-software/stsw-img007.html

********************************************************************************

COPYRIGHT(c) 2017 STMicroelectronics
COPYRIGHT(c) 2018 Pololu Corporation

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the name of STMicroelectronics nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 Saad Edits April 2, 2021
 1. The polulu example that connects to the sensor using the "!sensor.init()" function doesn't crash, but the "Dev->I2cDevAddr = 0x52" code does.
    Therefore...I'll put the "sensor.init()" code and see if it works 
 2. The above fix didn't work, what did work was adding a 5 second delay to the start of the code to let the TOF sensor wake up before ESP8266 taled to it

 Saad Edits April 3, 2021
 1. Will use the functions and structures of "VL53L1_GetUserROI()" and "VL53L1_GetCalibrationData()" to find the default ROI and center X,Y SPADs  
 


*******************************************************************************/

#include <Wire.h>
#include "vl53l1_api.h"

// By default, this example blocks while waiting for sensor data to be ready.
// Comment out this line to poll for data ready in a non-blocking way instead.
#define USE_BLOCKING_LOOP

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
int status_int;


int status;
int print_delay_flag = 0;
int print_delay_counter = 0;

void setup()
{
  uint8_t byteData = 0;
  uint16_t wordData = 0;

  delay(5000);

  Wire.begin();
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
  if(!status) status = VL53L1_DataInit(Dev);           // need to do second
  if(!status) status = VL53L1_StaticInit(Dev);         // need to do third

  // find default ROI and center co-ordinates (debug)

  status_int = VL53L1_GetUserROI(Dev, &ROI_OriginalSettings);
  if(!status_int) // it worked
  {
    Serial.printf("orig top leftX: %d \n",ROI_OriginalSettings.TopLeftX);
    Serial.printf("orig top leftY: %d \n",ROI_OriginalSettings.TopLeftY);
    Serial.printf("orig bot rightX: %d \n",ROI_OriginalSettings.BotRightX);
    Serial.printf("orig bot rightY: %d \n",ROI_OriginalSettings.BotRightY);
    
  }
  else  // it failed
  {
    Serial.printf("GetUserROI status failure: %d \n",status_int);
    
  }


  status_int = VL53L1_GetCalibrationData(Dev, &CalibrationData);
  if(!status_int) // it worked
  {
    Serial.printf("optical_centre.x_centre: %d \n", CalibrationData.optical_centre.x_centre);
    Serial.printf("optical_centre.y_centre: %d \n", CalibrationData.optical_centre.y_centre);
    
  }
  else  // it failed
  {
    Serial.printf("VL53L1_GetCalibrationData status failure: %d \n",status_int);
    
  }  

  // optional polling driver initiation
  if(!status) status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
  if(!status) status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, (uint32_t)MEASUREMENT_BUDGET_MS * 1000);
  if(!status) status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, INTER_MEASUREMENT_PERIOD_MS);
  if(!status) status = VL53L1_StartMeasurement(Dev);

  if(status)
  {
    Serial.println(F("VL53L1_StartMeasurement failed"));
    while(1);
  }
}

void loop()
{
#ifdef USE_BLOCKING_LOOP  // this is true

  // blocking wait for data ready
  status = VL53L1_WaitMeasurementDataReady(Dev);         // driver polling mode first step

  if(!status)
  {
    printRangingData();
    VL53L1_ClearInterruptAndStartMeasurement(Dev);       // error handling
  }
  else
  {
    Serial.print(F("Error waiting for data ready: "));
    Serial.println(status);
  }

#else

  static uint16_t startMs = millis();
  uint8_t isReady;

  // non-blocking check for data ready
  status = VL53L1_GetMeasurementDataReady(Dev, &isReady);   // driver polling mode second step

  if(!status)
  {
    if(isReady)
    {

      printRangingData();

      VL53L1_ClearInterruptAndStartMeasurement(Dev);       // driver polling third step
      startMs = millis();
    }
    else if((uint16_t)(millis() - startMs) > VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS)
    {
      Serial.print(F("Timeout waiting for data ready."));
      VL53L1_ClearInterruptAndStartMeasurement(Dev);
      startMs = millis();
    }
  }
  else
  {
    Serial.print(F("Error getting data ready: "));
    Serial.println(status);
  }

  // Optional polling delay; should be smaller than INTER_MEASUREMENT_PERIOD_MS,
  // and MUST be smaller than VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS
  delay(10);

#endif
}

void printRangingData()
{
  static VL53L1_RangingMeasurementData_t RangingData;

  status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);         // driver polling mode third step
  if(!status)
  {
          if(print_delay_flag == 0)
          { 
              Serial.print(RangingData.RangeStatus);
              Serial.print(F(","));
              Serial.print(RangingData.RangeMilliMeter);
              Serial.print(F(","));
              Serial.print(RangingData.SignalRateRtnMegaCps/65536.0);
              Serial.print(F(","));
              Serial.println(RangingData.AmbientRateRtnMegaCps/65336.0);
              print_delay_flag = 1;
          }
          else
          {
              print_delay_counter++;
              if(print_delay_counter > 25)
              {
                print_delay_flag = 0;
                print_delay_counter = 0;
              }
          }
          
  }
}
