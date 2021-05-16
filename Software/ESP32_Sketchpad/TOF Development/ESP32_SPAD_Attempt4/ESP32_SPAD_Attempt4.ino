#include <Wire.h>
 
#include <VL53L1X.h>  // from pololu library

#include "vl53l1_api.h"


// Timing budget set through VL53L1_SetMeasurementTimingBudgetMicroSeconds().
#define MEASUREMENT_BUDGET_MS 50

// Interval between measurements, set through
// VL53L1_SetInterMeasurementPeriodMilliSeconds(). According to the API user
// manual (rev 2), "the minimum inter-measurement period must be longer than the
// timing budget + 4 ms." The STM32Cube example from ST uses 500 ms, but we
// reduce this to 55 ms to allow faster readings.
#define INTER_MEASUREMENT_PERIOD_MS 55

VL53L1X sensor;



int TOF_discovered_flag = 0; // set to 1 when the TOF address 0x29 is discovered 
int initialize_once_flag = 0; // used so the TOF sensor is initialized once
int success_flag = 0;


// SPAD stuff
VL53L1_UserRoi_t ROI_OriginalSettings;
VL53L1_CalibrationData_t CalibrationData;
int status_int, i, x, y, distance[16];
// 16 ROI configurations
VL53L1_UserRoi_t  roiConfig[16];

VL53L1_Dev_t                   dev;
VL53L1_DEV                     Dev = &dev;
//int status_int = 0; 


 
void setup()
{

  byte error, address;
  int nDevices;
  uint8_t byteData = 0;
  uint16_t wordData = 0;

 // This is the default 8-bit slave address (including R/W as the least
 // significant bit) as expected by the API. Note that the Arduino Wire library
 // uses a 7-bit address without the R/W bit instead (0x29 or 0b0101001).  
   Dev->I2cDevAddr = 0x52;
  
  Wire.begin(25,23); //Join I2C bus, set SDA and SCL pins respectively (tested on ESP32, remove pins if using another chip)
  Wire.setClock(400000); //Set speed to 400Khz, chip supports up to 1Mhz
  
  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");

      Serial.println("Scanning...");
    
      nDevices = 0;
      for(address = 1; address < 127; address++ )
      {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
     
        if (error == 0)
        {
          Serial.print("I2C device found at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.print(address,HEX);
          Serial.println("  !");
    
          if(address == 41)
          {
            Serial.println("VL53L1X discovered  !");
            TOF_discovered_flag = 1;
          }
     
          nDevices++;
        }
        else if (error==4)
        {
          Serial.print("Unknown error at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.println(address,HEX);
        }    
      }
      if (nDevices == 0)
        Serial.println("No I2C devices found\n");
      else
        Serial.println("done\n");
     
      delay(2000);           // wait 5 seconds for next scan

  // after finding the address, initialize the sensor
  if(TOF_discovered_flag == 1)
  {
    if (!sensor.init())
    {
      Serial.println("Failed to detect and initialize sensor!");
      success_flag = 0;
    }
    else
    {
      Serial.println("VL53L1X is initialized");

      VL53L1_RdByte(Dev, 0x010F, &byteData);
      Serial.print(F("VL53L1X Model_ID: "));
      Serial.println(byteData, HEX);
      VL53L1_RdByte(Dev, 0x0110, &byteData);
      Serial.print(F("VL53L1X Module_Type: "));
      Serial.println(byteData, HEX);
      VL53L1_RdWord(Dev, 0x010F, &wordData);
      Serial.print(F("VL53L1X: "));
      Serial.println(wordData, HEX);

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

      sensor.setDistanceMode(VL53L1X::Long);
      sensor.setMeasurementTimingBudget(50000);
      sensor.startContinuous(50);
      success_flag = 1;  // allow for continuous reading
      
    }

    
  }
  
}
 
 
void loop()
{


  if(success_flag == 1)
  {
    Serial.print(sensor.read());
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
    Serial.println();    
  }
  else
  {
    Serial.println("Failure :(");
    delay(2000); 
  }

}
