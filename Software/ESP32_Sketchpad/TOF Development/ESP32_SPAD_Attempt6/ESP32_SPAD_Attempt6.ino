/*
 * Version History
 * version: May 12, 2021 2:15am
 * - these library functions work: https://github.com/pololu/vl53l1x-arduino, ST Library function fail here: https://github.com/pololu/vl53l1x-st-api-arduino/blob/master/vl53l1x-st-api/vl53l1x-st-api.ino
 * - going to try setROISize, setROICenter from pololu arduino library with short distance mode, 50ms timing budget, 55ms inter period
 */


#include <Wire.h>
 
#include <VL53L1X.h>  // from pololu library

VL53L1X sensor;

#define MEASUREMENT_BUDGET_MS 50
#define INTER_MEASUREMENT_PERIOD_MS 55

int TOF_discovered_flag = 0; // set to 1 when the TOF address 0x29 is discovered 
int initialize_once_flag = 0; // used so the TOF sensor is initialized once
int success_flag = 0;

uint8_t ROI_width;
uint8_t  ROI_height;
uint8_t  ROI_center;
uint16_t distance_in_mm = 0;

void setup()
{

  byte error, address;
  int nDevices;
  
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

      sensor.setDistanceMode(VL53L1X::Short);

     // ********     
     // set ROIs
     // ********
     sensor.getROISize(&ROI_width, &ROI_height);
     Serial.printf("original width: %d, original height: %d \n", ROI_width,ROI_height);
     ROI_center = sensor.getROICenter();
     Serial.printf("original center: %d \n", ROI_center);
     
     // the ROi will be 4 x 4, around the center ROI number of 199
     sensor.setROISize(4, 4);
     sensor.setROICenter(207);

     // check for change
     sensor.getROISize(&ROI_width, &ROI_height);
     Serial.printf("new width: %d, new height: %d \n", ROI_width,ROI_height);
     ROI_center = sensor.getROICenter();
     Serial.printf("new center: %d \n", ROI_center);

     // ********
     // set timing budget and inter-measurement period
     // ********
      sensor.setMeasurementTimingBudget(MEASUREMENT_BUDGET_MS*1000); // in microseconds, but define constant is in milliseconds, so multiply by 1000    
      sensor.startContinuous(MEASUREMENT_BUDGET_MS + 5);  // intermeasurement period must be timing budget + 4ms at minimum
      success_flag = 1;  // allow for continuous reading
      
    }

    
  }
  
}
 
 
void loop()
{


  if(success_flag == 1)
  {
    
    sensor.readSingle(false);  // start single measurement

    delay(200);

    if(sensor.dataReady())
    {
      distance_in_mm = sensor.read(false);     
      Serial.printf("distance is: %d \n", distance_in_mm );
      
    }
    else
    {
       Serial.printf("data wasn't ready \n");
       if (sensor.timeoutOccurred()) 
       { 
        Serial.printf(" TIMEOUT \n"); 
       }
    }
      
  }
  else
  {
    Serial.println("Failure :(");
    delay(2000); 
  }

}
