#include <Wire.h>
 
#include <VL53L1X.h>  // from pololu library

VL53L1X sensor;

int TOF_discovered_flag = 0; // set to 1 when the TOF address 0x29 is discovered 
int initialize_once_flag = 0; // used so the TOF sensor is initialized once

 
void setup()
{
  Wire.begin(25,23); //Join I2C bus, set SDA and SCL pins respectively (tested on ESP32, remove pins if using another chip)
  Wire.setClock(400000); //Set speed to 400Khz, chip supports up to 1Mhz
  
  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}
 
 
void loop()
{
  byte error, address;
  int nDevices;
 


  if(TOF_discovered_flag == 0)  // find the address for the TOF
  {
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
     
      delay(5000);           // wait 5 seconds for next scan
  }

  // after finding the address, initialize the sensor
  if(initialize_once_flag == 0)
  {
    if (!sensor.init())
    {
      Serial.println("Failed to detect and initialize sensor!");

    }
    else
    {
      Serial.println("VL53L1X is initialized");
    }
    initialize_once_flag = 1;
    
  }

}
