/*
   Version History
   version: May 12, 2021 2:15am
   - these library functions work: https://github.com/pololu/vl53l1x-arduino, ST Library function fail here: https://github.com/pololu/vl53l1x-st-api-arduino/blob/master/vl53l1x-st-api/vl53l1x-st-api.ino
   - going to try setROISize, setROICenter from pololu arduino library with short distance mode, 50ms timing budget, 55ms inter period

 *  * version: May 12, 2021 2:15am
   - previous version works well to get single spad. this version will try to connect to ROS bridge
   -

 *  * version: Calibration Attempt1m - May 29, 2021 2:15am
   - to do: understand Arduino EEPROM library to store calibration values
   -   
*/


#include <Wire.h>

#include <VL53L1X.h>  // from pololu library

//#include <WiFi.h>
//#include <WebServer.h>
//#include <WebSocketsClient.h>
//#include <ArduinoJson.h>
#include <EEPROM.h>

VL53L1X sensor;
/*
  This includes wifi credentials, ensuring they are not checked into a repo

  1. Create a file called "wifi.h" in the same folder
  2. Place the following text in the file:
  #define HOME_SSID "replace with your wifi ssid"
  #define PWD "replace your wifi password"
  #define IP "replace with your laptop's IP"
  3. Save.
*/
//#include "wifi_details.h"

#define MEASUREMENT_BUDGET_MS 15000
#define EEPROM_SIZE 34


int TOF_discovered_flag = 0; // set to 1 when the TOF address 0x29 is discovered
int initialize_once_flag = 0; // used so the TOF sensor is initialized once
int success_flag = 0;

uint8_t ROI_width;
uint8_t  ROI_height;
uint8_t  ROI_center;
float distance_in_mm = 0;

//WebSocketsClient webSocket; // this is a websocket client object
char buffer[800];
//struct timeval tv; //handles setting and getting time

String tx_array_string = "";
String nextdata_string = "";

uint8_t topright_spad_centers[16] = {10, 14, 42, 46, 74, 78, 106, 110, 145, 149, 177, 181, 209, 213, 241, 245};

uint8_t left2right_top2down_spad_centers[16] = {145, 177, 209, 241, 149, 181, 213, 245, 110, 78, 46, 14, 106, 74, 42, 10};
uint16_t left2right_top2down_spad_distances[16];

uint8_t spad_center_index = 0;

void setup()
{

  uint8_t low_byte = 0;
  uint8_t high_byte = 0;
  uint16_t spad_data = 6000; 
  
  uint16_t read_spad_data = 0;
  uint8_t read_lowbyte = 0;
  uint8_t read_highbyte = 0;

  byte error, address;
  int nDevices;

  EEPROM.begin(EEPROM_SIZE);

  Wire.begin(25, 23); //Join I2C bus, set SDA and SCL pins respectively (tested on ESP32, remove pins if using another chip)
  Wire.setClock(400000); //Set speed to 400Khz, chip supports up to 1Mhz

  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      if (address == 41)
      {
        Serial.println("VL53L1X discovered  !");
        TOF_discovered_flag = 1;
      }

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(2000);           // wait 5 seconds for next scan

  // **********
  // after finding the address, initialize the sensor
  // **********

  if (TOF_discovered_flag == 1)
  {
    if (!sensor.init())
    {
      Serial.println("Failed to detect and initialize sensor!");
      success_flag = 0;
    }
    else
    {
      Serial.println("VL53L1X is initialized");

      sensor.setDistanceMode(VL53L1X::Medium);

      // ********
      // set ROIs
      // ********
      sensor.getROISize(&ROI_width, &ROI_height);
      Serial.printf("original width: %d, original height: %d \n", ROI_width, ROI_height);
      ROI_center = sensor.getROICenter();
      Serial.printf("original center: %d \n", ROI_center);

      // the ROi will be 4 x 4, around the center ROI number of 199
      //sensor.setROISize(4, 4);
      //sensor.setROICenter(207);

      // check for change
      sensor.getROISize(&ROI_width, &ROI_height);
      Serial.printf("new width: %d, new height: %d \n", ROI_width, ROI_height);
      ROI_center = sensor.getROICenter();
      Serial.printf("new center: %d \n", ROI_center);

      // ********
      // set timing budget and inter-measurement period
      // ********
      sensor.setMeasurementTimingBudget(MEASUREMENT_BUDGET_MS);
      sensor.startContinuous(MEASUREMENT_BUDGET_MS + 4000);  // intermeasurement period must be timing budget + 4ms at minimum
      success_flag = 1;  // allow for continuous reading

    }


  }

  // **********
  // wifi connection
  // **********
  /*
  WiFi.begin(HOME_SSID, PWD);

  while (WiFi.status() != WL_CONNECTED) // if it's not connected to home wi-fi
  {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  webSocket.begin(IP, 9090, "/");  // on server side, run the begin method within the websocket object
  webSocket.onEvent(webSocketEvent); // if a request happens to the server, trigger this function
  webSocket.setReconnectInterval(1000);
  */

  // EEPROM Setup
  // convert 1000 to 2 bytes, address 0 = MSB, address 1 = LSB


  high_byte = (uint8_t)(spad_data  >> 8);
  low_byte = (uint8_t)(spad_data & 0x00FF);
  
  // writing first spad in address 0 and 1
  EEPROM.write(0,high_byte);
  EEPROM.write(1,low_byte);

  EEPROM.commit();



  read_highbyte = (uint8_t)EEPROM.read(0);
  read_lowbyte = (uint8_t)EEPROM.read(1);

  read_spad_data = (uint16_t)(read_highbyte << 8);

  read_spad_data = (uint16_t)(read_spad_data + read_lowbyte);


  Serial.printf("read_spad_data: %d \n",read_spad_data);


  calibrate_allspads();
  success_flag = 0;

}


void loop()
{




  //webSocket.loop();




  if (success_flag == 1)
  {

    sensor.readSingle(false);  // start single measurement

    while (!sensor.dataReady()) {
      //wait for sensor data
    }

    sensor.setROISize(4, 4);
    sensor.setROICenter(topright_spad_centers[spad_center_index]);


    distance_in_mm = sensor.read(false) / 1000.0;

    //Serial.printf("spad_index: %d, spad center: %d, distance is: %f \n", spad_center_index, topright_spad_centers[spad_center_index], distance_in_mm );

    nextdata_string = String(distance_in_mm);
    tx_array_string += String(distance_in_mm);

    if (spad_center_index != 15)
    {
      tx_array_string += String(",");
    }


    if (sensor.timeoutOccurred())
    {
      Serial.printf(" TIMEOUT \n");
    }



    if (spad_center_index >= 15)
    {
      spad_center_index = 0;

      Serial.printf("transmitting following data string \n");
      Serial.println(tx_array_string);

      //json_Pub_TOF_A["msg"]["data"] = "0";
      //json_Pub_TOF_A["msg"]["data"] = tx_array_string;
      //serializeJsonPretty(json_Pub_TOF_A, Serial);
      //String val = "1000,2000,3000,4000,5000,6000,7000,8000,9000,1000,11000,12000,13000,14000,15000,16000";
      //json_Pub_TOF_A["msg"]["data"]= val;
      //StaticJsonDocument<800> json_Pub_TOF_A;
      //json_Pub_TOF_A["op"] = "publish";
      //json_Pub_TOF_A["id"] = "2";
      //json_Pub_TOF_A["topic"] = "/Shelly/TOF_A";
      //json_Pub_TOF_A["type"] = "std_msgs/String";
      //json_Pub_TOF_A["msg"]["data"] = tx_array_string;
      //webSocket.sendTXT(buffer, serializeJson(json_Pub_TOF_A, buffer));
      //json_Pub_TOF_A["msg"]["data"]="1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16";

      tx_array_string = "";
      nextdata_string = "";
    }
    else
    {
      spad_center_index++;

    }



  }
  else
  {
    Serial.println("Failure :(");
    delay(2000);
  }

}


void calibrate_allspads()
{
   //int total_iterations = 16*50;
   //int i1 = 0;
   //int i2 = 0;

   
   //int spad_center_counter = 0;

   float spad_avg[16]; 
   float spad_current_data[16];   
   float spad_avg_old[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
   int num_of_elements = 1;

/*
   float spad145_avg = 0;
   float spad177_avg = 0;
   float spad209_avg = 0;
   float spad241_avg = 0;   
   float spad149_avg = 0;     
   float spad181_avg = 0;
   float spad213_avg = 0;
   float spad245_avg = 0;
   float spad110_avg = 0;     
   float spad78_avg = 0;    
   float spad46_avg = 0;   
   float spad14_avg = 0;       
   float spad106_avg = 0;     
   float spad74_avg = 0;  
   float spad42_avg = 0;    
   float spad10_avg = 0;    
   */    
   
   

    sensor.readSingle(false);  // start single measurement

    while (!sensor.dataReady()) {
      //wait for sensor data
    }

    sensor.setROISize(4, 4);


   for(int i1 = 0; i1 < 50; i1++)  // take 50 samples of each SPAD to find the average reading of each SPAD
   {
      Serial.printf("sample: %d \n",i1);  
      for(int i2 = 0; i2 < 16; i2++ )  // for each of the 16 SPADS SPAD
      {

         sensor.setROISize(4, 4);
         sensor.setROICenter(left2right_top2down_spad_centers[i2]);  // uint8_t left2right_top2down_spad_centers[16] = {145, 177, 209, 241, 149, 181, 213, 245, 110, 78, 46, 14, 106, 74, 42, 10};
                 
         // get the reading for that SPAD
          sensor.readSingle(false);  // start single measurement
      
          while (!sensor.dataReady()) {
            //wait for sensor data
          }

         spad_current_data[i2] = (float)(sensor.read(false) / 1000.0);  

         // New average = old average * (n-1)/n + new value /n
         spad_avg[i2] = spad_avg_old[i2]*((float)num_of_elements - 1)/(float)num_of_elements + spad_current_data[i2]/(float)num_of_elements;

         spad_avg_old[i2] =  spad_avg[i2];   
      }

        
        
      num_of_elements++;  
    
   }


    for(int i2 = 0; i2 < 16; i2++ )  // for each of the 16 SPADS SPAD
    {

       Serial.printf("spad_avg[%d]: %f \n",i2,spad_avg[i2]);  
    }
   
   // read all SPAD 50 times. For each 4x4 SPAD, take a running average

//uint8_t left2right_top2down_spad_centers[16] = {145, 177, 209, 241, 149, 181, 213, 245, 110, 78, 46, 14, 106, 74, 42, 10};
//uint16_t left2right_top2down_spad_distances[16];

   // save the running average in a calibration array 

   // write the calibration array

   // 

}


/*
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  char *payload_pointer = (char *)payload;

  //Serial.printf("Got Type %i\n", type);

  switch (type) {
    case WStype_ERROR: // type 0
      {
        Serial.println("Websocket Error");
        break;
      }
    case WStype_DISCONNECTED: // type
      {
        Serial.printf("[WSc] Disconnected from url: %s\n", IP);
        break;
      }
    case WStype_CONNECTED: // type 2
      {
        Serial.printf("[WSc] Connected to url: %s\n", IP);

        //SUBSCRIBERS
        // Receives Header from ROS on laptop, used to set time and check connection
        StaticJsonDocument<800> json_Sub_header;
        json_Sub_header["op"] = "subscribe";
        json_Sub_header["id"] = "1";
        json_Sub_header["topic"] = "/Shelly/header_sent";
        json_Sub_header["type"] = "std_msgs/Header";

        //PUBLISHERS
        //Publishes TOF data from Shelly to Laptop
        StaticJsonDocument<800> json_Pub_TOF_A;
        json_Pub_TOF_A["op"] = "advertise";
        json_Pub_TOF_A["id"] = "2";
        json_Pub_TOF_A["topic"] = "/Shelly/TOF_A";
        json_Pub_TOF_A["type"] = "std_msgs/String";

        //Subscribe to stuff
        webSocket.sendTXT(buffer, serializeJson(json_Sub_header, buffer));
        //Advertise Publishers
        webSocket.sendTXT(buffer, serializeJson(json_Pub_TOF_A, buffer));

        break;
      }
    case WStype_TEXT: // type 3
      {
        StaticJsonDocument<800> json_recievedmessage;
        DeserializationError err = deserializeJson(json_recievedmessage, payload_pointer);

        if (err)
        {
          Serial.print("Error: deserializeJson() returned: ");
          Serial.println(err.c_str());
        }
        else
        {
          String topic  = json_recievedmessage["topic"];
          if (topic == "/Shelly/header_sent")
          {
            //serializeJsonPretty(json_recievedmessage, Serial);
            //int seq  = json_recievedmessage["msg"]["seq"];
            //Serial.println(seq);
            gettimeofday(&tv, NULL);
            //            Serial.print ("-----> Was ");
            //            Serial.print (tv.tv_sec);
            //            Serial.print (" ");
            //            Serial.print (tv.tv_usec);
            struct timeval tv_new;
            tv_new.tv_sec = json_recievedmessage["msg"]["stamp"]["secs"];
            tv_new.tv_usec = (int)json_recievedmessage["msg"]["stamp"]["nsecs"] / 1000;
            settimeofday(&tv_new, NULL);
            //            Serial.print ("    Is ");
            //            Serial.print (tv.tv_sec);
            //            Serial.print (" ");
            //            Serial.print (tv.tv_usec);
            //            Serial.println("");
          }
        }
      }
    default:
      break;
  }


}
  */
