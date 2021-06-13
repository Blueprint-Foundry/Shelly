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
//float distance_in_mm = 0;
uint32_t distance_in_millimeters = 0;

//WebSocketsClient webSocket; // this is a websocket client object
char buffer[800];
//struct timeval tv; //handles setting and getting time

String tx_array_string = "";
String nextdata_string = "";

uint8_t topright_spad_centers[16] = {10, 14, 42, 46, 74, 78, 106, 110, 145, 149, 177, 181, 209, 213, 241, 245};

uint8_t left2right_top2down_spad_centers[16] = {145, 177, 209, 241, 149, 181, 213, 245, 110, 78, 46, 14, 106, 74, 42, 10};
uint16_t left2right_top2down_spad_distances[16];

uint8_t spad_center_index = 0;

int16_t calibrated_offsets[16];

//void calibrate_allspads(void);

void setup()
{




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

      sensor.setDistanceMode(VL53L1X::Short);

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

/*
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
 */

  calibrate_allspads();
  retrieve_EEPROM_data(calibrated_offsets);
  //success_flag = 0;

}


void loop()
{




  //webSocket.loop();




  if (success_flag == 1)
  {

    for(int i2 = 0; i2 < 16; i2++ )  // for each of the 16 SPADS SPAD
    {
  
     sensor.setROISize(4, 4);
     sensor.setROICenter(left2right_top2down_spad_centers[i2]);  // uint8_t left2right_top2down_spad_centers[16] = {145, 177, 209, 241, 149, 181, 213, 245, 110, 78, 46, 14, 106, 74, 42, 10};
         
     // get the reading for that SPAD
      sensor.readSingle(false);  // start single measurement
  
      while (!sensor.dataReady()) 
      {
      //wait for sensor data
      }
  
     distance_in_millimeters = (uint32_t)(sensor.read(false));  // 
  
     Serial.printf("left2right_top2down_spad_centers[%d] = %d, uncalibrated distance is: %d \n",i2, left2right_top2down_spad_centers[i2], distance_in_millimeters);

     distance_in_millimeters = (uint32_t)(((int16_t)sensor.read(false)) + calibrated_offsets[i2]);
     
     Serial.printf("calibrated_offsets[%d] = %d,                 calibrated distance is: %d \n",i2, calibrated_offsets[i2], distance_in_millimeters);

     //distance_in_millimeters = distance_in_millimeters + calibrated_offsets[i2];
     
    }  

    /*
    for(int i2 = 0; i2 < 16; i2++ )  // for each of the 16 SPADS SPAD
    {
  
     sensor.setROISize(4, 4);
     sensor.setROICenter(left2right_top2down_spad_centers[i2]);  // uint8_t left2right_top2down_spad_centers[16] = {145, 177, 209, 241, 149, 181, 213, 245, 110, 78, 46, 14, 106, 74, 42, 10};
         
     // get the reading for that SPAD
      sensor.readSingle(false);  // start single measurement
  
      while (!sensor.dataReady()) 
      {
      //wait for sensor data
      }
  
     distance_in_millimeters = (uint32_t)(((int16_t)sensor.read(false)) + calibrated_offsets);  // 
  
     Serial.printf("left2right_top2down_spad_centers[%d] = %d, calibrated distance is: %d \n",i2, left2right_top2down_spad_centers[i2], distance_in_millimeters);

     
    }  
    */     



/*
    sensor.readSingle(false);  // start single measurement

    while (!sensor.dataReady()) {
      //wait for sensor data
    }

    sensor.setROISize(4, 4);
    sensor.setROICenter(topright_spad_centers[spad_center_index]);


    distance_in_mm = (sensor.read(false) + calibrated_offsets[spad_center_index]);  // actually distance in millimeters

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

    */
    success_flag = 0; // only do once
  }
  else
  {
    Serial.println("Failure :(");
    delay(2000);
  }
  

}


float to_degrees(float rad)
{
  return rad * (180.0 / M_PI);
  
}

float to_rad(float degrees)
{
  return (degrees * M_PI)/180.0;
  
}

void calibrate_allspads()
{


   //float spad_avg[16]; 
   float spad_avg[32]; 
   float spad_current_data[16];   
   //float spad_avg_old[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
   float spad_avg_old[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
   int num_of_elements = 1;
   float calibrated_differences[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // the error for each SPAD location

   uint8_t low_byte = 0;
   uint8_t high_byte = 0;
   uint16_t spad_diff = 0; 
   int16_t negative_value_check = 0;


  int theta = 0;                  // horizontal angle in degrees, positive right
  int phi = 0;                    // vertical angle in degrees, positive down
  
  float flat_distance = 140.0;     // in mm - 15cm = 150mm
  //float L2R_T2B_distances[16];    // left to right, top to bottom SPADs
  float L2R_T2B_distances[32];
  float z = flat_distance;        // axis straight ahead
  float a = 0;                    //distance vector projected on the X-Z plane
  float sub_angle = 27.0/8.0;     
  
  int count = 0;
  int data_count = 0;

  // find the theoretical hypotenuse distances away from flat surface
  Serial.printf("Calculating true distances given %f mm away flat surface \n",flat_distance);
  //Serial.printf("t:%d, p: %d, L2R_T2B_distances[%d] = %f \n",theta,phi,count,L2R_T2B_distances[count]);
  for (theta = -3; theta < 4; theta = theta + 2)
  {
    a = z/cos(to_rad(theta*sub_angle)); 
    
    for(phi = -3; phi < 4; phi = phi+ 2)
    {
      L2R_T2B_distances[count] = a/cos(to_rad(phi*sub_angle));
      
      Serial.printf("t:%d, p: %d, L2R_T2B_distances[%d] = %f \n",theta,phi,count,L2R_T2B_distances[count]);
      count++;  // should only go up to 15
    }
    
  }
  
    Serial.printf("Taking 50 samples of each 16 ROIs to find average reported distance... \n");

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

         spad_current_data[i2] = (float)(sensor.read(false));  // perhaps we shouldn't divide by 1000 if converting to uint16_t later

         // New average = old average * (n-1)/n + new value /n
         spad_avg[i2] = spad_avg_old[i2]*((float)num_of_elements - 1)/(float)num_of_elements + spad_current_data[i2]/(float)num_of_elements;

         spad_avg_old[i2] =  spad_avg[i2];   
      }

        
        
      num_of_elements++;  
    
   }

    Serial.printf("find offsets between average and true distances and save in EEPROM... \n");
    count = 0;
    while(count < 32)
    {
      
      if((count % 2 == 0) && (count >= 0))
      {
         data_count = count/2;
          // error_diff = real - measured (to get back real = measured + error_diff_fromEEPROM)
         negative_value_check = (int16_t)(L2R_T2B_distances[data_count] - spad_avg[data_count]);

         //Serial.printf("spad_avg[%d] = %f \n",data_count,spad_avg[data_count]);
         
         if(negative_value_check < 0)
         {

            calibrated_differences[data_count] = negative_value_check;

            spad_diff = (uint16_t)(-1*negative_value_check) ; 
            high_byte = (uint8_t)(spad_diff  >> 8);
            high_byte =  high_byte | (0x80);              // set MSB to 1 if value is negative
            //low_byte  = (uint8_t)(spad_diff & 0x00FF);

            Serial.printf("spad_avg: %f, real: %f, calibrated_differences[%d]: %f \n",spad_avg[data_count],L2R_T2B_distances[data_count],data_count,calibrated_differences[data_count]);
          //  Serial.printf("negative_value_check: %d ",negative_value_check);
          //  Serial.printf("spad_diff: %d ",spad_diff);
         }
         else
         {

            calibrated_differences[data_count] = negative_value_check;  // ?? set MSB if negative
           
            spad_diff = (uint16_t)negative_value_check ; 
            high_byte = (uint8_t)(spad_diff  >> 8);
            //low_byte  = (uint8_t)(spad_diff & 0x00FF);
           
            Serial.printf("spad_avg: %f, real: %f, calibrated_differences[%d]: %f \n",spad_avg[data_count],L2R_T2B_distances[data_count],data_count,calibrated_differences[data_count]);
         //   Serial.printf("negative_value_check: %d ",negative_value_check);
          //  Serial.printf("spad_diff: %d ",spad_diff);            
         }
          
         
         //Serial.printf("calibrated_differences[%d]: %f \n",count,calibrated_differences[count]);  
  
  
        //Serial.printf("spad_diff = %d \n",spad_diff);

         //   high_byte = (uint8_t)(spad_diff  >> 8);
            //high_byte =  high_byte | (0x80);              // set MSB to 1 if value is negative
            low_byte  = (uint8_t)(spad_diff & 0x00FF);

        EEPROM.write(count,high_byte);
       // Serial.printf("writing high byte: %d \n",high_byte);
        count++;
        EEPROM.write(count,low_byte);  
       //  Serial.printf("writing low byte: %d \n",low_byte);       
        count++;     
                
      } 

      
    }

    EEPROM.commit(); 




}

void retrieve_EEPROM_data(int16_t *data_array_pointer)
{
    uint8_t  read_lowbyte   = 0;
    uint8_t  read_highbyte  = 0;
    uint16_t read_spad_data = 0;

    int negative_true_flag = 0;

    int data_count = 0;
    int count = 0;
    
    while(count < 32)
    {
      if((count % 2 == 0))
      {
        data_count = count/2;
        read_highbyte = (uint8_t)EEPROM.read(count);

 
        if(read_highbyte >= 0x80)
        {
          //Serial.printf("negative read_highbyte: %d || ",read_highbyte);
          negative_true_flag = 1;
          read_highbyte = read_highbyte & ~(0x80);  // remove the 1 set to indicate a negative
          //Serial.printf("new read_highbyte: %d \n ",read_highbyte);         
        }
        else
        {
          negative_true_flag = 0;
        }
        
        count++;
        read_lowbyte  = (uint8_t)EEPROM.read(count); 
        count++;   

        //Serial.printf("lowbyte: %d \n ",read_lowbyte); 
        
        read_spad_data = (uint16_t)(read_highbyte << 8);
        read_spad_data = (uint16_t)(read_spad_data + read_lowbyte);


        //Serial.printf("read_spad_data: %d \n ",read_spad_data); 
 
        if(negative_true_flag == 1)
        {
          //Serial.printf("[%d]read_spad_data: - %d || ",data_count,read_spad_data); 
          data_array_pointer[data_count] = (int16_t)(-1*(int16_t)(read_spad_data));
          //Serial.printf("data_array_pointer[%d] = %d \n",data_count,data_array_pointer[data_count]); 
        }
        else
        {
          //Serial.printf("[%d]read_spad_data: + %d || ",data_count,read_spad_data);
          data_array_pointer[data_count] = (int16_t)(read_spad_data);
          //Serial.printf("data_array_pointer[%d] = %d \n",data_count,data_array_pointer[data_count]); 
          
        }

        Serial.printf("EEPROM readback data[%d]: %d \n",data_count,data_array_pointer[data_count]);        
      }   
    }  

  



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
