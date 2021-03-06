/*
  Devkit testing software for Shelly
  By: Ilia Baranov
  BLueprint Foundry
  Date: April 30th, 2021
  License: BSD 3 Clause
*/

#include <Wire.h>
#include <PI4IOE5V96248.h>
#include <EAAPMST3923A2.h>
#include <Arduino-MAX17055_Driver.h>
#include <ICM_20948.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <time.h> //TODO: Time is not being kept without a constant header stream... Hmm...

WebSocketsClient webSocket; // this is a websocket client object

/*
  This includes wifi credentials, ensuring they are not checked into a repo

  1. Create a file called "wifi.h" in the same folder
  2. Place the following text in the file:
  #define HOME_SSID "replace with your wifi ssid"
  #define PWD "replace your wifi password"
  #define IP "replace with your laptop's IP"
  3. Save.
*/
#include "wifi_details.h"

// Subscribers and Publishers over Webscokets/Rosbridge/JSON
//Specifications: https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md
char buffer[400];
StaticJsonDocument<400> json_Subscribe_header_sent;
StaticJsonDocument<400> json_Sub_cmd_vel;
StaticJsonDocument<400> json_recievedmessage;
StaticJsonDocument<400> json_Publish_IMU;
StaticJsonDocument<400> json_Publish_IMU_Temp;

void TaskMotors( void *pvParameters );
void TaskBlank( void *pvParameters );

struct timeval tv; //handles setting and getting time

PI4IOE5V96248 io_exp; // Object for communicating with the io expander
const byte PI4IOE5V96248_ADDRESS = 0x23;  // Example PI4IOE5V96248 I2C address, depends on setting for AD0, AD1, AD2
ICM_20948_I2C IMU;
MAX17055 battery;

// the setup function runs once when you press reset or power the board
void setup() {

  SetupJSON();

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  Serial.print("Attempting to connect:");

  WiFi.begin(HOME_SSID, PWD);

  while (WiFi.status() != WL_CONNECTED) // if it's not connected to home wi-fi
  {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  Wire.begin(25, 23); //Join I2C bus, set SDA and SCL pins respectively
  Wire.setClock(400000); //Set speed to 400Khz

  delay(50); //give system chance to stabilize, likely not needed.

  SetupSensors();

  webSocket.begin(IP, 9090, "/");  // on server side, run the begin method within the websocket object
  webSocket.onEvent(webSocketEvent); // if a request happens to the server, trigger this function
  webSocket.setReconnectInterval(1000);

  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskMotors
    ,  "TaskBlink"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL
    ,  1); //Motors has to be on core 1 to work...?

  xTaskCreatePinnedToCore(
    TaskBlank
    ,  "TaskBlank"
    ,  2048  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL
    ,  1);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  webSocket.loop();
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
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
        //Subscribe to stuff
        webSocket.sendTXT(buffer, serializeJson(json_Subscribe_header_sent, buffer));
        webSocket.sendTXT(buffer, serializeJson(json_Sub_cmd_vel, buffer));
        //Advertise Publishers
        //len = serializeJson(json_Publish_IMU, buffer);
        webSocket.sendTXT(buffer, serializeJson(json_Publish_IMU, buffer));
        json_Publish_IMU["op"] = "publish";
        webSocket.sendTXT(buffer, serializeJson(json_Publish_IMU_Temp, buffer));
        json_Publish_IMU_Temp["op"] = "publish";

        break;
      }
    case WStype_TEXT: // type 3
      {
        webSocket.sendTXT(buffer, serializeJson(json_Publish_IMU, buffer)); //TODO: How to send outside of Websocket event??
        webSocket.sendTXT(buffer, serializeJson(json_Publish_IMU_Temp, buffer));

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
          if (topic == "/Shelly/cmd_vel")
          {
            Serial.print(tv.tv_sec);
            Serial.print(" ");
            Serial.print("Shelly Moving: ");
            float cmd_x = json_recievedmessage["msg"]["linear"]["x"];
            float cmd_z = json_recievedmessage["msg"]["angular"]["z"];
            Serial.println(cmd_x);
          }
        }
      }
    default:
      break;
  }

}

void TaskMotors(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  //io_exp.writePort(0, 0xF9); // BIN 1111 1001 configures for full step mode, forward, enabled
  for (;;) // A Task shall never return or exit.
  {
    // Pause the task again for 50ms
    vTaskDelay(500 / portTICK_PERIOD_MS);
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
    }
    else {
      Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    }
    //io_exp.writePort(0, 0xFF);
    //vTaskDelay(50 / portTICK_PERIOD_MS);
    //io_exp.writePort(0, 0xF7);
  }
}

void TaskBlank(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  /*
    AnalogReadSerial
    Reads an analog input on pin A3, prints the result to the serial monitor.
    Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
    Attach the center pin of a potentiometer to pin A3, and the outside pins to +5V and ground.

    This example code is in the public domain.
  */

  for (;;)
  {
    //Serial.println("------> HERE");
    // gettimeofday(&tv_now, NULL);
    //int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    //Serial.print("Time: ");
    //Serial.println(time_us);
    icm_20948_DMP_data_t data;
    IMU.readDMPdataFromFIFO(&data);
    if ((IMU.status == ICM_20948_Stat_Ok) || (IMU.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {

      if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
      {
        // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
        // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
        // The quaternion data is scaled by 2^30.

        //SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

        // Scale to +/- 1
        double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
        double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
        double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
        double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

        //        Serial.print(F("Q1:"));
        //        Serial.print(q1, 3);
        //        Serial.print(F(" Q2:"));
        //        Serial.print(q2, 3);
        //        Serial.print(F(" Q3:"));
        //        Serial.print(q3, 3);
        //        Serial.print(F(" Q0:"));
        //        Serial.print(q0, 3);
        //        Serial.print(F(" Accuracy:"));
        //        Serial.println(data.Quat9.Data.Accuracy);
        json_Publish_IMU["msg"]["orientation"]["x"] = q1;
        json_Publish_IMU["msg"]["orientation"]["y"] = q2;
        json_Publish_IMU["msg"]["orientation"]["z"] = q3;
        json_Publish_IMU["msg"]["orientation"]["w"] = q0;

        //We care most about our Quaternion orientation, so timestamp is taken closest to it
        json_Publish_IMU["msg"]["header"]["stamp"]["secs"] = tv.tv_sec;
        json_Publish_IMU["msg"]["header"]["stamp"]["nsecs"] = tv.tv_usec * 1000;

        IMU.getAGMT();  // TODO: Need to move it somewhere more reasonable
        json_Publish_IMU_Temp["msg"]["header"]["stamp"]["secs"] = tv.tv_sec;
        json_Publish_IMU_Temp["msg"]["header"]["stamp"]["nsecs"] = tv.tv_usec * 1000;
        json_Publish_IMU_Temp["msg"]["temperature"] = IMU.temp();
      }
      if ((data.header & DMP_header_bitmap_Accel) > 0) // Check for Accel
      {
        float acc_x = (float)data.Raw_Accel.Data.X / 107.1; // Extract the raw accelerometer data
        float acc_y = (float)data.Raw_Accel.Data.Y / 107.1; //Seems to use the 8.192 scaling??
        float acc_z = (float)data.Raw_Accel.Data.Z / 107.1; //TODO: Set correct scale? 1g = 9.80665m/s^2

        //Serial.print("Raw: ");
        //Serial.println(acc_z);
        //Serial.println(IMU.agmt.acc.axes.z);
        //Serial.println(IMU.accZ());

        json_Publish_IMU["msg"]["linear_acceleration"]["x"] = acc_x;
        json_Publish_IMU["msg"]["linear_acceleration"]["y"] = acc_y; // m/s^2
        json_Publish_IMU["msg"]["linear_acceleration"]["z"] = acc_z;

        //Serial.print(F("Accel: X:"));
        //        Serial.print(acc_x);
        //        Serial.print(F(" Y:"));
        //        Serial.print(acc_y);
        //        Serial.print(F(" Z:"));
        //        Serial.println(acc_z);
      }
      //      if ((data.header & DMP_header_bitmap_Gyro_Calibr) > 0) // Check for callibrated Gyro data
      //      {
      //        //unsigned char gyroBiasX[4]; // Big-endian
      //        //boolean success = (IMU.readDMPmems(GYRO_BIAS_X, 4, &gyroBiasX[0]) == ICM_20948_Stat_Ok);
      //
      //        float gyro_x = (float)data.Gyro_Calibr.Data.X;
      //        float gyro_y = (float)data.Gyro_Calibr.Data.Y;
      //        float gyro_z = (float)data.Gyro_Calibr.Data.Z;
      //
      //        Serial.print(F("gyroC: X:"));
      //        Serial.print(gyro_x);
      //        Serial.print(F(" Y:"));
      //        Serial.print(gyro_y);
      //        Serial.print(F(" Z:"));
      //        Serial.println(gyro_z);
      //      }
      if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for callibrated Gyro data
      {
        //unsigned char gyroBiasX[4]; // Big-endian
        //boolean success = (IMU.readDMPmems(GYRO_BIAS_X, 4, &gyroBiasX[0]) == ICM_20948_Stat_Ok);

        float gyro_x = (float)data.Raw_Gyro.Data.X / 16.4 * 0.0174533 ;
        float gyro_y = (float)data.Raw_Gyro.Data.Y / 16.4 * 0.0174533;
        float gyro_z = (float)data.Raw_Gyro.Data.Z / 16.4 * 0.0174533;
        float gyro_xb = (float)data.Raw_Gyro.Data.BiasX;
        float gyro_yb = (float)data.Raw_Gyro.Data.BiasY;
        float gyro_zb = (float)data.Raw_Gyro.Data.BiasZ;

        //Serial.print("Raw: ");
        //Serial.println(gyro_z);
        //Serial.println(IMU.gyrZ());

        //Scale: https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/8341270a4a7d08e6b5df95a8c123f6c9378d7268/src/ICM_20948.cpp

        json_Publish_IMU["msg"]["angular_velocity"]["x"] = gyro_x;
        json_Publish_IMU["msg"]["angular_velocity"]["y"] = gyro_y; //rad/s.  1 deg = 0.0174533 rad
        json_Publish_IMU["msg"]["angular_velocity"]["z"] = gyro_z;

        //        Serial.print(F("gyro: X:"));
        //        Serial.print(gyro_x);
        //        Serial.print(F(" Y:"));
        //        Serial.print(gyro_y);
        //        Serial.print(F(" Z:"));
        //        Serial.print(gyro_z);
        //        Serial.print(F(" XB:"));
        //        Serial.print(gyro_xb);
        //        Serial.print(F(" YB:"));
        //        Serial.print(gyro_yb);
        //        Serial.print(F(" ZB:"));
        //        Serial.println(gyro_zb);
      }
    }

    if (IMU.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
    {
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }
}

void SetupSensors() {
  if (!io_exp.begin(PI4IOE5V96248_ADDRESS))
  {
    Serial.println("Failed to init PI4IOE5V96248 :(");
    while (1); //loop forever
  }
  Serial.println("PI4IOE5V96248 found! :)");

  IMU.begin(Wire, 0);
  Serial.print("IMU Status: ");
  Serial.println(IMU.statusString());
  bool IMU_success = true;
  IMU_success &= (IMU.initializeDMP() == ICM_20948_Stat_Ok);
  IMU_success &= (IMU.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok); // enable 9-axis quaternion + heading accuracy
  IMU_success &= (IMU.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok); // enable 16 bit raw accel
  IMU_success &= (IMU.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok); // enable 16bit raw gyro
  //IMU_success &= (IMU.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok); // enable calibrated(?) gryoscope
  IMU_success &= (IMU.setDMPODRrate(DMP_ODR_Reg_Quat9, 10) == ICM_20948_Stat_Ok);
  IMU_success &= (IMU.setDMPODRrate(DMP_ODR_Reg_Accel, 10) == ICM_20948_Stat_Ok);
  IMU_success &= (IMU.setDMPODRrate(DMP_ODR_Reg_Gyro, 10) == ICM_20948_Stat_Ok);
  //IMU_success &= (IMU.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 10) == ICM_20948_Stat_Ok);
  IMU_success &= (IMU.enableFIFO() == ICM_20948_Stat_Ok);
  IMU_success &= (IMU.enableDMP() == ICM_20948_Stat_Ok);
  IMU_success &= (IMU.resetDMP() == ICM_20948_Stat_Ok);
  IMU_success &= (IMU.resetFIFO() == ICM_20948_Stat_Ok);
  if (IMU_success) Serial.println(F("DMP enabled!"));

  //initialization check and settings
  if (!battery.init()) {
    Serial.println("NO MAX17055 detected");
  }
  else {
    Serial.println("MAX17055 initialized :)");
  }

}

void SetupJSON() {

  //SUBSCRIBERS
  // Receives Header from ROS on laptop, used to set time and check connection
  json_Subscribe_header_sent["op"] = "subscribe";
  json_Subscribe_header_sent["id"] = "1";
  json_Subscribe_header_sent["topic"] = "/Shelly/header_sent";
  json_Subscribe_header_sent["type"] = "std_msgs/Header";

  // Receives twist command, used to drive the wheels
  json_Sub_cmd_vel["op"] = "subscribe";
  json_Sub_cmd_vel["id"] = "2";
  json_Sub_cmd_vel["topic"] = "/Shelly/cmd_vel";
  json_Sub_cmd_vel["type"] = "geometry_msgs/Twist";


  //PUBLISHERS
  //Publishes IMU data from Shelly to Laptop
  json_Publish_IMU["op"] = "advertise";
  json_Publish_IMU["id"] = "3";
  json_Publish_IMU["topic"] = "/Shelly/Imu";
  json_Publish_IMU["type"] = "sensor_msgs/Imu";
  json_Publish_IMU["msg"]["header"]["frame_id"] = "/Shelly/Imu";

  //Publishes IMU Temperature data from Shelly to Laptop
  json_Publish_IMU_Temp["op"] = "advertise";
  json_Publish_IMU_Temp["id"] = "4";
  json_Publish_IMU_Temp["topic"] = "/Shelly/Imu/Temperature";
  json_Publish_IMU_Temp["type"] = "sensor_msgs/Temperature";
  json_Publish_IMU_Temp["msg"]["header"]["frame_id"] = "/Shelly/Imu/Temp";
}
