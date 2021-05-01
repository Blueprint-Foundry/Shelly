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
char buffer[200];
StaticJsonDocument<200> json_Subscribe_header_sent;
StaticJsonDocument<200> json_recievedmessage;

void TaskMotors( void *pvParameters );
void TaskBlank( void *pvParameters );

struct timeval tv; //handles setting and getting time

PI4IOE5V96248 io_exp; // Object for communicating with the io expander
const byte PI4IOE5V96248_ADDRESS = 0x23;  // Example PI4IOE5V96248 I2C address, depends on setting for AD0, AD1, AD2

// the setup function runs once when you press reset or power the board
void setup() {

  json_Subscribe_header_sent["op"] = "subscribe";
  json_Subscribe_header_sent["id"] = "1";
  json_Subscribe_header_sent["topic"] = "/Shelly/header_sent";
  json_Subscribe_header_sent["type"] = "std_msgs/Header";

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  Serial.print("Attempting to connect:");

  WiFi.begin(HOME_SSID, PWD);

  while (WiFi.status() != WL_CONNECTED) // if it's not connected to home wi-fi
  {
    Serial.print("not connected\n");
    delay(2000);
  }

  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  Wire.begin(25, 23); //Join I2C bus, set SDA and SCL pins respectively
  Wire.setClock(400000); //Set speed to 400Khz

  delay(50); //give system chance to stabilize, likely not needed.

  if (!io_exp.begin(PI4IOE5V96248_ADDRESS))
  {
    Serial.println("Failed to init PI4IOE5V96248 :(");
    while (1); //loop forever
  }
  Serial.println("PI4IOE5V96248 found! :)");
  byte test[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  io_exp.writeAll(test); //set the entire chip to an array of hex byte values
  //delay(5000);

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
    ,  1024  // Stack size
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
  size_t len;

  Serial.printf("Got Type %i\n", type);

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
        len = serializeJson(json_Subscribe_header_sent, buffer);
        webSocket.sendTXT(buffer, len);
        break;
      }
    case WStype_TEXT: // type 3
      {
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
            serializeJsonPretty(json_recievedmessage, Serial);
            //int seq  = json_recievedmessage["msg"]["seq"];
            //Serial.println(seq);
            gettimeofday(&tv, NULL);
            Serial.print ("-----> Was ");
            Serial.print (tv.tv_sec);
            Serial.print (" ");
            Serial.print (tv.tv_usec);
            struct timeval tv_new;
            tv_new.tv_sec = json_recievedmessage["msg"]["stamp"]["secs"];
            tv_new.tv_usec = (int)json_recievedmessage["msg"]["stamp"]["nsecs"]/1000;
            settimeofday(&tv_new, NULL);
            Serial.print ("    Is ");
            Serial.print (tv.tv_sec);
            Serial.print (" ");
            Serial.print (tv.tv_usec);
            Serial.println("");
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
    //vTaskDelay(50 / portTICK_PERIOD_MS);
    io_exp.writePort(0, 0xFF);
    //vTaskDelay(50 / portTICK_PERIOD_MS);
    io_exp.writePort(0, 0xF7);
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
    // gettimeofday(&tv_now, NULL);
    //int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    //Serial.print("Time: ");
    //Serial.println(time_us);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
