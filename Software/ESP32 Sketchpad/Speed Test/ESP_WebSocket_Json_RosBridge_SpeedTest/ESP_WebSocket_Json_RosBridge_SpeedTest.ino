/*
 *  ESP_Websocket_Json_RosBridge_client2 - January 02, 2020 7:37pm
 *  - clean up code
 *  - figure out why connection to python server keep disconnecting (maybe later)
 *  - add json messages
 * 
 * ESP_Websocket_Json_RosBridge_client4 - January 10, 2020 11:18am
 *  - able to advertise a topic on ubuntu ros (port 9090) using following format:
 *    json_AdvertiseExample1["op"] = "advertise";
      json_AdvertiseExample1["id"] = "4";
      json_AdvertiseExample1["topic"] = "topic1";
      json_AdvertiseExample1["type"] = "std_msgs/String";
 *  - new goal, publish string and subscribe to same topic to get back string
 *  - 
 *  - 
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

WebSocketsClient webSocket; // this is a websocket client object 

uint8_t pin_led = 2;                            // pin 2 is LED on Wemos D1 Mini?
char* ssid = "SmartRG-dec9";
char* password = "**********";

int sum = 0;

  char buffer[200];
  StaticJsonDocument<200> json_Advertise_pynoderx;
  StaticJsonDocument<200> json_Subscribe_pynodeshout;
  StaticJsonDocument<200> json_Subscribe_pynodetx;  
  StaticJsonDocument<30200> json_PublishMessage_to_especho;
  StaticJsonDocument<200> json_PublishMessage_to_pynoderx;  
  StaticJsonDocument<200> json_Message1;
  StaticJsonDocument<30200> json_recievedmessage;
  StaticJsonDocument<200> json_Advertise_especho;

  size_t pub_len; 

int first_message = 0;
 
void setup()
{
  //char* string_recieved_data[] = {"1610763800.623323"};

  json_Subscribe_pynodeshout["op"] = "subscribe";
  json_Subscribe_pynodeshout["id"] = "1";
  json_Subscribe_pynodeshout["topic"] = "pynode_shout"; 
  json_Subscribe_pynodeshout["type"] = "std_msgs/String"; 

  json_Subscribe_pynodetx["op"] = "subscribe";
  json_Subscribe_pynodetx["id"] = "2";
  json_Subscribe_pynodetx["topic"] = "pynode_tx"; 
  json_Subscribe_pynodetx["type"] = "std_msgs/String";   

  json_Message1["data"] = "message from arduino";    

  json_PublishMessage_to_especho["op"] = "publish";
  json_PublishMessage_to_especho["id"] = "3";
  json_PublishMessage_to_especho["topic"] = "esp_echo";
  json_PublishMessage_to_especho["msg"] = json_Message1;

  json_Advertise_especho["op"] = "advertise";
  json_Advertise_especho["id"] = "4";
  json_Advertise_especho["topic"] = "esp_echo";
  json_Advertise_especho["type"] = "std_msgs/String";

  json_PublishMessage_to_pynoderx["op"] = "publish";
  json_PublishMessage_to_pynoderx["id"] = "4";
  json_PublishMessage_to_pynoderx["topic"] = "pynode_rx";
  json_PublishMessage_to_pynoderx["msg"] = json_Message1;  

  json_Advertise_pynoderx["op"] = "advertise";
  json_Advertise_pynoderx["id"] = "4";
  json_Advertise_pynoderx["topic"] = "pynode_rx";
  json_Advertise_pynoderx["type"] = "std_msgs/String";
  
  //pub_len = serializeJson(json_PublishMessage_to_Topic1, buffer);
  
  pinMode(pin_led, OUTPUT);

  Serial.begin(115200);                // this is where we initialize the serial object, open a serial connection

  Serial.print("Attempting to connect:");
  
  WiFi.begin(ssid,password); // from the ESP8266WiFi.h class

  while(WiFi.status()!=WL_CONNECTED)  // if it's not connected to home wi-fi
  {
    Serial.print("not connected\n");
    delay(2000);
  }

  
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // from the ESP8266WiFi.h class


  webSocket.begin("192.168.1.4", 9090, "/");  // on server side, run the begin method within the websocket object
  webSocket.onEvent(webSocketEvent); // if a request happens to the server, trigger this function

  // Allocate the JSON document
  //
  // Inside the brackets, 200 is the RAM allocated to this document.
  // Don't forget to change this value to match your requirement.
  // Use arduinojson.org/v6/assistant to compute the capacity.

  toggleLED();
  
}

void loop()
{
  
  webSocket.loop();                           // constantly check for websocket events

  toggleLED();
}

 /*
 typedef enum 
 {
  WStype_ERROR,
  WStype_DISCONNECTED,
  WStype_CONNECTED,
  WStype_TEXT,
  WStype_BIN,
  WStype_FRAGMENT_TEXT_START,
  WStype_FRAGMENT_BIN_START,
  WStype_FRAGMENT,
  WStype_FRAGMENT_FIN,
  WStype_PING,
  WStype_PONG,
 } WStype_t;
 */
       
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

  char *string_sum;
  size_t len;

  int x = (int)length;
  char *payload_pointer = (char *)payload;

  const char* op;
  const char* topic;
  const char* data;

  switch(type) {
    case WStype_DISCONNECTED:
      //Serial.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED: {
      //Serial.printf("[WSc] Connected to url [%d]: %s\n", payload, );
      Serial.printf("[WSc] Connected!\n");

      Serial.printf("json_Advertise_pynoderx: \n");
      serializeJsonPretty(json_Advertise_pynoderx, Serial); 

      size_t len = serializeJson(json_Advertise_pynoderx, buffer);
      webSocket.sendTXT(buffer, len);

      delay(2000);

      Serial.printf("json_Advertise_especho: \n");
      serializeJsonPretty(json_Advertise_especho, Serial); 

      len = serializeJson(json_Advertise_especho, buffer);
      webSocket.sendTXT(buffer, len);      

      delay(2000);
      
      // subscribe to pynode_shout
      Serial.printf("Subscribing to pynode_shout topic with this json:\n");
      serializeJsonPretty(json_Subscribe_pynodeshout, Serial);               // prints json to serial port COM4
     
      len = serializeJson(json_Subscribe_pynodeshout, buffer);               // send json wirelessly to ROSBridge
      webSocket.sendTXT(buffer, len);


      delay(2000);
      
      // subscribe to pynode_tx
      Serial.printf("Subscribing to pynode_tx topic with this json:\n");
      serializeJsonPretty(json_Subscribe_pynodetx, Serial);               // prints json to serial port COM4
     
      len = serializeJson(json_Subscribe_pynodetx, buffer);               // send json wirelessly to ROSBridge
      webSocket.sendTXT(buffer, len);  

      delay(2000);

      Serial.printf("Publishing to pynode_rx topic with this json:\n");
      serializeJsonPretty(json_PublishMessage_to_pynoderx, Serial);     

      pub_len = serializeJson(json_PublishMessage_to_pynoderx, buffer);
      webSocket.sendTXT(buffer, pub_len);
      
    }
      break;
    case WStype_TEXT:  

      // immediately send message1
       

      // Deserialize the JSON document
      /*
      DeserializationError error = deserializeJson(json_recievedmessage, payload);

      
      // Test if parsing succeeds.
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }     
      Serial.printf("recieved this json>");
      Serial.printf(payload);
      Serial.printf("converted Json looks like>");     
      Serial.printf(json_recievedmessage["data"]);
      Serial.printf("<");   
      */       
//      JsonObject& root = jsonBuffer.parseObject(payload);

     // Serial.printf("size of payload>");     
     // Serial.printf("%d",x);
     // Serial.printf("<\n");  

      /*
      for (int i = 0; i <= x; i++) 
      {
        Serial.printf("(%i , %d)",i,payload[i]);
      }
      */

      //Serial.printf(payload_pointer);  

      
      DeserializationError err = deserializeJson(json_recievedmessage, payload_pointer);

      if (err) 
      {
        Serial.print("deserializeJson() returned: ");
        Serial.println(err.c_str());
      }
      else
      {

       //op     = json_recievedmessage["op"];
       //topic  = json_recievedmessage["topic"];
       data   = json_recievedmessage["msg"]["data"];

       Serial.printf("\nrecieved data:");  
       //Serial.printf(op);  
       Serial.printf(data); 
       

       //json_Message1["data"] = json_recievedmessage["msg"]["data"];    

       json_PublishMessage_to_especho["msg"]["data"] = json_recievedmessage["msg"]["data"];
       
       
      }
      
      if(first_message == 0)
      {
        pub_len = serializeJson(json_PublishMessage_to_pynoderx, buffer);
        webSocket.sendTXT(buffer, pub_len);
        first_message = 1;
      }
      else
      {
        pub_len = serializeJson(json_PublishMessage_to_especho, buffer);
        webSocket.sendTXT(buffer, pub_len);        
      }
      

      break;
    
    }

}

void toggleLED()
{
  digitalWrite(pin_led,!digitalRead(pin_led));
  //server.send_P(200,"text/html", webpage);
}
