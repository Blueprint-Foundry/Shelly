#include <Wire.h>
#include <PI4IOE5V96248.h>

void TaskMotors( void *pvParameters );
void TaskBlank( void *pvParameters );

PI4IOE5V96248 io_exp; // Object for communicating with the io expander
const byte PI4IOE5V96248_ADDRESS = 0x23;  // Example PI4IOE5V96248 I2C address, depends on setting for AD0, AD1, AD2

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
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
    ,  0);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

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
    // read the input on analog pin A3:
    int sensorValueA3 = analogRead(A3);
    // print out the value you read:
    Serial.println(sensorValueA3);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}
