/* 
 HotWaterController program.
 Reads some analog sensors and controls a circulating pump to circulate water through solar collectors to efficiently heat a tank of water.
 A suite of Dallas Onewire sensors provide monitoring of other temperatures in the system.  
 We are using FreeRTOS to allow different tasks to run simultaneously.
 There is :
 A sensor reading task.
 A TFT display print task.
 A pump control task.
 A pump speed calculation task.
 A cloud update task.
*/
#include <FreeRTOS_SAMD51.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <MQTT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_PCF8591.h>
#include "Adafruit_GFX.h"
#include "Adafruit_HX8357.h"

// Change the defines in this file to match your local settings!
#include "LocalSettings.h"

// OneWire bus is on pin 4
#define ONE_WIRE_BUS 4

// Reset the wifi module using pin 7
#define WIFI_RST 7

// PyPortal Titano TFT control
#define TFT_BACKLIGHT 25 // Set the backlight power
#define TFT_D0 34        // Data bit 0 pin (MUST be on PORT byte boundary)
#define TFT_WR 26        // Write-strobe pin (CCL-inverted timer output)
#define TFT_DC 10        // Data/command pin
#define TFT_CS 11        // Chip-select pin
#define TFT_RST 24       // Reset pin
#define TFT_RD 9         // Read-strobe pin
#define TFT_TE 12
Adafruit_HX8357 tft = Adafruit_HX8357(tft8bitbus, TFT_D0, TFT_WR, TFT_DC, TFT_CS, TFT_RST, TFT_RD);

// Mutex around print to avoid two threads trying to print at the same time.
SemaphoreHandle_t printMutex = xSemaphoreCreateMutex();
// Mutex around the ADC to avoid two threads accessing the PCF8591 chip at the same time.
SemaphoreHandle_t adcMutex = xSemaphoreCreateMutex();
// Mutex around the pumpSpeed variable to avoid the calculation and the pump drive threads from accessing the variable at the same time.
SemaphoreHandle_t speedMutex = xSemaphoreCreateMutex();

// Addesses of the oneWire sensors we are using.
// On bootup the program scans the bus and outputs these addresses on the serial port.
DeviceAddress owRoms[][8] = {
            {0x28, 0xF7, 0xE2, 0x81, 0x94, 0x21, 0x06, 0x88 },
            {0x28, 0xAB, 0x83, 0x7F, 0x94, 0x21, 0x06, 0x63 },
            {0x28, 0xEE, 0x1D, 0xA2, 0x94, 0x21, 0x06, 0x74 },
            {0x28, 0x62, 0xC7, 0x7F, 0x94, 0x21, 0x06, 0x9C },
            {0x28, 0x7A, 0x5B, 0x85, 0x94, 0x21, 0x06, 0x9A },
            };

// Sensor names.
// Ensure that the order of these names matches the owRom addresses above.
// Extra names are added on the end for the analog sensors and other bits of info.
char sensorNames[][40] = {"Ambient_Temperature",
              "Outlet_Temperature",
              "Inlet_Temperature",
              "Panel_Inlet_Temperature",
              "Panel_Outlet_Temperature",
              "coldSensor",
              "hotSensor",
              "RSSI",
              "pumpSpeed",
              "wifiConnects",
              "MQTTConnects",
              "maxDiff"};
              
// The combined data from all sensors above is stored in this array.
float sensorData[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
// Pump speed from 0 to 100 percent.
uint8_t pumpSpeed = 0;
// Wifi stats.
uint8_t wifiConnects = 0;
uint8_t MQTTConnects = 0;


// Set up networking.
WiFiClient net;
MQTTClient client(500);

// Set up the A/D board
// This is actually a generic PCF8951 board I got off ebay running over I2C.
Adafruit_PCF8591 pcf = Adafruit_PCF8591();
#define ADC_REFERENCE_VOLTAGE 3.3

// A few buffers.
char stringBuffer[500];
char ptrTaskList[250];


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);


static void setupTFT() {
  // Set up the display. Print these names (that match the order in the sensorData array) down the screen on the left, the update routine does the corresponding values a little to the right.
  // Note that DMA is turned off in Adafruit_SPITFT.h, it doesn't play well with FreeRTOS it seems.
  
  String screenNames[] = {"Ambient","Tank Outlet","Tank Inlet","Collector In","Collector Out","Cold Sensor","Collector Top","RSSI","Pump Drive","Wifi Connects","MQTT Connects", "Max Diff"};
  // Turn on the backlight and rest the TFT.
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH);
  delay(10);
  digitalWrite(TFT_RST, LOW);
  delay(10);
  digitalWrite(TFT_RST, HIGH);
  delay(10);
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(0x0000);
  tft.setTextSize(2);
  tft.setTextColor(HX8357_GREEN);
  tft.setTextWrap(true);
  // Loop through the screen names and print them down the side of the screen.
  for (int i=0;i<12;i++) {
   tft.setCursor(0,i*20);
   tft.print(screenNames[i]);
  }
}

float sensorGet(char* wantedName) {
 // A simple helper function to return the value given by the name
 // Just like a dictionary would.
 for(int i = 0; i<13; i++) {
  if (strcmp(sensorNames[i],wantedName)==0) return sensorData[i];
 }
 // Return nonsense value if not in the list.
  return -255;
}

void sensorSet(char* sensorName, float sensorValue) {
 // Sets a value by it's name, like a dictionary. 
 for(int i = 0; i<13; i++) {
  if (strcmp(sensorNames[i],sensorName)==0) {
    sensorData[i] = sensorValue;
  }
 }
}

float smooth(float smoothValue, float rawValue) {
  // returns a smoothed value by shifting the smoothed value 20 percent of the difference
  // between raw and smooth.
  // Initial load if smoothValue is zero
  if (smoothValue == 0.0) return rawValue * 0.95;
  float diff = rawValue - smoothValue;
  return smoothValue + (diff * 0.20);
}

// functions around Serial.println and print to avoid two threads talking at the same time.
void Println(const char *str) { 
  // use instead of Println(char *str)
  xSemaphoreTake(printMutex, portMAX_DELAY);
  Serial.println(str);
  xSemaphoreGive(printMutex);
}

void Print(const char *str) { // use instead of Serial.println(char *str)
  xSemaphoreTake(printMutex, portMAX_DELAY);
  Serial.print(str);
  xSemaphoreGive(printMutex);
}

static float lookupTemp(int rawValue) {
  // Converts the raw ADC value from the analog NTC thermistors to a temperature value.
  // On my board a 1k resistor pulls the ADC input towards VCC and their internal resistance pulls the ADC input towards ground.
  // Loss of connection gives therefore gives high ADC values which relate to temps below zero.
  // Uses interpolation between 17 lookup values. Didn't bother with negatives, it's not that cold where I live.
  // Put your own function here for your particular sensor, all I had was a table of resistances and temperatures to work with from the original manufacturer.
  float temp = -255;
  float tempsArray[] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160};
  float rawValueArray[] = {243.0,238.0,231.0,222.0,210.0,197.0,181.0,164.0,147.0,125.0,106.0,89.0,73.0,48.0,38.0};
  if (rawValue <= rawValueArray[14]) return 255;
  if (rawValue >= rawValueArray[0]) return -255;
 
  for (int i=0; i<15; i++) {
    if (rawValue > rawValueArray[i]) {
      float perc = (rawValue - rawValueArray[i])/ (rawValueArray[i] - rawValueArray[i-1]);
      temp = perc * (tempsArray[i] - tempsArray[i-1]);
      temp = temp + tempsArray[i];
      break;  
    }
  }
   return temp;
}

static void connectWiFi() {
   // Resets the ESP32 daughterboard and restarts wifi.
   while ( WiFi.status()  != WL_CONNECTED || WiFi.RSSI() == 0)
  {
    WiFi.end();
    Println("WiFi resetting and connecting...");
    pinMode(WIFI_RST, OUTPUT);
    digitalWrite(WIFI_RST, HIGH);
    delay(100);
    digitalWrite(WIFI_RST, LOW);
    delay(100);
    digitalWrite(WIFI_RST, HIGH);
    delay(1000); 
    wifiConnects ++;
    WiFi.begin(MY_SSID,WIFI_PASSWORD);
    delay(5000);
   }
   Println("WiFi Connected");
}

static void connectThingsBoard() {
   // Connects to my MQTT server.
   // Use your own API key and server :-P
   Print("\nConnecting ThingsBoard...");
   while (WiFi.ping(MQTT_SERVER) < 0) {
     Print("Ping check failed, reconnecting wifi.");
     connectWiFi();
   }
   Println("Ping check good");
   while (!client.connected()) {
      MQTTConnects++;
      Print("ThingsBoard connection init...");
      client.begin(MQTT_SERVER, net);
      client.connect("ClientID",APIKEY);
      delay(5000);    
   }
   Println("\nThingsBoard connected.");
}

static void updateCloud(void* pvParameters)
{
  // Sends data to the MQTT server every 5 seconds.
  // It's my server, there's no rate limites.
  // Adjust the taskDelay accordingly for your server.
  char tempBuf[30];
  while(1) { 
    vTaskDelay(5000/portTICK_PERIOD_MS);
    if (!net.connected() || WiFi.RSSI() == 0) {
      connectWiFi();
    }
    if (!client.connected()) {
      connectThingsBoard();
    }
    Print("Pushing data...");
    strcpy(stringBuffer,"{");
    for(int i=0;i<sizeof(sensorData)/sizeof(sensorData[0]);i++) {
      // Stick the comma in only after the first one.
      if (i != 0) strcat(stringBuffer,",");
      sprintf(tempBuf,"\"%s\": %3.2f",sensorNames[i],sensorData[i]);
      strcat(stringBuffer, tempBuf);
    }
    strcat(stringBuffer,"}");
    //Println(stringBuffer);
    client.publish("v1/devices/me/telemetry", stringBuffer);
    Println("Pushed!");
  }
}

static void readSensors(void* pvParameters)
{
  // Read the onewire and analog sensors every two seconds.
  char buffer[40];
  DeviceAddress myDevice;
  while(1) { 
    Print("Read Sensors...");
    // Print("Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    Print("Temps requested....");
    vTaskDelay(800/portTICK_PERIOD_MS);
    for (int i=0; i< sizeof(owRoms)/sizeof(owRoms[0]);i++) {
      float tempC = sensors.getTempC(*owRoms[i]);
      if (tempC == DEVICE_DISCONNECTED_C) {
          continue;
        }
      sensorData[i] = tempC;
    }
    Print("Looking up analog values....");
    // Fill in the analog values. Mutex around all the pcf commands as we're driving the output concurrently.
    xSemaphoreTake(adcMutex, portMAX_DELAY);
    sensorData[5] = smooth(sensorData[5],lookupTemp(pcf.analogRead(0)));
    sensorData[6] = smooth(sensorData[6],lookupTemp(pcf.analogRead(1)));
    xSemaphoreGive(adcMutex);
    sensorData[7] =  WiFi.RSSI();
    sensorData[8] = pumpSpeed;
    sensorData[9] = wifiConnects;
    sensorData[10] = MQTTConnects;
    Println("all read!");
    vTaskDelay(2000/portTICK_PERIOD_MS);
  }
}

static void drivePump(void* pvParameters)
{
  // Drive the pump at a rate consistent with the global pumpSpeed.
  // 
  const float PWM_TIME = 5;
  char tempBuf[10];
  
  // Just switch the DAC 0/255 to correspond with the proportional on/off time.
  while(1)
  { 
    Print(" Drive Pump: ");
    // work out on and off times
    xSemaphoreTake(speedMutex, portMAX_DELAY);
    uint32_t offTime = (100.0-pumpSpeed)/100.0 * PWM_TIME * 1000;
    uint32_t onTime = (pumpSpeed/100.0) * PWM_TIME * 1000;
    xSemaphoreGive(speedMutex);
    itoa(onTime,tempBuf, 10);
    Print(tempBuf);
    Print(" ");
    itoa(offTime,tempBuf, 10);
    Println(tempBuf);
    // Mutexs around PCF access as we're reading concurrently
    xSemaphoreTake(adcMutex, portMAX_DELAY);
    pcf.analogWrite(255);
    xSemaphoreGive(adcMutex);
    vTaskDelay(onTime/portTICK_PERIOD_MS);   
    xSemaphoreTake(adcMutex, portMAX_DELAY);
    pcf.analogWrite(0);
    xSemaphoreGive(adcMutex);
    vTaskDelay(offTime/portTICK_PERIOD_MS);
  }
 
}

static void calcPumpSpeed(void* pvParameters) {
 // Look at sensor data, calculate pump speed.
 // In general:
 // Switch on when hotSensor - coldSensor > START_MAX_DIFF
 // Switch off when hotSensor - coldSensor < START_MIN_DIFF
 // ramp from 20 to 100 percent in between 4 and 8.
 // If we are above initial maxDiff, increase maxDiff for this run to throttle back the pump drive and get higher collector temps.
 // 
 bool pumpTriggered = false;
 bool overTemp = false;
 float maxDiff = START_MAX_DIFF;
 float minDiff = START_MIN_DIFF;
 
 while(1) {
 Println("CalcPumpSpeed");
  float tempDiff = sensorGet("hotSensor")-sensorGet("coldSensor");
  if (tempDiff > maxDiff) pumpTriggered = true;
  // If things are proper hot turn the pump on regardless.
  if (sensorGet("hotSensor") > 100.0) pumpTriggered = true;
  
  if (tempDiff < minDiff) {
    pumpTriggered = false;
    maxDiff = START_MAX_DIFF;
    pumpSpeed = 0;
  }
  
  if (pumpTriggered == true) {
    // Wrap pumpspeed in a mutex so we don't read it in DrivePump while setting it.
    xSemaphoreTake(speedMutex, portMAX_DELAY);
    if (tempDiff > maxDiff) {
      // Flat out.
      pumpSpeed = 100;
      // Slowly increase maxDiff while we are still above it.
      // This should throttle back the pump and increase collector output temp
      maxDiff = maxDiff * 1.01;
    } else {
      pumpSpeed = ((tempDiff-minDiff) / (maxDiff-minDiff)) * 100;
      // Set limits to stop small pulses, don't bother below 10 percent either ( PWM of 5 seconds gives 0.5 sec on time at 10%)
      if (pumpSpeed > 90) pumpSpeed = 100;
      // If pumpSpeed drops below 40 percent start winding down maxDiff again, a bit quicker than we wound up.
      if (pumpSpeed < 40) maxDiff = maxDiff * 0.98;
      if (pumpSpeed < 10) pumpSpeed = 0;
    }
    xSemaphoreGive(speedMutex);
  }

  // Overtemp cool-down mode.
  // Triggered when cold sensor gets over 70 degrees at any time.
  // Wait until hot sensor is more than 30 degrees below cold sensor and turn pump on.
  // Once cold sensor gets down to 60 degrees turn pump off and reset overTemp.
  if (sensorGet("coldSensor") > 70.0) overTemp = true;
  if (overTemp == true) {
    if (sensorGet("coldSensor") - sensorGet("hotSensor") > 30) {
      pumpSpeed = 100;
    }
    if (sensorGet("coldSensor") < 60.0) {
      pumpSpeed = 0;
      overTemp = false;
    }
  }
  sensorData[11] = maxDiff;
  vTaskDelay(2000/portTICK_PERIOD_MS);
 }
}

static void updateScreen(void* pvParameters) {
  // Updates the screen with all the values in the sensorData array every two seconds.
  while(1) {
  xSemaphoreTake(adcMutex, portMAX_DELAY);  
  for (int i=0;i<12;i++) {
  tft.setCursor(200, i*20);
  tft.fillRect(200,i*20,90,i*20+20,0x0000);
  tft.print(sensorGet(sensorNames[i]));
  }
  xSemaphoreGive(adcMutex);
  vTaskDelay(2000/portTICK_PERIOD_MS);
  }


}

void findDevices() {
// Finds oneWire devices. Runs once on boot.
  uint8_t address[8];
  uint8_t count = 0;
  if (oneWire.search(address))
  {
    Serial.println("Devices found:");
    do {
      count++;
      Println("  {");
      for (uint8_t i = 0; i < 8; i++)
      {
        Print("0x");
        if (address[i] < 0x10) Print("0");
        Serial.print(address[i], HEX);
        if (i < 7) Print(", ");
      }
      Serial.println("  },");
    } while (oneWire.search(address));

    Serial.println("");
    Serial.print("// nr devices found: ");
    Serial.println(count);
  }
}

void setup() {
 // Uncomment this if you want to debug and catch the first few lines of output, but the USB serial is pretty quick to connect after flashing anyway
 //while(!Serial);
 // Kick start the I/O board
 pcf.begin();
 pcf.enableDAC(true);

 Print("Setting up display...");
 // Set up the display
 setupTFT();
 Println("Done");
 
 // Set up MQTT and wifi
 connectWiFi();
 connectThingsBoard();
 // Set up Onewire sensors
 findDevices();
 sensors.begin();
 // We'll wait ourselves for the result.
 sensors.setWaitForConversion(false);

 // Create tasks and allocate 512 bytes of stack space to each one.
 // Each task has the same priority.
 xTaskCreate(updateCloud, "MQTT", 512, NULL, 1, NULL);
 xTaskCreate(readSensors, "readData", 512, NULL,1, NULL);
 xTaskCreate(drivePump, "doPump", 512, NULL, 1, NULL);
 xTaskCreate(calcPumpSpeed, "pumpSpeed", 512, NULL, 1, NULL);
 xTaskCreate(updateScreen, "updateScreen", 512, NULL, 1, NULL);
 // Let's get this party started.
 vTaskStartScheduler();
}

void loop() {
  // This just prints the available stack space in the FreeRTOS idle task (loop in arduino)
  // If stack space gets to zero things will crash. The TaskCreate settings above are fine.
  // There's plenty of ram to go around in the PyPortal Titano

    Println("Looping in the Idle Task.");

    vTaskList(ptrTaskList);
    Println("*******************************************");
    Println("Task         State   Prio    Stack    Num"); 
    Println("*******************************************");
    Print(ptrTaskList);
    Println("*******************************************");
    
    delay(10000);

}
