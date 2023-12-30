#include <FreeRTOS_SAMD51.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <MQTT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_PCF8591.h>
#include "Adafruit_GFX.h"
#include "Adafruit_HX8357.h"


#define ONE_WIRE_BUS 4
#define WIFI_RST 7
#define TFT_BACKLIGHT 25 // Set the backlight power
#define TFT_D0 34        // Data bit 0 pin (MUST be on PORT byte boundary)
#define TFT_WR 26        // Write-strobe pin (CCL-inverted timer output)
#define TFT_DC 10        // Data/command pin
#define TFT_CS 11        // Chip-select pin
#define TFT_RST 24       // Reset pin
#define TFT_RD 9         // Read-strobe pin
#define TFT_TE 12

#define START_MAX_DIFF 8;
#define START_MIN_DIFF 4;

// PyPortal Titano
Adafruit_HX8357 tft = Adafruit_HX8357(tft8bitbus, TFT_D0, TFT_WR, TFT_DC, TFT_CS, TFT_RST, TFT_RD);


SemaphoreHandle_t printMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t adcMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t speedMutex = xSemaphoreCreateMutex();

 
DeviceAddress owRoms[][8] = {
            {0x28, 0xF7, 0xE2, 0x81, 0x94, 0x21, 0x06, 0x88 },
            {0x28, 0xAB, 0x83, 0x7F, 0x94, 0x21, 0x06, 0x63 },
            {0x28, 0xEE, 0x1D, 0xA2, 0x94, 0x21, 0x06, 0x74 },
            {0x28, 0x62, 0xC7, 0x7F, 0x94, 0x21, 0x06, 0x9C },
            {0x28, 0x7A, 0x5B, 0x85, 0x94, 0x21, 0x06, 0x9A },
            };
 
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

float sensorData[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t wifiConnects = 0;
uint8_t MQTTConnects = 0;

WiFiClient net;
MQTTClient client(500);

// Set up the A/D board
Adafruit_PCF8591 pcf = Adafruit_PCF8591();
#define ADC_REFERENCE_VOLTAGE 3.3

// A few buffers.
char stringBuffer[500];
char ptrTaskList[250];
uint8_t pumpSpeed = 0;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);


static void setupTFT() {
  String screenNames[] = {"Ambient","Tank Outlet","Tank Inlet","Collector In","Collector Out","Cold Sensor","Collector Top","RSSI","Pump Drive","Wifi Connects","MQTT Connects", "Max Diff"};
 // Note that DMA is turned off in Adafruit_SPITFT.h, it doesn't play well with FreeRTOS it seems.
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
  for (int i=0;i<12;i++) {
   tft.setCursor(0,i*20);
   tft.print(screenNames[i]);
  }
}

float sensorGet(char* wantedName) {
 // Return the value given by the name
 for(int i = 0; i<13; i++) {
  if (strcmp(sensorNames[i],wantedName)==0) return sensorData[i];
 }
  return -255;
}

void sensorSet(char* sensorName, float sensorValue) {
 for(int i = 0; i<13; i++) {
  if (strcmp(sensorNames[i],sensorName)==0) {
    sensorData[i] = sensorValue;
  }
 }
}

float smooth(float smoothValue, float rawValue) {
  // returns a smoothed value by shifting the smoothed value 20 percent of the difference
  // between raw and smooth.
  // Initial load
  if (smoothValue == 0.0) return rawValue * 0.95;
  float diff = rawValue - smoothValue;
  return smoothValue + (diff * 0.20);
}

void Println(const char *str) { // use instead of Println(char *str)
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

   while ( WiFi.status()  != WL_CONNECTED || WiFi.RSSI() == -255)
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
    WiFi.begin("DavesInnernet","Fucker09");
    delay(5000);
   }
   Println("WiFi Connected");
}

static void connectThingsBoard() {
    Print("\nConnecting ThingsBoard...");
    while (WiFi.ping("dgriffith.com.au") < 0) {
      Print("Ping check failed, reconnecting wifi.");
      connectWiFi();
    }
    Println("Ping check good");
    while (!client.connected()) {
       MQTTConnects++;
       Print("ThingsBoard connection init...");
       client.begin("dgriffith.com.au", net);
       client.connect("ClientID","xD2kUeR9PIyYYGrACAId");
       delay(5000);
       
    }
    Println("\nThingsBoard connected.");
}

static void updateCloud(void* pvParameters)
{
  char tempBuf[30];
  while(1) { 
    vTaskDelay(5000/portTICK_PERIOD_MS);
    if (!net.connected() || WiFi.RSSI() == -255) {
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
  // Read the onewire and analog sensors.
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
    long rssi = WiFi.RSSI();
    if ( rssi < 0 && rssi > -100) {
      sensorData[7] = rssi;
    } else {
      sensorData[7] = -255;
    }
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

 // Create tasks 
 xTaskCreate(updateCloud, "MQTT", 512, NULL, 1, NULL);
 xTaskCreate(readSensors, "readData", 512, NULL,1, NULL);
 xTaskCreate(drivePump, "doPump", 512, NULL, 1, NULL);
 xTaskCreate(calcPumpSpeed, "pumpSpeed", 512, NULL, 1, NULL);
 xTaskCreate(updateScreen, "updateScreen", 512, NULL, 1, NULL);
 vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:

    Println("Looping in the Idle Task.");

    vTaskList(ptrTaskList);
    Println("*******************************************");
    Println("Task         State   Prio    Stack    Num"); 
    Println("*******************************************");
    Print(ptrTaskList);
    Println("*******************************************");
    
    delay(10000);

}
