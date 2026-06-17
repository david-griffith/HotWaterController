// Local settings. Change your settings here!

//Wifi settings
#define MY_SSID "DavesInnernet"
#define WIFI_PASSWORD "Fucker09"

// MQTT settings

const char* mqtt_server = "192.168.1.110";  // MQTT Server URL
const uint16_t mqtt_server_port = 1883;               // MQTT Server port

const char*  _MQTTUSER = "";                    // MQTT Username
const char*  _MQTTPASSWORD = "";                // MQTT Password

const char*  _MANUF = "DaveSoft";                   // Manufacturer name
const char*  _MODEL = "SolaControlla";             // Device model name
const char*  _VERSION = "v0.1";               // Device firmware version
const char*  _NAME = "Solar Hot Water";               // Device name in Home Assistant
byte  _MAC[] = {0,0,0,0,0,0,1};
