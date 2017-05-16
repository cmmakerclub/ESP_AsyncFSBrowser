#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

#include <Ticker.h>
#include <Hash.h>
#include <ArduinoOTA.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>
#include <ArduinoJson.h>
#include <CMMC_Blink.hpp>
#include "ota.h"
#include "doconfig.h"
#include "util.h"
extern "C" {
  #include <espnow.h>
  #include <user_interface.h>
}
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN            12
uint32_t delayMS;
// Uncomment the type of sensor in use:
//#define DHTTYPE         DHT11     // DHT 11
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

const char* ssid = "belkin.636";
const char* password = "3eb7e66b";
const char * hostName = "esp-async";
const char* http_username = "admin";
const char* http_password = "admin";


uint8_t master_mac[6];

// SKETCH BEGIN
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncEventSource events("/events");
CMMC_Blink *blinker;

uint32_t counter = 0;
uint32_t send_ok_counter = 0;
uint32_t send_fail_counter = 0;
bool must_send_data = 0;
Ticker ticker;
bool longpressed = false;


#include "webserver.h"


void setup(){
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  SPIFFS.begin();
  WiFi.disconnect();
  // Initialize device.
  dht.begin();
  Serial.println("DHTxx Unified Sensor Example");
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(13, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, HIGH);
  blinker = new CMMC_Blink;
  blinker->init();
  Serial.println("Wating configuration pin..");
  _wait_config_signal(13, &longpressed);
  if (longpressed) {
    WiFi.hostname(hostName);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(hostName);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.printf("STA: Failed!\n");
      WiFi.disconnect(false);
      delay(1000);
      WiFi.begin(ssid, password);
    }
    setupWebServer();
  }
  else {
    WiFi.disconnect();
    Serial.println("Initializing ESPNOW...");
    DEBUG_PRINTLN("Initializing... SLAVE");
    WiFi.mode(WIFI_AP_STA);

    uint8_t macaddr[6];
    wifi_get_macaddr(STATION_IF, macaddr);
    DEBUG_PRINT("[master] mac address (STATION_IF): ");
    printMacAddress(macaddr);

    wifi_get_macaddr(SOFTAP_IF, macaddr);
    DEBUG_PRINT("[slave] mac address (SOFTAP_IF): ");
    printMacAddress(macaddr);

    if (esp_now_init() == 0) {
      DEBUG_PRINTLN("init");
    } else {
      DEBUG_PRINTLN("init failed");
      ESP.restart();
      return;
    }
    DEBUG_PRINTLN("SET ROLE SLAVE");
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    static uint8_t recv_counter = 0;
    esp_now_register_recv_cb([](uint8_t *macaddr, uint8_t *data, uint8_t len) {
      recv_counter++;
      // DEBUG_PRINTLN("recv_cb");
      // DEBUG_PRINT("mac address: ");
      // printMacAddress(macaddr);
      DEBUG_PRINT("data: ");
      for (int i = 0; i < len; i++) {
        DEBUG_PRINT(" 0x");
        DEBUG_PRINT(data[i], HEX);
      }
        DEBUG_PRINT(data[0], DEC);

      DEBUG_PRINTLN("");
      digitalWrite(LED_BUILTIN, data[0]);
      if (data[0] == 0xff && data[1] == 0xfa) {
        if (data[2] == 0x00 && data[3] == 0x00) {
          Serial.printf("CLEAR COUNTER >>> %lu \r\n", recv_counter);
          uint8_t msg[] = { recv_counter };
          esp_now_send(master_mac, msg, 1);
          recv_counter = 0;
        }
      }
    });

    esp_now_register_send_cb([](uint8_t* macaddr, uint8_t status) {
      DEBUG_PRINT(millis());
      DEBUG_PRINT("send to mac addr: ");
      printMacAddress(macaddr);
      if (status == 0) {
        send_ok_counter++;
        counter++;
        DEBUG_PRINTF("... send_cb OK. [%lu/%lu]\r\n", send_ok_counter, send_ok_counter + send_fail_counter);
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else {
        send_fail_counter++;
        DEBUG_PRINTF("... send_cb FAILED. [%lu/%lu]\r\n", send_ok_counter, send_ok_counter + send_fail_counter);
      }
    });
  }
  setupOTA();
  loadConfig(master_mac);
  // ticker.attach_ms(500, [&]() {
  //   must_send_data = 1;
  // });
}
uint8_t message[20] = {0};

void loop(){
  ArduinoOTA.handle();
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    Serial.print("Temperature: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
  }
  uint32_t temperature_uint32 = (uint32_t)(event.temperature*100);
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    Serial.print("Humidity: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }
  uint32_t humidity_uint32 = (uint32_t)(event.relative_humidity*100);

  if (digitalRead(13) == HIGH) {
    Serial.println("BUTTON PRESSED.");
    digitalWrite(LED_BUILTIN, LOW);
    // DEBUG_PRINTf("[%lu] sending...\r\n", millis());
    // | START | MSGTYPE | CAT  | SENSOR |  UID |   DATA   | BATT  | SUM |
    // FF FA    0x00     0x00    0x01      4       4       2      FF
    //   2        1       1        1       4         4       2      1

    // struct {
    //   byte category;
    //   uint32_t uuid;
    //   uint16_t batt;
    // } data;

    message[0] = 0xff;
    message[1] = 0xfa;

    message[2] = 0x01;
    message[3] = 0x01;
    message[4] = 0x01;

    // UUID
    message[5]  = 0x01;
    message[6]  = 0x02;
    message[7]  = 0x03;
    message[8]  = 0x04;
    message[9]  = 0x05;
    message[10] = 0x06;

    memcpy(message+11, (const void*)&temperature_uint32, 4);
    memcpy(message+15, (const void*)&humidity_uint32, 4);

    message[19] = 0xFF;

    Serial.println(temperature_uint32, HEX);
    Serial.println(humidity_uint32, HEX);

    Serial.println(temperature_uint32);
    Serial.println(humidity_uint32);

    // uint8_t master_mac2[] = {0x18,0xFE,0x34,0xEE,0xA0,0xF9};
    esp_now_send(master_mac, message, sizeof(message));
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
  }
}
