#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <FS.h>
#include <Ticker.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>
#include <ArduinoJson.h>
#include <CMMC_Blink.hpp>
extern "C" {
  #include <espnow.h>
  #include <user_interface.h>
}

// SKETCH BEGIN
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncEventSource events("/events");
CMMC_Blink *blinker;
uint8_t master_mac[6];
uint32_t counter = 0;
uint32_t send_ok_counter = 0;
uint32_t send_fail_counter = 0;
bool must_send_data = 0;
Ticker ticker;

#define DEBUG_SERIAL 1
#if DEBUG_SERIAL
    #define DEBUG_PRINTER Serial
    #define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
    #define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
    #define DEBUG_PRINTF(...) { DEBUG_PRINTER.printf(__VA_ARGS__); }
#else
    #define DEBUG_PRINT(...) { }
    #define DEBUG_PRINTLN(...) { }
    #define DEBUG_PRINTF(...) { }
#endif

bool saveConfig(String mac);
bool loadConfig() {
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println("Config file size is too large");
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());

  if (!json.success()) {
    Serial.println("Failed to parse config file");
    return false;
  }

  const char* mac = json["mac"];
  String macStr = String(mac);
  for (size_t i = 0; i < 12; i+=2) {
    String mac = macStr.substring(i, i+2);
    byte b = strtoul(mac.c_str(), 0, 16);
    master_mac[i/2] = b;
  }
  printMacAddress(master_mac);

  return true;
}

void printMacAddress(uint8_t* macaddr) {
  Serial.print("{");
  for (int i = 0; i < 6; i++) {
    Serial.print("0x");
    Serial.print(macaddr[i], HEX);
    if (i < 5) Serial.print(',');
  }
  Serial.println("};");
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
    client->printf("Hello Client %u :)", client->id());
    client->ping();
  } else if(type == WS_EVT_DISCONNECT){
    Serial.printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
  } else if(type == WS_EVT_ERROR){
    Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
  } else if(type == WS_EVT_PONG){
    Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len)?(char*)data:"");
  } else if(type == WS_EVT_DATA){
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    String msg = "";
    if(info->final && info->index == 0 && info->len == len){
      //the whole message is in a single frame and we got all of it's data
      Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);
      Serial.printf("[HEX]= %x \r\n", info->len);
      Serial.printf("size = %d \r\n", info->len);
      if(info->opcode == WS_TEXT){
        for(size_t i=0; i < info->len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < info->len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }

      Serial.printf("MESSAGE => %s\n",msg.c_str());
      String header = msg.substring(0, 7);
      String value = msg.substring(7);
      String macStr;
      bool validMessage = 0;
      if (header == "MASTER:" && value.length() == 12) {
        macStr = value;
        validMessage = true;
        saveConfig(macStr);
      }
      else {
        Serial.print("INVALID:");
        Serial.println(msg);
      }

      if(info->opcode == WS_TEXT)
        if (validMessage) {
          // client->text("I got your text message");
          client->text(macStr);
        }
        else {
          client->text(String("INVALID: ") + msg);
        }
      else
        client->binary("I got your binary message");
    } else {
      //message is comprised of multiple frames or the frame is split into multiple packets
      if(info->index == 0){
        if(info->num == 0)
          Serial.printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
        Serial.printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
      }

      Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT)?"text":"binary", info->index, info->index + len);

      if(info->opcode == WS_TEXT){
        for(size_t i=0; i < info->len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < info->len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }
      Serial.printf("%s\n",msg.c_str());

      if((info->index + len) == info->len){
        Serial.printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);
        if(info->final){
          Serial.printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
          if(info->message_opcode == WS_TEXT)
            client->text("I got your text message");
          else
            client->binary("I got your binary message");
        }
      }
    }
  }
}

const char* ssid = "Boonchukamai";
const char* password = "0899506685";
const char * hostName = "esp-async";
const char* http_username = "admin";
const char* http_password = "admin";

bool saveConfig(String mac) {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["mac"] = mac;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return false;
  }

  json.printTo(configFile);
  return true;
}

void _wait_config_signal(uint8_t gpio, bool* longpressed) {
    Serial.println("WAITING... CONFIG PIN");
    unsigned long _c = millis();
    Serial.println(digitalRead(gpio));
    while(digitalRead(gpio) == LOW) {
      if((millis() - _c) >= 1000) {
        *longpressed = true;
        blinker->blink(500, LED_BUILTIN);
        Serial.println("Release to take an effect.");
        while(digitalRead(gpio) == LOW) {
          yield();
        }
        // Serial.println("Restarting...");
        // rtcData.data[0] = CMMC_RTC_MODE_AP;
        // WiFi.disconnect();
        // WiFi.mode(WIFI_AP_STA);
        // writeRTCMemory();
        // // Try pushing frequency to 160MHz.
        // system_update_cpu_freq(SYS_CPU_160MHZ);
        // ESP.reset();
      }
      else {
        yield();
      }
    }
    // Serial.println("/NORMAL");
}

void setupOTA() {
    //Send OTA events to the browser
    ArduinoOTA.onStart([]() { events.send("Update Start", "ota"); });
    ArduinoOTA.onEnd([]() { events.send("Update End", "ota"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      char p[32];
      sprintf(p, "Progress: %u%%\n", (progress/(total/100)));
      events.send(p, "ota");
    });
    ArduinoOTA.onError([](ota_error_t error) {
      if(error == OTA_AUTH_ERROR) events.send("Auth Failed", "ota");
      else if(error == OTA_BEGIN_ERROR) events.send("Begin Failed", "ota");
      else if(error == OTA_CONNECT_ERROR) events.send("Connect Failed", "ota");
      else if(error == OTA_RECEIVE_ERROR) events.send("Recieve Failed", "ota");
      else if(error == OTA_END_ERROR) events.send("End Failed", "ota");
    });
    ArduinoOTA.setHostname(hostName);
    ArduinoOTA.begin();

    MDNS.addService("http","tcp",80);
}

void setupWebServer() {
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    events.onConnect([](AsyncEventSourceClient *client){
      client->send("hello!",NULL,millis(),1000);
    });
    server.addHandler(&events);
    server.addHandler(new SPIFFSEditor(http_username,http_password));
    server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", String(ESP.getFreeHeap()));
    });
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");
    server.onNotFound([](AsyncWebServerRequest *request){
      Serial.printf("NOT_FOUND: ");
      if(request->method() == HTTP_GET)
        Serial.printf("GET");
      else if(request->method() == HTTP_POST)
        Serial.printf("POST");
      else if(request->method() == HTTP_DELETE)
        Serial.printf("DELETE");
      else if(request->method() == HTTP_PUT)
        Serial.printf("PUT");
      else if(request->method() == HTTP_PATCH)
        Serial.printf("PATCH");
      else if(request->method() == HTTP_HEAD)
        Serial.printf("HEAD");
      else if(request->method() == HTTP_OPTIONS)
        Serial.printf("OPTIONS");
      else
        Serial.printf("UNKNOWN");
      Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

      if(request->contentLength()){
        Serial.printf("_CONTENT_TYPE: %s\n", request->contentType().c_str());
        Serial.printf("_CONTENT_LENGTH: %u\n", request->contentLength());
      }

      int headers = request->headers();
      int i;
      for(i=0;i<headers;i++){
        AsyncWebHeader* h = request->getHeader(i);
        Serial.printf("_HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
      }

      int params = request->params();
      for(i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isFile()){
          Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
        } else if(p->isPost()){
          Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        } else {
          Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
      }

      request->send(404);
    });
    server.onFileUpload([](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
      if(!index)
        Serial.printf("UploadStart: %s\n", filename.c_str());
      Serial.printf("%s", (const char*)data);
      if(final)
        Serial.printf("UploadEnd: %s (%u)\n", filename.c_str(), index+len);
    });
    server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
      if(!index)
        Serial.printf("BodyStart: %u\n", total);
      Serial.printf("%s", (const char*)data);
      if(index + len == total)
        Serial.printf("BodyEnd: %u\n", total);
    });
    server.begin();
    Serial.println("Starting webserver...");
}

void setup(){
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  SPIFFS.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(13, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, HIGH);
  blinker = new CMMC_Blink;
  blinker->init();
  Serial.println("Wating configuration pin..");
  bool longpressed = false;
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
    esp_now_register_recv_cb([](uint8_t *macaddr, uint8_t *data, uint8_t len) {
      DEBUG_PRINTLN("recv_cb");
      DEBUG_PRINT("mac address: ");
      printMacAddress(macaddr);
      DEBUG_PRINT("data: ");
      for (int i = 0; i < len; i++) {
        DEBUG_PRINT(" 0x");
        DEBUG_PRINT(data[i], HEX);
      }
      DEBUG_PRINTLN("");
      digitalWrite(LED_BUILTIN, data[0]);
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
  loadConfig();
  ticker.attach_ms(500, [&]() {
    must_send_data = 1;
  });
}
uint8_t message[] = {0};

void loop(){
  ArduinoOTA.handle();
  if (must_send_data) {
    must_send_data = 0;
    // DEBUG_PRINTf("[%lu] sending...\r\n", millis());
    message[3] =  counter & 0xFF;
    message[2] = (counter >> 8)  & 0xFF;
    message[1] = (counter >> 16) & 0xFF;
    message[0] = (counter >> 24) & 0xFF;
    // digitalWrite(LED_BUILTIN, LOW);
    // DEBUG_PRINTLN(millis());
    esp_now_send(master_mac, message, 4);
  }
}
