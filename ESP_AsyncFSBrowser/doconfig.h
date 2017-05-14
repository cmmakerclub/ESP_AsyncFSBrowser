#include <FS.h>
#include <ArduinoJson.h>

uint8_t master_mac[6];

extern void printMacAddress(uint8_t* macaddr);

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
