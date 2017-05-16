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

extern CMMC_Blink *blinker;

void printMacAddress(uint8_t* macaddr) {
  Serial.print("{");
  for (int i = 0; i < 6; i++) {
    Serial.print("0x");
    Serial.print(macaddr[i], HEX);
    if (i < 5) Serial.print(',');
  }
  Serial.println("};");
}

void _wait_config_signal(uint8_t gpio, bool* longpressed) {
    unsigned long _c = millis();
    while(digitalRead(gpio) == LOW) {
      if((millis() - _c) >= 1000) {
        *longpressed = true;
        blinker->blink(500, LED_BUILTIN);
        Serial.println("Release to take an effect.");
        while(digitalRead(gpio) == LOW) {
          yield();
        }
      }
      else {
        *longpressed = false;
        yield();
      }
    }
    // Serial.println("/NORMAL");
}
