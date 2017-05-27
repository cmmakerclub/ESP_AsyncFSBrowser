#include <Arduino.h>
#include <Ticker.h>
#include <CMMC_Blink.hpp>


extern CMMC_Blink *blinker;
// goto sleep when no events after this time.
const int idleTime = 10;

// Time to sleep (in seconds):
uint16_t sleepTime = 5;

bool bTimeout = false;

void gotoDeepSleep()
{
  bTimeout = true;
}

// Battery >>>
// max values in storange
#define MaxValues 21

// default
//#define MaxVoltage 6200
// adjusted
#define MaxVoltage 6080

#define TickerBattery 20
Ticker tickerBattery;


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

void swap(int &a, int &b)
{
  int t = a;
  a = b;
  b = t;
}

int partition(int *arr, const int left, const int right)
{
  const int mid = left + (right - left) / 2;
  const int pivot = arr[mid];
  // move the mid point value to the front.
  swap(arr[mid], arr[left]);
  int i = left + 1;
  int j = right;
  while (i <= j)
  {
    while (i <= j && arr[i] <= pivot)
    {
      i++;
    }

    while (i <= j && arr[j] > pivot)
    {
      j--;
    }

    if (i < j)
    {
      swap(arr[i], arr[j]);
    }
  }

  swap(arr[i - 1], arr[left]);
  return i - 1;
}

void quickSort(int *arr, const int left, const int right)
{
  if (left >= right)
  {
    return;
  }

  int part = partition(arr, left, right);

  quickSort(arr, left, part - 1);
  quickSort(arr, part + 1, right);
}

int median(int arr[], int maxValues)
{
  quickSort(arr, 0, maxValues - 1);
  return arr[maxValues / 2];
}

int getBatteryVoltage()
{
  static int filters[MaxValues] = {0};
  static int lastIndex = 0;

  int val = ESP.getVcc();

  filters[lastIndex++ % MaxValues] = val;
  val = median(filters, MaxValues);

  // for AVG >>>
  //static float filterAvg = 0;
  //filterAvg = (filterAvg + val) / 2.0;
  //return map(filterAvg, 0, 1023, 0, MaxVoltage);
  // for AVG <<<

  return map(val, 0, 1023, 0, MaxVoltage);
}

void storeBatteryValue()
{
  // DEBUG_PRINTF("storeBatteryValue:: %d \r\n", getBatteryVoltage()) ;
  getBatteryVoltage();
}

void initBattery()
{
  // Store values to buffer
  for (int i = 0; i < MaxValues; ++i)
  {
    storeBatteryValue();
  }

  if (TickerBattery > 0)
  {
    tickerBattery.attach_ms(TickerBattery, storeBatteryValue);
  }
}
