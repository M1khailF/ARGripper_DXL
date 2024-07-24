#include "DxlMaster.h"

#define USW_DATA 27             //uint8_t 0..1 return status
DynamixelDevice device(23);

#define statePIN 5
#define LED 13
#define debug true

uint8_t data_sw;
int data_sw_con;
int state = 0;
int key = 100;

int buttonState = 0;

unsigned long previousMillis1 = 0;
const long interval1 = 10;

unsigned long previousMillis2 = 0;
const long interval2 = 50;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  pinMode(LED, OUTPUT);
  pinMode(statePIN, INPUT_PULLUP);

  DxlMaster.begin(57600);
  while (device.ping() != DYN_STATUS_OK) {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }

  digitalWrite(LED, HIGH);
}

char data[4];

void loop() {
  int buttonState = digitalRead(statePIN);

  state = (buttonState == LOW) ? 1 : 0;

  unsigned long currentMillis1 = millis();
  unsigned long currentMillis2 = millis();
  if (currentMillis1 - previousMillis1 >= interval1) {
    previousMillis1 = currentMillis1;  // то запоминаем текущее время
    device.read(USW_DATA, data_sw);
    if (data_sw == 0)
    {
      data_sw_con = 10;
    }
    else
    {
      data_sw_con = 80;
    }

    key = data_sw_con + state;
  }

  if (currentMillis2 - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis2;
    int len = String(key).length() + 1;
    char arr[len];
    String(key).toCharArray(arr, len);
    for (int i = 0; i < len; i++) {
      Serial2.write(arr[i]);
    }

    if (debug) {
      Serial.print(" key -> ");
      Serial.println(key);
    }
  }
}
