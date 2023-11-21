
#include <BleKeyboard.h>
BleKeyboard bleKeyboard("LA C3 definitiva");

void setup() {

  Serial.begin(115200);
  
  Serial.println("Starting BLE work!");
  
  bleKeyboard.begin();
}

void loop() {

  if (bleKeyboard.isConnected()) {
        bleKeyboard.print("hola");
  delay(4000);
  }}