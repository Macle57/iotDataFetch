#include "select.h"
#ifdef ENASOIL
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial.println(F("Soil moisture test"));

}

void loop(){
    Serial.println("Soil moisture: ");
    int sensorValue = analogRead(27); // read the input on analog pin 34
    Serial.println(sensorValue);        // print out the value you read
    delay(2000);                       // delay in between reads for stability
}

#endif