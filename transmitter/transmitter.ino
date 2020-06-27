#include "Arduino.h"
#include "InputCmd.h"
#include "Transmit.h"

Transmit transmit;

void setup() {
    Serial.begin(115200);
    transmit.setup();
 }

void loop() {
    transmit.transmit_data();
}