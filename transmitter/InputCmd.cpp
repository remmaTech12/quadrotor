#include "InputCmd.h"

InputCmd::InputCmd() {}
void InputCmd::setup() { 
    pinMode(LEFT_SW_PIN, INPUT_PULLUP);
    pinMode(RIGHT_SW_PIN, INPUT_PULLUP);

#ifdef ADDITIONAL_BUTTONS
    pinMode(LEFT_SW2_PIN, INPUT_PULLUP);
    pinMode(RIGHT_SW2_PIN, INPUT_PULLUP);
#endif
}

void InputCmd::sense_value() {
    left_sw_val = digitalRead(LEFT_SW_PIN);
    left_x_val  = analogRead (LEFT_X_PIN) >> 4;
    left_y_val  = analogRead (LEFT_Y_PIN) >> 4;

    right_sw_val = digitalRead(RIGHT_SW_PIN);
    right_x_val  = analogRead (RIGHT_X_PIN) >> 4;
    right_y_val  = analogRead (RIGHT_Y_PIN) >> 4;

#ifdef ADDITIONAL_BUTTONS
    left_sw2_val = digitalRead(LEFT_SW2_PIN);
    right_sw2_val = digitalRead(RIGHT_SW2_PIN);
#endif
}
