#ifndef InputCmd_h
#define InputCmd_h
#include "Arduino.h"
#include "def_system.h"

class InputCmd {
   public:
    InputCmd();

    void setup();
    void sense_value();

    int get_left_sw_val()  { return left_sw_val; }
    int get_left_x_val()   { return left_x_val; }
    int get_left_y_val()   { return left_y_val; }

    int get_right_sw_val() { return right_sw_val; }
    int get_right_x_val()  { return right_x_val; }
    int get_right_y_val()  { return right_y_val; }

#ifdef ADDITIONAL_BUTTONS
    int get_left_sw2_val()  { return left_sw2_val; }
    int get_right_sw2_val()  { return right_sw2_val; }
#endif

   private:
    char left_sw_val = 0;
    unsigned int left_x_val = 0;
    unsigned int left_y_val = 0;

    char right_sw_val = 0;
    unsigned int right_x_val = 0;
    unsigned int right_y_val = 0;

#ifdef ADDITIONAL_BUTTONS
    char left_sw2_val = 0;
    char right_sw2_val = 0;
#endif
};
#endif  // #ifndef InputCmd_h
