#ifndef Arm_h
#define Arm_h
#include "Arduino.h"
#include "def_system.h"

class Arm {
   public:
    Arm();

    void setup();
    void set_arm_status(bool armed);
    bool get_arm_status();

   private:
    bool m_armed = false;
};

#endif  // #ifndef Arm_h