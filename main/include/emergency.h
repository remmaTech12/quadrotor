#ifndef Emergency_h
#define Emergency_h
#include "Arduino.h"
#include "def_system.h"
#include "arm.h"
#include "motor.h"

class Emergency {
   public:
    Emergency();

    void setup();
    void emergency_stop(Arm &arm, Motor &motor);

   private:
};

#endif  // #ifndef Emergency_h