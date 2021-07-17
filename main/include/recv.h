#ifndef Receiver_h
#define Receiver_h
#include "def_system.h"
#include "Arduino.h"
#include "BluetoothSerial.h"
#include "arm.h"
#include "motor.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

class Receiver {
   public:
    Receiver();

    void setup();
    void update_data();
    void get_command(int data[4]);
    void set_arm_status(Arm &arm);
    void emergency_stop(Arm &arm, Motor &motor);
    bool is_left_switch_pressed();

   private:
    int disconnect_count = 0;
    bool first_byte_check = true;
    bool checksum_success = true;
    uint8_t recv_data[RECEIVE_DATA_SIZE];
    uint8_t pre_left_sw_data  = 0x00;
    uint8_t pre_right_sw_data = 0x00;

    void notify_bluetooth_setup_finished();
    uint8_t calculate_checksum();
    bool is_right_switch_pressed();
};

#endif  // #ifndef Receiver_h