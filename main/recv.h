#ifndef Receiver_h
#define Receiver_h
#include "def_system.h"
#include "Arduino.h"
#include "BluetoothSerial.h"
#include "arm.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

class Receiver {
   public:
    Receiver();

    void setup(Arm &arm);
    void update_data();
    void get_command(int data[4]);
    void set_arm_status(Arm &arm);

   private:
    Arm m_arm;
    uint8_t recv_data[RECEIVE_DATA_SIZE];
    unsigned int left_x_val = 0;
    unsigned int left_y_val = 0;

    void notify_bluetooth_setup_finished();
    uint8_t calculate_checksum(uint8_t *data);
    void led_control(uint8_t *data);

    bool is_left_switch_pressed(uint8_t *data);
    bool is_right_switch_pressed(uint8_t *data);
};

#endif  // #ifndef Receiver_h