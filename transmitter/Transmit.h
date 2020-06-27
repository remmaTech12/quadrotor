#ifndef Transmit_h
#define Transmit_h
#include "BluetoothSerial.h"
#include "Arduino.h"
#include "InputCmd.h"
#include "def_system.h"

class Transmit {
    public:
        Transmit();
        InputCmd input;

        void setup();
        void transmit_data();

    private:
        int m_sampling_time_ms = SAMPLING_TIME_MS;
        String MACadd = "24:62:AB:DC:30:5E";
        uint8_t address[6] = {0x24, 0x62, 0xAB, 0xDC, 0x30, 0x5E};
        bool connected;

        void bluetooth_setup();
        void notify_bluetooth_setup_finished();
        uint8_t calculate_checksum(uint8_t *data);
        uint8_t pack_switch_data();
};
#endif  // #ifndef tranmit_h