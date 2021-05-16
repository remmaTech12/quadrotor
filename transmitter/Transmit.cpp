#include "Transmit.h"

BluetoothSerial SerialBT;

Transmit::Transmit() {}

void Transmit::setup() {
    input.setup();

    bluetooth_setup();
    notify_bluetooth_setup_finished();
}

void Transmit::bluetooth_setup() {
    SerialBT.begin("ESP32test", true);
    Serial.println("The device started in master mode, make sure remote BT device is on!");

    connected = SerialBT.connect(address);
    if (connected) {
        Serial.println("Connected Succesfully!");
    } else {
        while (!SerialBT.connected(10000)) {
            Serial.println(
                "Failed to connect. Make sure remote device is available and "
                "in range, then restart app.");
        }
    }
    if (SerialBT.disconnect()) {
        Serial.println("Disconnected Succesfully!");
    }
    SerialBT.connect();
}

void Transmit::notify_bluetooth_setup_finished() {
    pinMode(BUILTIN_LED, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUILTIN_LED, HIGH);
        delay(50);
        digitalWrite(BUILTIN_LED, LOW);
        delay(50);
    }
}

uint8_t Transmit::calculate_checksum(uint8_t *data) {
  uint8_t checksum = 0;
  checksum |= 0b11000000 & data[1];
  checksum |= 0b00110000 & data[2];
  checksum |= 0b00001100 & data[3];
  checksum |= 0b00000011 & data[4];

  return checksum;
}

uint8_t Transmit::pack_switch_data() {
    uint8_t left_sw_data  = 0x01 & input.get_left_sw_val();
    uint8_t right_sw_data = 0x02 & (input.get_right_sw_val() << 1);
    return left_sw_data | right_sw_data;
}

void Transmit::transmit_data() {
    uint8_t send_data[TRANSMIT_DATA_SIZE];

    input.sense_value();
#ifdef DEBUG_BUILD
    Serial.print("Left Switch: ");
    Serial.print(input.get_left_sw_val());
    Serial.print(", ");
    Serial.print("Left X-axis: ");
    Serial.print(input.get_left_x_val());
    Serial.print(", ");
    Serial.print("Left Y-axis: ");
    Serial.print(input.get_left_y_val());
    Serial.print(", ");

    Serial.print("Right Switch: ");
    Serial.print(input.get_right_sw_val());
    Serial.print(", ");
    Serial.print("Right X-axis: ");
    Serial.print(input.get_right_x_val());
    Serial.print(", ");
    Serial.print("Right Y-axis: ");
    Serial.print(input.get_right_y_val());
    Serial.print("\n\n");
#endif

#ifdef ADDITIONAL_BUTTONS
    Serial.print("Left Switch2: ");
    Serial.print(input.get_left_sw2_val());
    Serial.print(", ");
    Serial.print("Right Switch2: ");
    Serial.print(input.get_right_sw2_val());
    Serial.print(", ");
#endif

    send_data[0] = 'T';
    send_data[1] = input.get_left_y_val();
    send_data[2] = input.get_left_x_val();
    send_data[3] = input.get_right_y_val();
    send_data[4] = input.get_right_x_val();
    send_data[5] = pack_switch_data();
    send_data[6] = calculate_checksum(send_data);
    SerialBT.write(send_data, sizeof(send_data)/sizeof(send_data[0]));

    delay(m_sampling_time_ms);
}
