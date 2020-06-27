#include "recv.h"

BluetoothSerial SerialBT;

Receiver::Receiver() {}

void Receiver::setup() {
    /*
    pinMode(LED_PIN1, OUTPUT);
    pinMode(LED_PIN2, OUTPUT);
    pinMode(LED_PIN3, OUTPUT);
    pinMode(LED_PIN4, OUTPUT);
    */

    Serial.begin(115200);
    SerialBT.begin("ESP32test");  // Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");

/*
    ledcSetup(0, 12800, 8);
    ledcAttachPin(LED_PIN1, 0);
    ledcSetup(1, 12800, 8);
    ledcAttachPin(LED_PIN2, 1);
    ledcSetup(2, 12800, 8);
    ledcAttachPin(LED_PIN3, 2);
    ledcSetup(3, 12800, 8);
    ledcAttachPin(LED_PIN4, 3);
    */
}

void Receiver::notify_bluetooth_setup_finished() {
    pinMode(BUILTIN_LED, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUILTIN_LED, HIGH);
        delay(50);
        digitalWrite(BUILTIN_LED, LOW);
        delay(50);
    }
}

uint8_t Receiver::calculate_checksum(uint8_t *data) {
  uint8_t checksum = 0;
  checksum |= 0b11000000 & data[1];
  checksum |= 0b00110000 & data[2];
  checksum |= 0b00001100 & data[3];
  checksum |= 0b00000011 & data[4];

  return checksum;
}

bool Receiver::is_left_switch_pressed(uint8_t *data) {
    if ((0x01 & data[5]) == 0x00) {
        return true;
    }
    return false;
}

bool Receiver::is_right_switch_pressed(uint8_t *data) {
    if ((0x02 & data[5]) == 0x00) {
        return true;
    }
    return false;
}

void Receiver::led_control(uint8_t *data) {
    ledcWrite(0, data[1]);
    ledcWrite(1, data[2]);
    ledcWrite(2, data[3]);
    ledcWrite(3, data[4]);

    if (is_left_switch_pressed(data)) {
        ledcWrite(0, 0);
        ledcWrite(1, 0);
        ledcWrite(2, 0);
        ledcWrite(3, 0);
    }
    if (is_right_switch_pressed(data)) {
        ledcWrite(0, 255);
        ledcWrite(1, 255);
        ledcWrite(2, 255);
        ledcWrite(3, 255);
    }
}

void Receiver::update_data() {
    //uint8_t recv_data[RECEIVE_DATA_SIZE];
    if (SerialBT.available()) {
        SerialBT.readBytes(recv_data, RECEIVE_DATA_SIZE);

        if (recv_data[0] != 'T') {
            Serial.print("Receive error!");
            return;
        }

        if (recv_data[6] != calculate_checksum(recv_data)) {
            Serial.print("Decode error!");
            return;
        }
#ifdef DEBUG_BUILD
        Serial.println(recv_data[0]);
        Serial.println(recv_data[1]);
        Serial.println(recv_data[2]);
        Serial.println(recv_data[3]);
#endif

        //led_control(recv_data);
    }
}

void Receiver::get_command(int data[4]) {
    data[0] = recv_data[1]; // thrust
    data[1] = recv_data[2]; // yaw
    data[2] = recv_data[3]; // pitch
    data[3] = recv_data[4]; // roll
}