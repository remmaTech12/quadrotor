//================================================================//
//  AE-BMX055             Arduino UNO                             //
//    VCC                    +5V                                  //
//    GND                    GND                                  //
//    SDA                    A4(SDA)                              //
//    SCL                    A5(SCL)                              //
//                                                                //
//   (JP6,JP4,JP5はショートした状態)                              //
//   http://akizukidenshi.com/catalog/g/gK-13010/                 //
//================================================================//

#ifndef imu_bmx055_h
#define imu_bmx055_h
#include <Wire.h>
#include "Arduino.h"
#include "MadgwickAHRS.h"
#include "def_system.h"

// BMX055　I2C address of accelerometer
#define Addr_Accl 0x19  // (in the case that JP1,JP2,JP3 are open)
// BMX055　I2C address of gyro sensor
#define Addr_Gyro 0x69  // (in the case that JP1,JP2,JP3 are open)
// BMX055　I2C address of magnetometer
#define Addr_Mag 0x13   // (in the case that JP1,JP2,JP3 are open)

class imu_bmx055 {
   public:
    imu_bmx055();

    void setup();
    void get_attitude_data(float data[3]);
    void get_angvel_data(float data[3]);

    void print_all_data();
    void print_accel_data();
    void print_gyro_data();
    void print_mag_data();
    void print_attitude_data();

   private:
    float xAccl = 0.00;
    float yAccl = 0.00;
    float zAccl = 0.00;
    float xGyro = 0.00;
    float yGyro = 0.00;
    float zGyro = 0.00;
    float xMag  = 0.00;
    float yMag  = 0.00;
    float zMag  = 0.00;
    float roll  = 0.00;
    float pitch = 0.00;
    float yaw   = 0.00;

    float xAcclBiasSum = 0.00;
    float yAcclBiasSum = 0.00;
    float zAcclBiasSum = 0.00;
    float xGyroBiasSum = 0.00;
    float yGyroBiasSum = 0.00;
    float zGyroBiasSum = 0.00;
    float yawBiasSum   = 0.00;

    float xAcclBiasAve = 0.00;
    float yAcclBiasAve = 0.00;
    float zAcclBiasAve = 0.00;
    float xGyroBiasAve = 0.00;
    float yGyroBiasAve = 0.00;
    float zGyroBiasAve = 0.00;
    float yawBiasAve   = 0.00;

    Madgwick madgwick;
    float sample_frequency;

    int cnt = 0;

    void calculate_accel();
    void calculate_gyro();
    void calculate_mag();
    void calculate_attitude();
};

#endif  // #ifndef imu_bmx055_h