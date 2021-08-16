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

#include "../include/imu_bmx055.h"

imu_bmx055::imu_bmx055() {
}

void imu_bmx055::setup() {
    Wire.begin();
    sample_frequency = (float) 1/((float) SAMPLING_TIME_MS/1000);
    madgwick.begin(sample_frequency);

    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x0F);  // Select PMU_Range register
    Wire.write(0x03);  // Range = +/- 2g
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x10);  // Select PMU_BW register
    Wire.write(0x08);  // Bandwidth = 7.81 Hz
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x11);  // Select PMU_LPW register
    Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x0F);  // Select Range register
    Wire.write(0x04);  // Full scale = +/- 125 degree/s
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x10);  // Select Bandwidth register
    Wire.write(0x07);  // ODR = 100 Hz
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x11);  // Select LPM1 register
    Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4B);  // Select Mag register
    Wire.write(0x83);  // Soft reset
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4B);  // Select Mag register
    Wire.write(0x01);  // Soft reset
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4C);  // Select Mag register
    Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4E);  // Select Mag register
    Wire.write(0x84);  // X, Y, Z-Axis enabled
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x51);  // Select Mag register
    Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x52);  // Select Mag register
    Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
    Wire.endTransmission();
}

void imu_bmx055::get_attitude_data(float data[3]) {
    calculate_attitude();
    data[0] = roll;
    data[1] = pitch;
    data[2] = yaw;
}

void imu_bmx055::get_angvel_data(float data[3]) {
    calculate_gyro();
    data[0] = xGyro;
    data[1] = yGyro;
    data[2] = zGyro;
}

void imu_bmx055::print_all_data() {
    Serial.println("--------------------------------------");

    Serial.print("Accl= ");
    Serial.print(xAccl);
    Serial.print(",");
    Serial.print(yAccl);
    Serial.print(",");
    Serial.print(zAccl);
    Serial.println("");

    calculate_gyro();
    Serial.print("Gyro= ");
    Serial.print(xGyro);
    Serial.print(",");
    Serial.print(yGyro);
    Serial.print(",");
    Serial.print(zGyro);
    Serial.println("");

    calculate_mag();
    Serial.print("Mag= ");
    Serial.print(xMag);
    Serial.print(",");
    Serial.print(yMag);
    Serial.print(",");
    Serial.print(zMag);
    Serial.println("");
}

void imu_bmx055::print_accel_data() {
    calculate_accel();
    Serial.print(xAccl);
    Serial.print(" ");
    Serial.print(yAccl);
    Serial.print(" ");
    Serial.println(zAccl);
}

void imu_bmx055::print_gyro_data() {
    calculate_gyro();
    Serial.print(xGyro);
    Serial.print(" ");
    Serial.print(yGyro);
    Serial.print(" ");
    Serial.println(zGyro);
}

void imu_bmx055::print_mag_data() {
    calculate_mag();
    Serial.print(xMag);
    Serial.print(" ");
    Serial.print(yMag);
    Serial.print(" ");
    Serial.println(zMag);
}

void imu_bmx055::print_attitude_data() {
    //calculate_attitude();
    Serial.print("roll: ");
    Serial.print(roll);
    Serial.print(", pitch: ");
    Serial.print(pitch);
    Serial.print(", yaw: ");
    Serial.println(yaw);
}

void imu_bmx055::calculate_accel() {
    int data[6];
    for (int i = 0; i < 6; i++) {
        Wire.beginTransmission(Addr_Accl);
        Wire.write((2 + i));  // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_Accl, 1);  // Request 1 byte of data
        // Read 6 bytes of data
        // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
        if (Wire.available() == 1) data[i] = Wire.read();
    }
    // Convert the data to 12-bits
    xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
    if (xAccl > 2047) xAccl -= 4096;
    yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
    if (yAccl > 2047) yAccl -= 4096;
    zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
    if (zAccl > 2047) zAccl -= 4096;
    xAccl = xAccl * 0.0098;  // renge +-2g
    yAccl = yAccl * 0.0098;  // renge +-2g
    zAccl = zAccl * 0.0098;  // renge +-2g

    if (cnt > cnt_start_num && cnt <= cnt_start_num + cnt_total_num) {
        xAcclBiasSum += xAccl;
        yAcclBiasSum += yAccl;
        zAcclBiasSum += zAccl;
    } else {
        xAcclBiasAve = xAcclBiasSum / cnt_total_num;
        yAcclBiasAve = yAcclBiasSum / cnt_total_num;
        zAcclBiasAve = zAcclBiasSum / cnt_total_num;

        xAccl -= xAcclBiasAve;  //  Full scale = +/- 125 degree/s
        yAccl -= yAcclBiasAve;  //  Full scale = +/- 125 degree/s
        zAccl -= zAcclBiasAve - 9.8;  //  Full scale = +/- 125 degree/s
    }
}

void imu_bmx055::calculate_gyro() {
    int data[6];
    for (int i = 0; i < 6; i++) {
        Wire.beginTransmission(Addr_Gyro);
        Wire.write((2 + i));  // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_Gyro, 1);  // Request 1 byte of data
        // Read 6 bytes of data
        // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
        if (Wire.available() == 1) data[i] = Wire.read();
    }
    // Convert the data
    xGyro = (data[1] * 256) + data[0];
    if (xGyro > 32767) xGyro -= 65536;
    yGyro = (data[3] * 256) + data[2];
    if (yGyro > 32767) yGyro -= 65536;
    zGyro = (data[5] * 256) + data[4];
    if (zGyro > 32767) zGyro -= 65536;

    xGyro = xGyro * 0.0038;  //  Full scale = +/- 125 degree/s
    yGyro = yGyro * 0.0038;  //  Full scale = +/- 125 degree/s
    zGyro = zGyro * 0.0038;  //  Full scale = +/- 125 degree/s

    if (cnt > cnt_start_num && cnt <= cnt_start_num + cnt_total_num) {
        xGyroBiasSum += xGyro;
        yGyroBiasSum += yGyro;
        zGyroBiasSum += zGyro;
    } else {
        xGyroBiasAve = xGyroBiasSum / cnt_total_num;
        yGyroBiasAve = yGyroBiasSum / cnt_total_num;
        zGyroBiasAve = zGyroBiasSum / cnt_total_num;

        xGyro -= xGyroBiasAve;  //  Full scale = +/- 125 degree/s
        yGyro -= yGyroBiasAve;  //  Full scale = +/- 125 degree/s
        zGyro -= zGyroBiasAve;  //  Full scale = +/- 125 degree/s
    }
}

void imu_bmx055::calculate_mag() {
    int data[8];
    for (int i = 0; i < 8; i++) {
        Wire.beginTransmission(Addr_Mag);
        Wire.write((0x42 + i));  // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_Mag, 1);  // Request 1 byte of data
        // Read 6 bytes of data
        // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
        if (Wire.available() == 1) data[i] = Wire.read();
    }
    // Convert the data
    xMag = ((data[1] << 8) | (data[0] >> 3));
    if (xMag > 4095) xMag -= 8192;
    yMag = ((data[3] << 8) | (data[2] >> 3));
    if (yMag > 4095) yMag -= 8192;
    zMag = ((data[5] << 8) | (data[4] >> 3));
    if (zMag > 16383) zMag -= 32768;
}

void imu_bmx055::calculate_attitude() {
    cnt++;
    calculate_accel();
    calculate_gyro();
    //print_attitude_data();
    calculate_mag();
    madgwick.update(xGyro,yGyro,zGyro,xAccl,yAccl,zAccl,xMag,yMag,zMag);
    //madgwick.updateIMU(xGyro,yGyro,zGyro,xAccl,yAccl,zAccl);
    roll  = madgwick.getRoll();
    pitch = madgwick.getPitch();
    yaw   = madgwick.getYaw() - 180.0f;

/*
    if (cnt > cnt_start_num + cnt_total_num && cnt <= cnt_start_num + cnt_total_num + cnt_yaw_num) {
        yawBiasSum += yaw;
    } else {
        yawBiasAve = yawBiasSum / cnt_yaw_num;
        yaw -= yawBiasAve;
    }
    */

/*
    Serial.print("roll: ");
    Serial.print(roll);
    Serial.print(", pitch: ");
    Serial.print(pitch);
    Serial.print(", yaw: ");
    Serial.println(yaw);
    */
}