#include "IMU.h"

IMU::IMU() : _sda(21), _scl(22), _mpu(), _pressure_sensor() {
    _acceleration_scale = scale_2g;
    _gyroscope_scale = scale_250dps;
    _magnetometer_scale = 0.06;
}

IMU::IMU(const uint8_t sda, const uint8_t scl, const scales acceleration_scale, const scales gyroscope_scale, const double magnetometer_scale) 
    : _sda(sda), _scl(scl), _mpu(), _pressure_sensor() {
    _acceleration_scale = acceleration_scale;
    _gyroscope_scale = gyroscope_scale;
    _magnetometer_scale = magnetometer_scale;
}

IMU::begin() {
    _mpu.init();
}

Axis<int16_t> IMU::get_accelerometer_readings() {
    Axis raw_acceleration;

    _mpu.read_acc();

    raw_acceleration.x = _mpu.ax;
    raw_acceleration.y = _mpu.ay;
    raw_acceleration.z = _mpu.az;

    return raw_acceleration;
}

Axis<int16_t> IMU::get_gyroscope_readings() {
    Axis raw_rotation_rate;

    _mpu.read_gyro();

    raw_rotation_rate.x = _mpu.gx;
    raw_rotation_rate.y = _mpu.gy;
    raw_rotation_rate.z = _mpu.gz;

    return raw_rotation_rate;
}

Axis<int16_t> IMU::get_magnetometer_readings() {
    Axis raw_magnetic_flux;

    _mpu.read_mag();

    raw_magnetic_flux.x = _mpu.mx;
    raw_magnetic_flux.y = _mpu.my;
    raw_magnetic_flux.z = _mpu.mz;

    return raw_magnetic_flux;
}

Axis<double> IMU::get_acceleration() {
    Axis<double> acceleration;
    Axis<int16_t> raw_acceleration = this->get_accelerometer_readings();

    acceleration.x = this->calculate_acceleration(raw_acceleration.x);
    acceleration.y = this->calculate_acceleration(raw_acceleration.y);
    acceleration.z = this->calculate_acceleration(raw_acceleration.z);

    return acceleration;
}

Axis<double> IMU::get_rotation_rate() {
    Axis<double> rotation_rate;
    Axis<int16_t> raw_rotation_rate = this->get_gyroscope_readings();

    rotation_rate.x = this->calculate_rotation_rate(raw_rotation_rate.x);
    rotation_rate.y = this->calculate_rotation_rate(raw_rotation_rate.y);
    rotation_rate.z = this->calculate_rotation_rate(raw_rotation_rate.z);

    return rotation_rate;
}

Axis<double> IMU::get_magnetic_flux() {
    Axis<double> magnetic_flux;
    Axis<int16_t> raw_magnetic_flux = this->get_magnetometer_readings();

    magnetic_flux.x = this->calculate_magnetic_flux(raw_magnetic_flux.x);
    magnetic_flux.y = this->calculate_magnetic_flux(raw_magnetic_flux.y);
    magnetic_flux.z = this->calculate_magnetic_flux(raw_magnetic_flux.z);

    return magnetic_flux;
}

double IMU::calculate_acceleration(const int16_t raw) {
    double axis_acceleration = 1;
    switch(_acceleration_scale) {
        scale_2g:
            axis_acceleration / 16384;
            break;
        scale_4g:
            axis_acceleration / 8192;
            break;
        scale_8g:
            axis_acceleration / 4096;
            break;
        scale_16g:
            axis_acceleration / 2048;
            break;
    }

    return acceleration * _G;
}

double IMU::calculate_rotation_rate(const int16_t raw) {
    double axis_rotation_rate = 1;
    
    switch(_gyroscope_scale) {
        scale_250dps:
            axis_rotation_rate /= 131;
            break;
        scale_500dps:
            axis_rotation_rate /= 65.5;
            break;
        scale_1000dps:
            axis_rotation_rate /= 32.8;
            break;
        scale_2000dps:
            axis_rotation_rate /= 16.4;
            break;
    }

    return axis_rotation_rate;
}

double IMU::calculate_magnetic_flux(const int16_t raw) {
    // It lacks sensitivity parameter from example sketch.
    return (raw * _magnetometer_scale) / 0.6;
}