// SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
// SPDX-License-Identifier: MIT

#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <array>
#include <vector>
#include <tuple>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define ACCEL_RANGE_2G  0
#define ACCEL_RANGE_4G  1
#define ACCEL_RANGE_8G  2
#define ACCEL_RANGE_16G 3

#define GYRO_RANGE_250  0
#define GYRO_RANGE_500  1
#define GYRO_RANGE_1000 2
#define GYRO_RANGE_2000 3

class ICM20948 {
public:
    ICM20948(int i2c_bus) {
        i2c_fd = open("/dev/i2c-" + std::to_string(i2c_bus), O_RDWR);
        if (i2c_fd < 0) {
            std::cerr << "Failed to open I2C bus" << std::endl;
            exit(1);
        }
        if (ioctl(i2c_fd, I2C_SLAVE, 0x69) < 0) {
            std::cerr << "Failed to select I2C device" << std::endl;
            close(i2c_fd);
            exit(1);
        }
        accel_range = ACCEL_RANGE_4G;
        gyro_range = GYRO_RANGE_500;
    }

    ~ICM20948() {
        close(i2c_fd);
    }

    std::tuple<float, float, float> getAcceleration() {
        uint8_t data[6];
        if (read(i2c_fd, data, 6) != 6) {
            std::cerr << "Failed to read acceleration data" << std::endl;
            return {0.0f, 0.0f, 0.0f};
        }
        int16_t ax = (data[0] << 8) | data[1];
        int16_t ay = (data[2] << 8) | data[3];
        int16_t az = (data[4] << 8) | data[5];
        float scale = (accel_range == ACCEL_RANGE_2G) ? 0.061f : 
                      (accel_range == ACCEL_RANGE_4G) ? 0.122f :
                      (accel_range == ACCEL_RANGE_8G) ? 0.244f : 0.488f;
        return {static_cast<float>(ax) * scale, static_cast<float>(ay) * scale, static_cast<float>(az) * scale};
    }

    std::tuple<float, float, float> getGyro() {
        uint8_t data[6];
        if (read(i2c_fd, data, 6) != 6) {
            std::cerr << "Failed to read gyro data" << std::endl;
            return {0.0f, 0.0f, 0.0f};
        }
        int16_t gx = (data[0] << 8) | data[1];
        int16_t gy = (data[2] << 8) | data[3];
        int16_t gz = (data[4] << 8) | data[5];
        float scale = (gyro_range == GYRO_RANGE_250) ? 0.007633f :
                      (gyro_range == GYRO_RANGE_500) ? 0.015267f :
                      (gyro_range == GYRO_RANGE_1000) ? 0.030487f : 0.060975f;
        return {static_cast<float>(gx) * scale, static_cast<float>(gy) * scale, static_cast<float>(gz) * scale};
    }

    std::tuple<float, float, float> getMagnetometer() {
        uint8_t data[6];
        if (read(i2c_fd, data, 6) != 6) {
            std::cerr << "Failed to read magnetometer data" << std::endl;
            return {0.0f, 0.0f, 0.0f};
        }
        int16_t mx = (data[0] << 8) | data[1];
        int16_t my = (data[2] << 8) | data[3];
        int16_t mz = (data[4] << 8) | data[5];
        return {static_cast<float>(mx), static_cast<float>(my), static_cast<float>(mz)};
    }

    void setAccelRange(int range) {
        accel_range = range;
    }

    void setGyroRange(int range) {
        gyro_range = range;
    }

private:
    int i2c_fd;
    int accel_range;
    int gyro_range;
};

class Madgwick {
public:
    Madgwick() : q{1.0f, 0.0f, 0.0f, 0.0f}, beta(0.1f) {}

    Eigen::Quaternionf updateMARG(const Eigen::Vector3f& acc, const Eigen::Vector3f& gyr, const Eigen::Vector3f& mag) {
        float q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
        float ax = acc.x(), ay = acc.y(), az = acc.z();
        float gx = gyr.x(), gy = gyr.y(), gz = gyr.z();
        float mx = mag.x(), my = mag.y(), mz = mag.z();

        float q_dot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        float q_dot2 = 0.5f * (q0 * gx - q3 * gy + q2 * gz);
        float q_dot3 = 0.5f * (q3 * gx + q0 * gy - q1 * gz);
        float q_dot4 = 0.5f * (-q2 * gx + q1 * gy + q0 * gz);

        q.w() += q_dot1 * Dt;
        q.x() += q_dot2 * Dt;
        q.y() += q_dot3 * Dt;
        q.z() += q_dot4 * Dt;

        q.normalize();

        return q;
    }

    float Dt = 0.01f;
    float beta;
    Eigen::Quaternionf q;
};

int main() {
    int i2c_bus = 1;
    ICM20948 icm(i2c_bus);
    icm.setAccelRange(ACCEL_RANGE_4G);
    std::cout << "Acceleration Range: " << (icm.accel_range + 1) * 2 << "G" << std::endl;

    int gyro_range;
    switch (icm.gyro_range) {
        case GYRO_RANGE_250:
            gyro_range = 250;
            break;
        case GYRO_RANGE_500:
            gyro_range = 500;
            break;
        case GYRO_RANGE_1000:
            gyro_range = 1000;
            break;
        case GYRO_RANGE_2000:
            gyro_range = 2000;
            break;
        default:
            gyro_range = 0;
            break;
    }
    std::cout << "Gyro Range: " << gyro_range << " degrees/s" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    float frequency = 100.0f;
    Madgwick madgwick;

    while (true) {
        auto acc_data = icm.getAcceleration();
        auto gyr_data = icm.getGyro();
        auto mag_data = icm.getMagnetometer();

        std::cout << "Acceleration: X:" << std::get<0>(acc_data) << ", Y:" << std::get<1>(acc_data) << ", Z:" << std::get<2>(acc_data) << " m/s^2" << std::endl;
        std::cout << "Gyro: X:" << std::get<0>(gyr_data) << ", Y:" << std::get<1>(gyr_data) << ", Z:" << std::get<2>(gyr_data) << " rad/s" << std::endl;
        std::cout << "Magnetometer: X:" << std::get<0>(mag_data) << ", Y:" << std::get<1>(mag_data) << ", Z:" << std::get<2>(mag_data) << " uT" << std::endl;

        madgwick.Dt = 1.0f / frequency;
        madgwick.q = madgwick.updateMARG(Eigen::Vector3f(acc_data), Eigen::Vector3f(gyr_data), Eigen::Vector3f(mag_data));

        std::cout << "Quaternion Orientation: " << madgwick.q.w() << ", " << madgwick.q.x() << ", " << madgwick.q.y() << ", " << madgwick.q.z() << std::endl;

        Eigen::Vector3f euler = madgwick.q.toRotationMatrix().eulerAngles(2, 1, 0);
        std::cout << "Euler Orientation: " << euler.z() * 180.0f / M_PI << ", " << euler.y() * 180.0f / M_PI << ", " << euler.x() * 180.0f / M_PI << " degrees" << std::endl;

        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0f / frequency)));
    }

    return 0;
}

