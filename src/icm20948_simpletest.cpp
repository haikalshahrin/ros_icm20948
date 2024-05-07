#include <iostream>
#include <cstdlib>
#include <ctime>
#include "qwiic_icm20948.h"

void runExample() {
    std::cout << "\nSparkFun 9DoF ICM-20948 Sensor  Example 1\n" << std::endl;
    QwiicIcm20948 IMU;

    if (!IMU.connected) {
        std::cerr << "The Qwiic ICM20948 device isn't connected to the system. Please check your connection" << std::endl;
        return;
    }

    IMU.begin();

    while (true) {
        if (IMU.dataReady()) {
            IMU.getAgmt(); // read all axis and temp from sensor, note this also updates all instance variables
            std::cout << "Acceleration: X:" << IMU.axRaw << ", Y: " << IMU.ayRaw << ", Z: " << IMU.azRaw << " m/s^2" << std::endl;
            std::cout << "Gyro X:" << IMU.gxRaw << ", Y: " << IMU.gyRaw << ", Z: " << IMU.gzRaw << " rads/s" << std::endl;
            std::cout << "Magnetometer X:" << IMU.mxRaw << ", Y: " << IMU.myRaw << ", Z: " << IMU.mzRaw << " uT" << std::endl;
            std::cout << "" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        } else {
            std::cout << "Waiting for data" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

int main() {
    try {
        runExample();
    } catch (const std::exception& e) {
        std::cerr << "\nEnding Example 1" << std::endl;
        return 1;
    }
    return 0;
}

