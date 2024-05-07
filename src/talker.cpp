#include <ros/ros.h>
#include <time.h>
#include <sys/types.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <qwiic_icm20948.h>

void icm20948_node() {
    // Initialize ROS node
    ros::Publisher raw_pub = nh.advertise<sensor_msgs::Imu>("icm20948/raw", 10);
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("icm20948/mag", 10);
    ros::init(argc, argv, "icm20948");
    ros::NodeHandle nh;

    ros::Rate rate(100);
    ROS_INFO("%s icm20948 node launched.", ros::this_node::getName().c_str());

    QwiicIcm20948 IMU;

    while (!IMU.connected) {
        ROS_INFO("The Qwiic ICM20948 device isn't connected to the system. Please check your connection");
    }

    IMU.begin();

    while (ros::ok()) {
        if (IMU.dataReady()) {
            IMU.getAgmt();
            sensor_msgs::Imu raw_msg;
            raw_msg.header.stamp = ros::Time::now();

            raw_msg.orientation.w = 0;
            raw_msg.orientation.x = 0;
            raw_msg.orientation.y = 0;
            raw_msg.orientation.z = 0;

            raw_msg.linear_acceleration.x = IMU.axRaw;
            raw_msg.linear_acceleration.y = IMU.ayRaw;
            raw_msg.linear_acceleration.z = IMU.azRaw;

            raw_msg.angular_velocity.x = IMU.gxRaw;
            raw_msg.angular_velocity.y = IMU.gyRaw;
            raw_msg.angular_velocity.z = IMU.gzRaw;

            raw_msg.orientation_covariance[0] = -1;
            raw_msg.linear_acceleration_covariance[0] = -1;
            raw_msg.angular_velocity_covariance[0] = -1;

            raw_pub.publish(raw_msg);

            sensor_msgs::MagneticField mag_msg;
            mag_msg.header.stamp = ros::Time::now();
            mag_msg.magnetic_field.x = IMU.mxRaw;
            mag_msg.magnetic_field.y = IMU.myRaw;
            mag_msg.magnetic_field.z = IMU.mzRaw;
            mag_msg.magnetic_field_covariance[0] = -1;
            mag_pub.publish(mag_msg);
        }

        rate.sleep();
    }

    ROS_INFO("%s icm20948 node finished", ros::this_node::getName().c_str());
}

int main(int argc, char **argv) {
    try {
        icm20948_node();
    } catch (const ros::Exception& e) {
        ROS_ERROR("%s icm20948 node exited with exception: %s", ros::this_node::getName().c_str(), e.what());
    }
    return 0;
}

