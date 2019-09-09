#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>

#include <sstream>

#include "spi.h"
#include "bmi088_rp_spi.h"
#include "bmi08x.h"
#include "bmi088.h"

int8_t rslt;
uint8_t data = 0;
sensor_msgs::Imu imu_msg_;


extern struct bmi08x_dev dev;
struct bmi08x_sensor_data user_accel_bmi088;
struct bmi08x_sensor_data user_gyro_bmi088;

double raw_gyro_to_rps(int16_t data)
{
    double data_d = 0;
    // For 1000dps, deg/s
    data_d = data/32768.0 * 1000;
    return data_d;
}

double raw_accel_to_mpss(int16_t data)
{
    double data_d = 0;
    // For 6g -> 6, 12g -> 12, 24g -> 24, [mg]
    data_d = data/32768.0 * 1000 *12;
    return data_d;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bmi088_imu");

    ros::NodeHandle n;

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("bmi088_imu", 1);

    ros::Rate loop_rate(10);

    int count = 0;

    rslt = bmi088_init(&dev);
    if(rslt == BMI08X_OK)
    {
        char strtemp[12];
        sprintf(strtemp, "%d", dev.accel_chip_id);
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Accel ID " << strtemp;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
    }


    /* Initialize the accelerometer*/
    /* Assign the desired configurations */
    dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
    rslt = bmi08a_set_power_mode(&dev);
    if(rslt == BMI08X_OK)
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Acc power mode set!";
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
    }
    dev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
    dev.accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ;
    dev.accel_cfg.range = BMI088_ACCEL_RANGE_12G;
    rslt = bmi08a_set_meas_conf(&dev);
    if(rslt == BMI08X_OK)
    {
        char strtemp[12];
        sprintf(strtemp, "%d %d %d", dev.accel_cfg.range,dev.accel_cfg.odr,dev.accel_cfg.bw);
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Acc meas config set: " << strtemp;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
    }
    rslt = bmi08a_get_meas_conf(&dev);
    if(rslt == BMI08X_OK)
    {
        char strtemp[12];
        sprintf(strtemp, "%d %d %d", dev.accel_cfg.range,dev.accel_cfg.odr,dev.accel_cfg.bw);
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Acc meas config get:"<< strtemp;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
    }


    /* Initialize the gyroscope*/
    dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
    rslt = bmi08g_set_power_mode(&dev);
    /* Wait for 30ms to switch between the power modes - delay taken care inside the function*/
    /* Assign the desired configurations */
    dev.gyro_cfg.odr = BMI08X_GYRO_BW_23_ODR_200_HZ;
    dev.gyro_cfg.range = BMI08X_GYRO_RANGE_1000_DPS;
    dev.gyro_cfg.bw = BMI08X_GYRO_BW_23_ODR_200_HZ;
    rslt = bmi08g_set_meas_conf(&dev);
    if(rslt == BMI08X_OK)
    {
        char strtemp[12];
        sprintf(strtemp, "%d %d %d", dev.gyro_cfg.range, dev.gyro_cfg.odr,dev.gyro_cfg.bw);
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Gyro meas config set:"<< strtemp;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
    }
    rslt = bmi08g_get_meas_conf(&dev);
    if(rslt == BMI08X_OK)
    {
        char strtemp[12];
        sprintf(strtemp, "%d %d %d", dev.gyro_cfg.range, dev.gyro_cfg.odr,dev.gyro_cfg.bw);
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Gyro meas config get:"<< strtemp;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
    }

    while (ros::ok())
    {
         /* Read the sensor data into the sensor data instance */
        rslt = bmi08a_get_data(&user_accel_bmi088, &dev);
        rslt = bmi08g_get_data(&user_gyro_bmi088, &dev);

        ros::Time current_time = ros::Time::now();

        imu_msg_.header.stamp = current_time;
        imu_msg_.angular_velocity.x = raw_gyro_to_rps(user_gyro_bmi088.x);
        imu_msg_.angular_velocity.y = raw_gyro_to_rps(user_gyro_bmi088.y);
        imu_msg_.angular_velocity.z = raw_gyro_to_rps(user_gyro_bmi088.z);
        imu_msg_.linear_acceleration.x = raw_accel_to_mpss(user_accel_bmi088.x);
        imu_msg_.linear_acceleration.y = raw_accel_to_mpss(user_accel_bmi088.y);
        imu_msg_.linear_acceleration.z = raw_accel_to_mpss(user_accel_bmi088.z);

        imu_pub.publish(imu_msg_);

        ros::spinOnce();

        loop_rate.sleep();

        ++count;
    }

    spi_close_port(0);
    spi_close_port(1);
    return 0;
}
