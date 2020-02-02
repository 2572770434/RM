#include "main.h"
#define RM_IMU_QUAT_ID 0x401
#define RM_IMU_GYRO_ID 0x402
#define RM_IMU_ACCEL_ID 0x403
#define RM_IMU_MAG_ID 0x404
#define RM_IMU_PARAM_ID 0x405
//转换成 m/s^2
#define ACCEL_3G_SEN 0.0008974358974f
#define ACCEL_6G_SEN 0.00179443359375f
#define ACCEL_12G_SEN 0.0035888671875f
#define ACCEL_24G_SEN 0.007177734375f
//转换成 rad/s
#define GYRO_2000_SEN 0.00106526443603169529841533860381f
#define GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define GYRO_500_SEN 0.00026631610900792382460383465095346f
#define GYRO_250_SEN 0.00013315805450396191230191732547673f
#define GYRO_125_SEN 0.000066579027251980956150958662738366f
typedef struct
{
uint8_t quat_euler:1;
uint8_t gyro_rangle:3;
uint8_t accel_rangle:2;
uint8_t imu_sensor_rotation:5;
uint8_t ahrs_rotation_sequence:3;
int16_t quat[4];
float quat_float[4];
int16_t euler_angle[3];
float euler_angle_float[3];
float last_euler_angle_float[3];
int16_t gyro_int16[3];
int16_t accel_int16[3];
int16_t mag_int16[3];
float gyro_float[3];
float accel_float[3];
uint16_t sensor_time;
uint16_t sensor_temperature;
int16_t sensor_control_temperature;
float gyro_sen;
float accel_sen;
int round_cnt_yaw; 
int round_cnt_pitch;
float newyaw;
float lastyaw;
float newpitch;
float lastpitch;
float newroll;
int16_t gyro_yaw;
int16_t gyro_pitch;
int16_t gyro_roll;
}rm_imu_data_t;
extern rm_imu_data_t rm_imu_data;
void include_round_engle();
