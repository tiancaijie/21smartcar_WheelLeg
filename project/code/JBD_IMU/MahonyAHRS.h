//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MAHONYAHRS_H
#define MAHONYAHRS_H

//----------------------------------------------------------------------------------------------------
// Variable declaration
typedef struct
{
    //原陀螺仪坐标系下
    float gx_RowFrame;      //国际单位
    float gy_RowFrame;
    float gz_RowFrame;

    float ax_RowFrame;
    float ay_RowFrame;
    float az_RowFrame;
    //以下为定义的正确原陀螺仪坐标系下
    float gx;               //国际单位
    float gy;
    float gz;
    //滤波值
    float gx_Filt;
    float gy_Filt;
    float gz_Filt;
    /*零飘*/
    float gxOffset;         //国际单位
    float gyOffset;
    float gzOffset;

    float gxOffsetSum;   //国际单位
    float gyOffsetSum;
    float gzOffsetSum;

    unsigned char offsetErrCount;

    float ax;
    float ay;
    float az;
    //滤波值
    float ax_Filt;
    float ay_Filt;
    float az_Filt;

    float q[4];
    //----------------调用以下四个数据获得姿态角-------------------
    float roll_mahony;
    float pitch_mahony;
    float yaw_mahony;
    float yaw_trans;

    float roll_Filter;
    float pitch_Filter;
    float yaw_Filter;
    
    float sum_yaw_mahony;
    float yaw_mahony_Last;
    int   yaw_mahony_turns;
    //--------------------------------------------------------------
}IMUDataType;

extern IMUDataType IMUData;

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations
void AHRS_init(float quat[4], float accel[3], float mag[3]);
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);
void get_angle(float q[4], float *yaw, float *pitch, float *roll);
void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void get_angle_e(float q[4], float *yaw, float *pitch, float *roll);
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
