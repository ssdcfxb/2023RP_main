/**
 * @file        imu_protocol.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       Imu Protocol.
 * @update      
 *              v1.0(9-September-2020)
 *              v1.1(24-October-2021)
 *                  1.修改imu_potocol.c/.h->imu_protocol.c/.h 
 */
 
/* Includes ------------------------------------------------------------------*/
#include "imu_protocol.h"

#include "bmi.h"

#include "imu_sensor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ave_filter_t imu_pitch_dif_speed_ave_filter;
ave_filter_t imu_roll_dif_speed_ave_filter;
ave_filter_t imu_yaw_dif_speed_ave_filter;
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
short gyrox, gyroy, gyroz;
short accx, accy, accz;
float gyrox_, gyroy_, gyroz_;
float accx_, accy_, accz_;
float pitch_, roll_, yaw_;
void imu_sensor_update(imu_sensor_t *imu_sen)
{
    imu_sensor_info_t *imu_info = imu_sen->info;
	
	/* 获取陀螺仪数据 */
  BMI_Get_RawData(&gyrox, &gyroy, &gyroz, &accx, &accy, &accz);
	
	/* 原始数据低通滤波 */
	gyrox_ = lowpass(gyrox_, gyrox, 0.3);
	gyroy_ = lowpass(gyroy_, gyroy, 0.3);
	gyroz_ = lowpass(gyroz_, gyroz, 0.3);
	accx_ = lowpass(accx_, accx, 1);
	accy_ = lowpass(accy_, accy, 1);
	accz_ = lowpass(accz_, accz, 1);
	
	/* 解算陀螺仪数据 */
  BMI_Get_EulerAngle(&imu_info->pitch, &imu_info->roll, &imu_info->yaw,\
										 &pitch_, &roll_, &yaw_, \
										 &gyrox_, &gyroy_, &gyroz_, \
										 &accx_, &accy_, &accz_);
    
	/* 计算陀螺仪数据 */
	//pitch
	imu_info->rate_pitch = pitch_;
	if (abs(imu_info->rate_pitch) > 180.0f)
	{
		imu_info->rate_pitch -= one(imu_info->rate_pitch) * 360.0f;
	}
	// imu_info->rate_pitch *= 1000.0f
	arm_scale_f32(&imu_info->rate_pitch, 1000.0f, &imu_info->rate_pitch, 1);
	imu_info->ave_rate_pitch = ave_fil_update(&imu_pitch_dif_speed_ave_filter, imu_info->rate_pitch, 3);
	
	//roll
	imu_info->rate_roll = roll_;
	if (abs(imu_info->rate_roll) > 180.0f)
	{
		imu_info->rate_roll -= one(imu_info->rate_roll) * 360.0f;
	}
	// imu_info->rate_roll *= 1000.0f
	arm_scale_f32(&imu_info->rate_roll, 1000.0f, &imu_info->rate_roll, 1);
	imu_info->ave_rate_roll = ave_fil_update(&imu_roll_dif_speed_ave_filter, imu_info->rate_roll, 3);
	
	//yaw
	imu_info->rate_yaw = yaw_;
	if (abs(imu_info->rate_yaw) > 180.0f)
	{
		imu_info->rate_yaw -= one(imu_info->rate_yaw) * 360.0f;
	}
	// imu_info->rate_yaw *= 1000.0f
	arm_scale_f32(&imu_info->rate_yaw, 1000.0f, &imu_info->rate_yaw, 1);
	imu_info->ave_rate_yaw = ave_fil_update(&imu_yaw_dif_speed_ave_filter, imu_info->rate_yaw, 3);
	
	
	imu_sen->check(imu_sen);
}

//int8_t imu_init_errno;
void imu_sensor_init(imu_sensor_t *imu_sen)
{
    int8_t rslt;
	uint32_t tickstart = HAL_GetTick();
//	imu_sensor_info_t *imu_info = imu_sen->info;
	
	imu_sen->errno = NONE_ERR;

    rslt = BMI_Init();
	while(rslt) {
        // 如果初始化失败则重新初始化
        imu_sen->errno = DEV_INIT_ERR;
        rslt = BMI_Init();
    }
    //imu_init_errno = rslt;
    
//	for(uint16_t i=0; i<250; i++) {
//		BMI_Get_GRO(&gyrox, &gyroy, &gyroz);
//		imu_info->rate_pitch_offset += gyrox;
//		imu_info->rate_yaw_offset += gyroz;
//	}
    /**
        @note
        如果上电的时候云台运动，会导致计算出来的静态偏差数值出错。如果每次上电的时候，静态偏差均
        差别不大的话，可以直接给定一个固定值。或者，另外对计算出来的偏差值做判断等。
    */
//	imu_info->rate_pitch_offset /= 250.f;
//	imu_info->rate_yaw_offset /= 250.f;

	if(imu_sen->id != DEV_ID_IMU)
		imu_sen->errno = DEV_ID_ERR;
}
