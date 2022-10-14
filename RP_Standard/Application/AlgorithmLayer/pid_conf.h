#ifndef __PID_CONF_H
#define __PID_CONF_H


// M3508���PID����
#define CHAS_SP_MAX_OUT 12000.0f
#define CHAS_SP_MAX_INTEGRAL 6000.0f
#define CHAS_SP_MAX_I_OUT 6000.0f
#define CHAS_AG_MAX_OUT 10000.0f
#define CHAS_AG_MAX_INTEGRAL 3000.0f
#define CHAS_AG_MAX_I_OUT 3000.0f

#define SHOOT_SP_MAX_OUT 12000.0f
#define SHOOT_SP_MAX_INTEGRAL 6000.0f
#define SHOOT_SP_MAX_I_OUT 6000.0f
#define SHOOT_AG_MAX_OUT 10000.0f
#define SHOOT_AG_MAX_INTEGRAL 3000.0f
#define SHOOT_AG_MAX_I_OUT 3000.0f

// Plc-Inc
//#define M3508_INC_SP_KP 13.0f
//#define M3508_INC_SP_KI 0.5f
//#define M3508_INC_SP_KD 0.5f
//#define M3508_INC_AG_KP 60.0f
//#define M3508_INC_AG_KI 0.0f
//#define M3508_INC_AG_KD 1.0f

// Plc-Plc
#define CHAS_SP_KP 10.0f
#define CHAS_SP_KI 0.33f
#define CHAS_SP_KD 0.0f
#define CHAS_AG_KP 25.0f
#define CHAS_AG_KI 0.0f
#define CHAS_AG_KD 5.0f

#define SHOOT_SP_KP 5.0f
#define SHOOT_SP_KI 0.0f
#define SHOOT_SP_KD 0.0f
#define SHOOT_AG_KP 5.0f
#define SHOOT_AG_KI 0.0f
#define SHOOT_AG_KD 0.0f

// GM6020���PID����
#define GIM_SP_MAX_OUT 20000.0f
#define GIM_SP_MAX_INTEGRAL 20000.0f
#define GIM_SP_MAX_I_OUT 6000.0f
#define GIM_AG_MAX_OUT 10000.0f
#define GIM_AG_MAX_INTEGRAL 6000.0f
#define GIM_AG_MAX_I_OUT 6000.0f

// machineģʽ
#define YAW_MACHINE_SP_KP 11.0f//50.0f
#define YAW_MACHINE_SP_KI 1.2f//2.0f
#define YAW_MACHINE_SP_KD 0.0f
#define YAW_MACHINE_AG_KP 8.0f//12.0f
#define YAW_MACHINE_AG_KI 0.0f
#define YAW_MACHINE_AG_KD 0.0f

#define PITCH_MACHINE_SP_KP 20.0f
#define PITCH_MACHINE_SP_KI 1.3f
#define PITCH_MACHINE_SP_KD 0.0f
#define PITCH_MACHINE_AG_KP 8.0f
#define PITCH_MACHINE_AG_KI 0.0f
#define PITCH_MACHINE_AG_KD 0.0f

// gyroģʽ
#define YAW_GYRO_SP_KP 80.0f
#define YAW_GYRO_SP_KI 1.0f
#define YAW_GYRO_SP_KD 0.0f
#define YAW_GYRO_AG_KP 200.0f
#define YAW_GYRO_AG_KI 0.0f
#define YAW_GYRO_AG_KD 0.0f

#define PITCH_GYRO_SP_KP 25.0f
#define PITCH_GYRO_SP_KI 1.2f
#define PITCH_GYRO_SP_KD 0.0f
#define PITCH_GYRO_AG_KP 200.0f
#define PITCH_GYRO_AG_KI 0.0f
#define PITCH_GYRO_AG_KD 0.0f


// M2006���pid����
#define DIAL_SP_MAX_OUT 10000.0f
#define DIAL_SP_MAX_INTEGRAL 10000.0f
#define DIAL_SP_MAX_I_OUT 6000.0f
#define DIAL_AG_MAX_OUT 10000.0f
#define DIAL_AG_MAX_INTEGRAL 3000.0f
#define DIAL_AG_MAX_I_OUT 3000.0f

#define DIAL_SP_KP 17.0f
#define DIAL_SP_KI 1.2f
#define DIAL_SP_KD 0.0f
#define DIAL_AG_KP 50.0f
#define DIAL_AG_KI 0.0f
#define DIAL_AG_KD 0.0f


#endif
