#ifndef __PID_CONF_H
#define __PID_CONF_H

#define SP_MAX_OUT 20000.0f
#define SP_MAX_INTEGRAL 6000.0f  //Plc
#define SP_MAX_INC_OUT 6000.0f     //Inc
#define AG_MAX_OUT 10000.0f
#define AG_MAX_INTEGRAL 3000.0f  //Plc
#define AG_MAX_INC_OUT 10000.0f

// M3508���PID����
// Plc-Inc
#define M3508_INC_SP_KP 13.0f
#define M3508_INC_SP_KI 0.5f
#define M3508_INC_SP_KD 0.5f
#define M3508_INC_AG_KP 60.0f
#define M3508_INC_AG_KI 0.0f
#define M3508_INC_AG_KD 1.0f
// Plc-Plc
#define M3508_SP_KP 10.0f
#define M3508_SP_KI 0.33f
#define M3508_SP_KD 0.0f
#define M3508_AG_KP 25.0f
#define M3508_AG_KI 0.0f
#define M3508_AG_KD 5.0f


// GM6020���PID����
// machineģʽ
#define YAW_MACHINE_SP_KP 8.0f//30.0f
#define YAW_MACHINE_SP_KI 0.0f
#define YAW_MACHINE_SP_KD 0.0f//8.0f
#define YAW_MACHINE_AG_KP 3.0f//30.0f
#define YAW_MACHINE_AG_KI 0.0f
#define YAW_MACHINE_AG_KD 0.0f//8.0f

#define PITCH_MACHINE_SP_KP 8.0f//30.0f
#define PITCH_MACHINE_SP_KI 0.0f
#define PITCH_MACHINE_SP_KD 0.0f//8.0f
#define PITCH_MACHINE_AG_KP 3.0f//30.0f
#define PITCH_MACHINE_AG_KI 0.0f
#define PITCH_MACHINE_AG_KD 0.0f//8.0f

// gyroģʽ
#define YAW_GYRO_SP_KP 80.0f
#define YAW_GYRO_SP_KI 1.0f
#define YAW_GYRO_SP_KD 0.0f
#define YAW_GYRO_AG_KP 200.0f
#define YAW_GYRO_AG_KI 0.0f
#define YAW_GYRO_AG_KD 0.0f

#define PITCH_GYRO_SP_KP 20.0f
#define PITCH_GYRO_SP_KI 1.2f
#define PITCH_GYRO_SP_KD 0.0f
#define PITCH_GYRO_AG_KP 200.0f
#define PITCH_GYRO_AG_KI 0.0f
#define PITCH_GYRO_AG_KD 0.0f


#endif