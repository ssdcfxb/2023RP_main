#ifndef __PID_CONF_H
#define __PID_CONF_H

#define SP_MAX_OUT 10000.0f
#define SP_MAX_INTEGRAL 3000.0f  //Plc
#define SP_MAX_INC_OUT 3000.0f     //Inc
#define AG_MAX_OUT 10000.0f
#define AG_MAX_INTEGRAL 3000.0f  //Plc
#define AG_MAX_INC_OUT 10000.0f

// M3508电机PID参数
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


// GM6020电机PID参数
#define GM6020_YAW_SP_KP 8.0f//30.0f
#define GM6020_YAW_SP_KI 0.0f
#define GM6020_YAW_SP_KD 0.0f//8.0f
#define GM6020_YAW_AG_KP 3.0f//30.0f
#define GM6020_YAW_AG_KI 0.0f
#define GM6020_YAW_AG_KD 0.0f//8.0f

#define GM6020_PITCH_SP_KP 8.0f//30.0f
#define GM6020_PITCH_SP_KI 0.0f
#define GM6020_PITCH_SP_KD 0.0f//8.0f
#define GM6020_PITCH_AG_KP 3.0f//30.0f
#define GM6020_PITCH_AG_KI 0.0f
#define GM6020_PITCH_AG_KD 0.0f//8.0f
// Plc-Inc
#define GM6020_INC_SP_KP 0.0f//8.0f
#define GM6020_INC_SP_KI 0.0f//2.2f
#define GM6020_INC_SP_KD 0.0f//0.5f
#define GM6020_INC_AG_KP 0.0f//1.0f
#define GM6020_INC_AG_KI 0.0f
#define GM6020_INC_AG_KD 0.0f//2.0f



#endif
