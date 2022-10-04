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
#define GM6020_SP_KP 30.0f
#define GM6020_SP_KI 0.0f
#define GM6020_SP_KD 8.0f
#define GM6020_AG_KP 30.0f
#define GM6020_AG_KI 0.0f
#define GM6020_AG_KD 8.0f



#endif
