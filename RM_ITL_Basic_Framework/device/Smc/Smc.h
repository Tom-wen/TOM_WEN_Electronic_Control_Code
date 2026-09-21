#ifndef SMC_H
#define SMC_H

#include "main.h"
#include "config.h"
#include <math.h>

//滑模控制器结构体
typedef struct
{
    //初始化相关参数
    float C;            //滑模面系数
    float K;            //指数趋近律系数
    float epsilon;      //切换趋近律系数(Sat项增益)
    float error_eps;    //误差死区阈值
    float u_max;        //控制量限幅
    float J;            //等效转动惯量(输出增益)
    float delta;        //Sat函数边界层厚度

    //目标相关
    float ref;          //当前目标值
    float refl;         //上一次目标值
    float dref;         //目标一阶差分(近似速度前馈)
    float ddref;        //目标二阶差分(近似加速度前馈)

    //读变量
    float angle;        //当前位置(°)
    float ang_vel;      //当前角速度(°/s)
    float error;        //位置误差
    float s;            //滑模面值
    float u;            //控制输出
} SMC;

void SMC_Init(SMC *smc, float C, float K, float epsilon, float error_eps, float u_max, float J, float delta);
void SMC_SetRef(SMC *smc, float reference);
void SMC_Tick(SMC *smc, float angle_now, float angle_vel, float dt);
void SMC_Clear(SMC *smc);

float SMC_Sat(float s, float delta);

#endif
