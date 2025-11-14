#ifndef _VMC_H
#define _VMC_H

#include "stm32f10x.h"




//VMC数据结构
typedef struct {
    //腿长参数
    float l1;   // 上小腿长度 (mm)
    float l2;   // 上大腿长度 (mm)
    float l3;   // 下大腿长度 (mm)
    float l4;   // 下小腿长度 (mm)
    float l5;   // 髋部偏移量 (mm)
    
    //坐标数据（目标与实际）
    float target_x;   // 目标X坐标 (mm)
    float target_y;   // 目标Y坐标 (mm)
    float actual_x;   // 实际X坐标 (mm)
    float actual_y;   // 实际Y坐标 (mm)
    
    
    //力/扭矩指令
    float F0;   // 合力指令 (N)
    float Tp;   // 扭矩指令 (N·m)
    
    //VMC计算中间变量
    float XB, YB;   // B点坐标
    float XD, YD;   // D点坐标
    float lBD;      // B-D距离
    float A0, B0, C0; // 角度求解中间量
    float phi1;     // 关节1角度 (rad)
    float phi2;     // 关节2角度 (rad)
    float phi3;     // 关节3角度 (rad)
    float phi4;     // 关节4角度 (rad)
    float XC, YC;   // C点坐标
    float L0;       // C点到基准点距离
    float phi0;     // C点角度
    float j11, j12, j21, j22; // 雅可比矩阵元素
    
    //输出的关节扭矩
    float torque[2]; // 关节1和关节4的扭矩指令 (N·m)
} LegVMC_Data;


void LegVMC_Init(LegVMC_Data *data);

void LegVMC_SetTarget(LegVMC_Data *data, float target_x, float target_y);

void LegVMC_Calc(LegVMC_Data *data);

extern LegVMC_Data LEG_DATA;


#endif