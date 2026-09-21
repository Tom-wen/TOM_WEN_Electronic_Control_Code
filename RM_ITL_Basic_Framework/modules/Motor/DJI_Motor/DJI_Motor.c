#include "DJI_Motor.h"
#include "control_power.h"
 #ifdef COMPILE_M_POWER
  #include "Master_power.h"
  #endif
//DJI电机接收数据数组，支持多个电机
Motor_feedback DJI_Motor_RX[MAX_CAN][MAX_DJI_MOTORS];
// 电机ID映射表（将CAN ID映射到电机数组索引）
uint16_t dji_motor_id_map[MAX_DJI_MOTORS] = {0x201, 0x202, 0x203, 0x204, 0x205, 0x206, 0x207, 0x208, 0x209, 0x20A, 0x20B};
uint16_t bm_motor_id_map[MAX_DJI_MOTORS] = {0x97, 0x98, 0x99, 0x100, 0x101, 0x102, 0x103, 0x104, 0x105, 0x106, 0x107};
//CAN发送id更改
uint16_t id_change(MotorType motor_type, uint8_t id)
{
    uint16_t can_id = 0;
    switch (motor_type)
    {
    case Motor3508:
        if(id >= 1 && id <= 4)
        {
            can_id = 0x200;
        }
        else
        {
            can_id = 0x1FF;
        }
        return can_id;
        break;
    case Motor6020V:
        if(id >= 1 && id <= 4)
        {
            can_id = 0x1FF;
        }
        else
        {
            can_id = 0x2FF;
        }
        return can_id;
        break;
    case Motor6020C:
        if(id >= 1 && id <= 4)
        {
            can_id = 0x1FE;
        }
        else
        {
            can_id = 0x2FE;
        }
        return can_id;
        break;    
    case Motor2006:
        if(id >= 1 && id <= 4)
        {
            can_id = 0x200;
        }
        else
        {
            can_id = 0x1FF;
        }
        return can_id;
        break;  
    case MotorM15:
        if(id >= 1 && id <= 4)
        {
            can_id = 0x32;
        }
        else
        {
            can_id = 0x33;
        }
        return can_id;
        break;           
    default:
        break;
    }
    return can_id;
}

//大疆单个电机发送ID分配
uint8_t DJI_ID(uint8_t id)
{
    uint8_t motor_id = 0;
    if(id >= 1 && id <= 4)
    {
        motor_id = id - 1;
    }
    else
    {
        motor_id = id - 5;
    }
    return motor_id;
}

//大疆电机使能
void DJI_Motor_enable(MotorControlData *motors)
{
    motors->motor_enable = 1;
}
//大疆电机失能
void DJI_Motor_disable(MotorControlData *motors)
{
    motors->motor_enable = 0;
}

//开环， 四个电机为一组
void DJI3508_Spd_mode(MotorInstance *motors)
{
    uint8_t data[8] = {0};
    uint16_t can_id = 0;
    uint8_t id = 0;
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        id = DJI_ID(motors[i].motor_data->id);
        //当电机失能时，电机停止，适用于遥控器离线
        if(motors[i].motor_data->motor_enable != 0)
        {
            data[2 * id] = (int16_t)motors[i].motor_data->target_velocity >> 8;
            data[2 * id + 1] = (int16_t)motors[i].motor_data->target_velocity;            
        }
        else
        {
            data[2 * id] = 0;
            data[2 * id + 1] = 0;            
        }        
    }
    can_id = id_change(motors[0].type, motors[0].motor_data->id);
    fdcanx_send_data(motors[0].motor_data->hfdcan, can_id, data, 8);
}

//单环速度环3508控制
void DJI3508_SpdClose_mode(MotorInstance *motors)
{
    uint8_t data[8] = {0};
    uint16_t can_id = 0;
    uint8_t id = 0;
    // 计算每个电机的输出
    for (int i = 0; i < motors[0].motor_count; i++) {
        PID_Calc(&motors[i].motor_data->pid[single_loop], motors[i].motor_data->target_velocity, motors[i].motor_data->feedback->vel);
    }
    // 构造CAN数据
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        id = DJI_ID(motors[i].motor_data->id);
        if(motors[i].motor_data->motor_enable != 0)
        {
            data[2 * id] = (int16_t)motors[i].motor_data->pid[single_loop].Out >> 8;
            data[2 * id + 1] = (int16_t)motors[i].motor_data->pid[single_loop].Out;
        }
        else
        {
            data[2 * id] = 0;
            data[2 * id + 1] = 0;  
        }
    }
    can_id = id_change(motors[0].type, motors[0].motor_data->id);
    fdcanx_send_data(motors[0].motor_data->hfdcan, can_id, data, 8);
}

//PID3508控制速度环
                float decay = 0;
void DJI3508_SpdClose_mode2(MotorInstance *motors)
{
    // 计算每个电机的输出
    for (int i = 0; i < motors[0].motor_count; i++)
    {
        PID_CascadeCalc(motors[i].motor_data->pid, motors[i].motor_data->target_velocity, motors[i].motor_data->feedback->vel, (motors[i].motor_data->feedback->vel* rads));
    }
    //构造CAN数据
    uint8_t data[8] = {0};
    uint16_t can_id = 0;
    uint8_t id = 0;
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        id = DJI_ID(motors[i].motor_data->id);
        //当电机失能，电机停止，适用于遥控器离线
        if(motors[i].motor_data->motor_enable != 0)
        {
            #ifdef COMPILE_POWER
                decay = Chassis_3508_Get_Limited_Number(i);
                //int16_t output = (int16_t)(motors[i].motor_data->pid[cascade_inner].Out * decay);
                int16_t output = (int16_t)motors[i].motor_data->pid[cascade_inner].Out;
            #else
                int16_t output = (int16_t)motors[i].motor_data->pid[cascade_inner].Out;
            #endif
            data[2 * id] = output >> 8;
            data[2 * id + 1] = output; 
        }
        else
        {
            data[2 * id] = 0;
            data[2 * id + 1] = 0;            
        }
    }
    can_id = id_change(motors[0].type, motors[0].motor_data->id);
    fdcanx_send_data(motors[0].motor_data->hfdcan, can_id, data, 8);
}


#ifdef COMPILE_M_POWER
  // 带 MasterPower 功率控制的 3508 串级速度环
  void DJI3508_SpdClose_mode2_MasterPower(MotorInstance *motors)
  {
      // Step 1: PID 计算（与原函数相同）
      for (int i = 0; i < motors[0].motor_count; i++)
      {
          PID_CascadeCalc(motors[i].motor_data->pid,
                          motors[i].motor_data->target_velocity,
                          motors[i].motor_data->feedback->vel,
                          (motors[i].motor_data->feedback->vel * rads));
      }

      // Step 2: 构造 PowerObj 数组，调用 MasterPower 功率分配
      PowerObj objs[4];
      PowerObj *objPtrs[4];
      for (int i = 0; i < motors[0].motor_count; i++)
      {
          objs[i].pidOutput    = motors[i].motor_data->pid[cascade_inner].Out;
          objs[i].curAv        = motors[i].motor_data->feedback->vel * RPM_TO_RADS/19.0f;
          objs[i].setAv        = motors[i].motor_data->target_velocity * RPM_TO_RADS/19.0f;
          objs[i].pidMaxOutput = motors[i].motor_data->pid[cascade_inner].maxOutput;
          objPtrs[i]           = &objs[i];
      }
      float *limited = getControlledOutput(objPtrs);

      // Step 3: 用限制后的电流发送 CAN（替代原始 PID 输出）
      uint8_t data[8] = {0};
      uint8_t id = 0;
      for (int i = 0; i < motors[0].motor_count; i++)
      {
          id = DJI_ID(motors[i].motor_data->id);
          if (motors[i].motor_data->motor_enable != 0)
          {
              int16_t output = (int16_t)limited[i];
              data[2 * id]     = output >> 8;
              data[2 * id + 1] = output;
          }
          else
          {
              data[2 * id]     = 0;
              data[2 * id + 1] = 0;
          }
      }
      uint16_t can_id = id_change(motors[0].type, motors[0].motor_data->id);
      fdcanx_send_data(motors[0].motor_data->hfdcan, can_id, data, 8);
  }
#endif

void DJI3508_AngleSpdClose_mode2(MotorInstance *motors)
{
    // 更新各电机多圈累计角度
    for (int i = 0; i < motors[0].motor_count; i++)
    {
       update_total_angle(motors[i].motor_data);
    }
   // 串级PID计算
    for (int i = 0; i < motors[0].motor_count; i++)
   {
       PID_CascadeCalc(motors[i].motor_data->pid,
                        motors[i].motor_data->target_position,        // 外环目标：角度（度）
                        motors[i].motor_data->total_angle,            // 外环反馈：多圈累计角度（度）
                        motors[i].motor_data->feedback->vel * rads);  // 内环反馈：角速度（rad/s）
     }
    // 构造CAN数据并发送
    uint8_t data[8] = {0};
     uint16_t can_id = 0;
    uint8_t id = 0;
     for (int i = 0; i < motors[0].motor_count; i++)
    {
        id = DJI_ID(motors[i].motor_data->id);
        if (motors[i].motor_data->motor_enable != 0)
       {
             data[2 * id]     = (int16_t)motors[i].motor_data->pid[cascade_inner].Out >> 8;
            data[2 * id + 1] = (int16_t)motors[i].motor_data->pid[cascade_inner].Out;
         }
         else
        {
            data[2 * id]     = 0;
             data[2 * id + 1] = 0;
       }
  }
   can_id = id_change(motors[0].type, motors[0].motor_data->id);
   fdcanx_send_data(motors[0].motor_data->hfdcan, can_id, data, 8);
 }
void DJI3508_PosSpdClose_mode2(MotorInstance *motors)
{
    const float GEAR_RATIO = 36.0f; // 2006电机减速比 36:1
    const float NORMALIZE_THRESHOLD = 3600.0f; // 归零阈值（10圈）
    uint8_t data[8] = {0};
    uint16_t can_id = 0;
    uint8_t id = 0;
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        // 更新输出轴累计角度（除以减速比）
        float cur = motors[i].motor_data->feedback->pos / 22.75278f; // 电机轴角度 0~360
        float delta = cur - motors[i].motor_data->last_angle;
        // 处理 0/360 跳变
        if (delta > 180.0f)        delta -= 360.0f;
        else if (delta < -180.0f)  delta += 360.0f;
        // 累加到输出轴角度
        motors[i].motor_data->total_angle += delta / GEAR_RATIO;
        motors[i].motor_data->last_angle = cur;
        float target = motors[i].motor_data->target_position;  // 目标输出轴角度
        float current = motors[i].motor_data->total_angle;     // 当前输出轴角度
        // 周期性归零：当目标和当前都超过阈值时，同时减去相同值，保持误差不变
        if (target > NORMALIZE_THRESHOLD && current > NORMALIZE_THRESHOLD)
        {
            float normalize_value = floorf(fminf(target, current) / 360.0f) * 360.0f;
            motors[i].motor_data->target_position -= normalize_value;
            motors[i].motor_data->total_angle -= normalize_value;
            target -= normalize_value;
            current -= normalize_value;
        }
        else if (target < -NORMALIZE_THRESHOLD && current < -NORMALIZE_THRESHOLD)
        {
            float normalize_value = floorf(fmaxf(target, current) / -360.0f) * -360.0f;
            motors[i].motor_data->target_position -= normalize_value;
            motors[i].motor_data->total_angle -= normalize_value;
            target -= normalize_value;
            current -= normalize_value;
        }
        // 单环PID控制：位置环 （单位：rpm）
        PID_Calc(&motors[i].motor_data->pid[single_loop], target, current);
    }
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        id = DJI_ID(motors[i].motor_data->id);
        if(motors[i].motor_data->motor_enable != 0)
        {
            data[2 * id] = (int16_t)motors[i].motor_data->pid[single_loop].Out >> 8;
            data[2 * id + 1] = (int16_t)motors[i].motor_data->pid[single_loop].Out;
        }
        else
        {
            data[2 * id] = 0;
            data[2 * id + 1] = 0;
        }
    }
    can_id = id_change(motors[0].type, motors[0].motor_data->id);
    fdcanx_send_data(motors[0].motor_data->hfdcan, can_id, data, 8);
}

//开环电压6020控制
void DJI6020_Voltage_mode(MotorInstance *motors)
{
    uint8_t data[8] = {0};
    uint16_t can_id = 0;
    uint8_t id = 0;
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        id = DJI_ID(motors[i].motor_data->id);
        //当电机失能时，电机停止，适用于遥控器离线
        if(motors[i].motor_data->motor_enable != 0)
        {
            data[2 * id] = (int16_t)motors[i].motor_data->target_position >> 8;
            data[2 * id + 1] = (int16_t)motors[i].motor_data->target_position;            
        }
        else
        {
            data[2 * id] = 0;
            data[2 * id + 1] = 0;            
        }        
    }
    can_id = id_change(motors[0].type, motors[0].motor_data->id);
    fdcanx_send_data(motors[0].motor_data->hfdcan, can_id, data, 8);
}

//单环角速度环6020控制
void DJI6020_SpdClose_mode(MotorInstance *motors)
{
    uint8_t data[8] = {0};
    uint16_t can_id = 0;
    uint8_t id = 0;
    // 计算每个电机的输出
    for (int i = 0; i < motors[0].motor_count; i++) {
        PID_Calc(&motors[i].motor_data->pid[single_loop], motors[i].motor_data->target_velocity, (motors[i].motor_data->feedback->vel * rads));
    }
    // 构造CAN数据
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        id = DJI_ID(motors[i].motor_data->id);
        if(motors[i].motor_data->motor_enable != 0)
        {
            data[2 * id] = (int16_t)motors[i].motor_data->pid[single_loop].Out >> 8;
            data[2 * id + 1] = (int16_t)motors[i].motor_data->pid[single_loop].Out;
        }
        else
        {
            data[2 * id] = 0;
            data[2 * id + 1] = 0;  
        }
    }
    can_id = id_change(motors[0].type, motors[0].motor_data->id);
    fdcanx_send_data(motors[0].motor_data->hfdcan, can_id, data, 8);
}

//串级PID6020控制,外环角速度环，内环电流环
void DJI6020_SpdClose_mode2(MotorInstance *motors)
{
    // 计算每个电机的输出
    for (int i = 0; i < motors[0].motor_count; i++)
    {
        PID_CascadeCalc(motors[i].motor_data->pid, motors[i].motor_data->target_velocity, (motors[i].motor_data->feedback->vel * rads), motors[i].motor_data->feedback->current);
    }
    //构造CAN数据
    uint8_t data[8] = {0};
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        data[2 * i] = (int16_t)motors[i].motor_data->pid[cascade_inner].Out >> 8;
        data[2 * i + 1] = (int16_t)motors[i].motor_data->pid[cascade_inner].Out;
    }
    fdcanx_send_data(motors[0].motor_data->hfdcan, 0x1FF, data, 8);
}

//6020单圈位置环模式，单环（角度环）
void DJI6020_PosClose_mode(MotorInstance *motors)
{
    const float max_angle = 360;//编码器值8192.0f;

    for (int i = 0; i < motors[0].motor_count; i++) 
    {
        float current = motors[i].motor_data->feedback->pos / 22.75278;//将编码器值转为角度值0~360度
        float target = motors[i].motor_data->target_position;

        // 把目标值“拉”到 current 附近的一个连续区间内
        float delta = target - current;
        if (delta > max_angle / 2) 
        {
            target -= max_angle;
        } 
        else if (delta < -max_angle / 2) 
        {
            target += max_angle;
        }

        // 现在 target 和 current 都在同一个连续空间内，可以直接做 PID
        PID_Calc(&motors[i].motor_data->pid[single_loop], target, current);
    }

    uint8_t data[8] = {0};
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        data[2 * i] = (int16_t)motors[i].motor_data->pid[single_loop].Out >> 8;
        data[2 * i + 1] = (int16_t)motors[i].motor_data->pid[single_loop].Out;
    }
    fdcanx_send_data(motors[0].motor_data->hfdcan, 0x1FF, data, 8);
}

//6020多圈位置模式，单环（角度环）
void DJI6020_PosClose_MultiTurn(MotorInstance *motors)
{
    for (int i = 0; i < motors[0].motor_count; i++) {
        update_total_angle(motors[i].motor_data);          // 先累加总角度

        float target = motors[i].motor_data->target_position; // 目标角度
        float current = motors[i].motor_data->total_angle;    // 总角度

        PID_Calc(&motors[i].motor_data->pid[single_loop], target, current);
    }

    uint8_t data[8] = {0};
    for (int i = 0; i < motors[0].motor_count; i++) {
        data[2*i]   = (int16_t)motors[i].motor_data->pid[single_loop].Out >> 8;
        data[2*i+1] = (int16_t)motors[i].motor_data->pid[single_loop].Out;
    }
    fdcanx_send_data(motors[0].motor_data->hfdcan, 0x1FF, data, 8);
}

//6020单圈位置速度模式，双环（外环位置环，内环角速度环）
void DJI6020_PosSpdClose_mode2(MotorInstance *motors)
{
    const float max_angle = 360;//编码器值8192.0f;    
    uint8_t data[8] = {0};
    uint16_t can_id = 0;
    uint8_t id = 0;
    for(int i = 0; i < motors[0].motor_count; i++) 
    {
        float current = motors[i].motor_data->feedback->pos / 22.75278;//将编码器值转为角度值0~360度
        float target = motors[i].motor_data->target_position;

        // 多次循环处理，确保目标角度在合理范围内
        while (target - current > max_angle / 2) 
        {
            target -= max_angle;
        } 
        while (target - current < -max_angle / 2) 
        {
            target += max_angle;
        }

        // 现在 target 和 current 都在同一个连续空间内，可以直接做 PID
        PID_CascadeCalc(motors[i].motor_data->pid, target, current, motors[i].motor_data->feedback->vel);
    }

    for(int i = 0; i < motors[0].motor_count; i++)
    {
        id = DJI_ID(motors[i].motor_data->id);
        //当电机失能时，电机停止，适用于遥控器离线
        if(motors[i].motor_data->motor_enable != 0)
        {
            data[2 * id] = (int16_t)motors[i].motor_data->pid[cascade_inner].Out >> 8;
            data[2 * id + 1] = (int16_t)motors[i].motor_data->pid[cascade_inner].Out;
        }
        else
        {
            data[2 * id] = 0;
            data[2 * id + 1] = 0;            
        }

    }
    can_id = id_change(motors[0].type, motors[0].motor_data->id);
    fdcanx_send_data(motors[0].motor_data->hfdcan, can_id, data, 8);
}

//开环6020控制(电流模式)
void DJI6020_current_mode(MotorInstance *motors)
{
    uint8_t data[8] = {0};
    uint16_t can_id = 0;
    uint8_t id = 0;
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        id = DJI_ID(motors[i].motor_data->id);
        //当电机失能时，电机停止，适用于遥控器离线
        if(motors[i].motor_data->motor_enable != 0)
        {
            data[2 * id] = (int16_t)motors[i].motor_data->target_current >> 8;
            data[2 * id + 1] = (int16_t)motors[i].motor_data->target_current;            
        }
        else
        {
            data[2 * id] = 0;
            data[2 * id + 1] = 0;            
        }        
    }
    can_id = id_change(motors[0].type, motors[0].motor_data->id);
    fdcanx_send_data(motors[0].motor_data->hfdcan, can_id, data, 8);
}

//开环2006控制(电流)
void DJI2006_current_mode(MotorInstance *motors)
{
    uint8_t data[8] = {0};
    uint16_t can_id = 0;
    uint8_t id = 0;
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        id = DJI_ID(motors[i].motor_data->id);
        //当电机失能时，电机停止，适用于遥控器离线
        if(motors[i].motor_data->motor_enable != 0)
        {
            data[2 * id] = (int16_t)motors[i].motor_data->target_current >> 8;
            data[2 * id + 1] = (int16_t)motors[i].motor_data->target_current;            
        }
        else
        {
            data[2 * id] = 0;
            data[2 * id + 1] = 0;            
        }        
    }
    can_id = id_change(motors[0].type, motors[0].motor_data->id);
    fdcanx_send_data(motors[0].motor_data->hfdcan, can_id, data, 8);
}

//单环速度环2006控制
void DJI2006_SpdClose_mode(MotorInstance *motors)
{
    uint8_t data[8] = {0};
    uint16_t can_id = 0;
    uint8_t id = 0;
    // 计算每个电机的输出
    for (int i = 0; i < motors[0].motor_count; i++) {
        PID_Calc(&motors[i].motor_data->pid[single_loop], motors[i].motor_data->target_velocity, motors[i].motor_data->feedback->vel);
    }
    // 构造CAN数据
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        id = DJI_ID(motors[i].motor_data->id);
        if(motors[i].motor_data->motor_enable != 0)
        {
            data[2 * id] = (int16_t)motors[i].motor_data->pid[single_loop].Out >> 8;
            data[2 * id + 1] = (int16_t)motors[i].motor_data->pid[single_loop].Out;
        }
        else
        {
            data[2 * id] = 0;
            data[2 * id + 1] = 0;  
        }
    }
    can_id = id_change(motors[0].type, motors[0].motor_data->id);
    fdcanx_send_data(motors[0].motor_data->hfdcan, can_id, data, 8);
}

void DJI2006_PosSpdClose_mode2(MotorInstance *motors)
{
    const float GEAR_RATIO = 36.0f; // 2006电机减速比 36:1
    const float NORMALIZE_THRESHOLD = 3600.0f; // 归零阈值（10圈）
    uint8_t data[8] = {0};
    uint16_t can_id = 0;
    uint8_t id = 0;
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        // 更新输出轴累计角度（除以减速比）
        float cur = motors[i].motor_data->feedback->pos / 22.75278f; // 电机轴角度 0~360
        float delta = cur - motors[i].motor_data->last_angle;
        // 处理 0/360 跳变
        if (delta > 180.0f)        delta -= 360.0f;
        else if (delta < -180.0f)  delta += 360.0f;
        // 累加到输出轴角度
        motors[i].motor_data->total_angle += delta / GEAR_RATIO;
        motors[i].motor_data->last_angle = cur;
        float target = motors[i].motor_data->target_position;  // 目标输出轴角度
        float current = motors[i].motor_data->total_angle;     // 当前输出轴角度
        // 周期性归零：当目标和当前都超过阈值时，同时减去相同值，保持误差不变
        if (target > NORMALIZE_THRESHOLD && current > NORMALIZE_THRESHOLD)
        {
            float normalize_value = floorf(fminf(target, current) / 360.0f) * 360.0f;
            motors[i].motor_data->target_position -= normalize_value;
            motors[i].motor_data->total_angle -= normalize_value;
            target -= normalize_value;
            current -= normalize_value;
        }
        else if (target < -NORMALIZE_THRESHOLD && current < -NORMALIZE_THRESHOLD)
        {
            float normalize_value = floorf(fmaxf(target, current) / -360.0f) * -360.0f;
            motors[i].motor_data->target_position -= normalize_value;
            motors[i].motor_data->total_angle -= normalize_value;
            target -= normalize_value;
            current -= normalize_value;
        }
        // 单环PID控制：位置环 （单位：rpm）
        PID_Calc(&motors[i].motor_data->pid[single_loop], target, current);
    }
    for(int i = 0; i < motors[0].motor_count; i++)
    {                          
        id = DJI_ID(motors[i].motor_data->id);
        if(motors[i].motor_data->motor_enable != 0)
        {
            data[2 * id] = (int16_t)motors[i].motor_data->pid[single_loop].Out >> 8;
            data[2 * id + 1] = (int16_t)motors[i].motor_data->pid[single_loop].Out;
        }
        else
        {
            data[2 * id] = 0;
            data[2 * id + 1] = 0;
        }
    }
    can_id = id_change(motors[0].type, motors[0].motor_data->id);
    fdcanx_send_data(motors[0].motor_data->hfdcan, can_id, data, 8);
}

//电机总角度更新
void update_total_angle(MotorControlData *m)
{
    float cur = m->feedback->pos / 22.75278f; // 0~360
    float delta = cur - m->last_angle;

    //处理 0/360 跳变
    if (delta > 180.0f)        delta -= 360.0f;
    else if (delta < -180.0f)  delta += 360.0f;

    m->total_angle += delta;
    m->last_angle   = cur;
}
//大疆电机反馈数据解析
void DJI_motor_can_callback(CANRxData *Rx_data, CAN_PORT can_port)
{
    // 边界检查
    if(can_port >= MAX_CAN)
    {
      return;
    }
    // 查找电机索引
    uint8_t motor_index = 0;
    uint8_t found = 0;
    for(uint8_t i = 0; i < MAX_DJI_MOTORS; i++)
    {
        if(dji_motor_id_map[i] == Rx_data->id)
        {
            motor_index = i;
            found = 1;
            break;
        }
    }
    // 如果找到了对应的电机ID，解析数据
    if(found)
    {
        DJI_Motor_RX[can_port][motor_index].pos = (float)((int16_t)((Rx_data->data[0] << 8) | Rx_data->data[1]));
        DJI_Motor_RX[can_port][motor_index].vel = (float)((int16_t)((Rx_data->data[2] << 8) | Rx_data->data[3]));
        DJI_Motor_RX[can_port][motor_index].current = (float)((int16_t)((Rx_data->data[4] << 8) | Rx_data->data[5]));
        DJI_Motor_RX[can_port][motor_index].temp = (float)(Rx_data->data[6]);
        DJI_Motor_RX[can_port][motor_index].p_int = 0.0;
        DJI_Motor_RX[can_port][motor_index].v_int = 0.0;
        DJI_Motor_RX[can_port][motor_index].t_int = 0.0;
        DJI_Motor_RX[can_port][motor_index].tor = 0.0;
    }
}

//本末电机反馈数据解析
void BM_motor_can_callback(CANRxData *Rx_data, CAN_PORT can_port)
{
    // 边界检查
    if(can_port >= MAX_CAN)
    {
      return;
    }
    // 查找电机索引
    uint8_t motor_index = 0;
    uint8_t found = 0;
    for(uint8_t i = 0; i < MAX_DJI_MOTORS; i++)
    {
        if(bm_motor_id_map[i] == Rx_data->id)
        {
            motor_index = i;
            found = 1;
            break;
        }
    }
    // 如果找到了对应的电机ID，解析数据i
    if(found)
    {
        DJI_Motor_RX[can_port][motor_index].vel = (float)((int16_t)((Rx_data->data[0] << 8) | Rx_data->data[1]));
        DJI_Motor_RX[can_port][motor_index].current = (float)((int16_t)((Rx_data->data[2] << 8) | Rx_data->data[3]));
        DJI_Motor_RX[can_port][motor_index].pos = (float)((int16_t)((Rx_data->data[4] << 8) | Rx_data->data[5]));
        DJI_Motor_RX[can_port][motor_index].temp = (float)(Rx_data->data[6]);
        DJI_Motor_RX[can_port][motor_index].p_int = 0.0;
        DJI_Motor_RX[can_port][motor_index].v_int = 0.0;
        DJI_Motor_RX[can_port][motor_index].t_int = 0.0;
        DJI_Motor_RX[can_port][motor_index].tor = 0.0;
    }
}
