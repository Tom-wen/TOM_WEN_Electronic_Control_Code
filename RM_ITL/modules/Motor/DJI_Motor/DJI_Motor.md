DJI_Motor.c
大疆电机控制模式
1.各类模式适用的PID：
（1）DJI6020_SpdClose_mode 角速度闭环模式，使用单环角速度环 
     PID_Init(&motor_instance2[i].control_data->set_pid, 850.0, 0.7, 0.1, 15000, 20000);//单环角速度PID

（2）DJI6020_SpdClose_mode2 串级PID角速度为外环，内环为电流环
     PID_Init(&motor_instance2[i].control_data->set_pid, 850.0, 0.7, 0.1, 15000, 20000); //外环角速度
     PID PID_Init(&motor_instance2[i].control_data->cascade_pid.inner, 1.0, 0.0, 0.0, 1000, 20000);//串级PID的内环，电流环

（3）DJI6020_PosSpdClose_mode2 6020单圈位置速度模式， 外环为位置环，内环为角速度环： 
     PID_Init(&motor_instance2[i].control_data->cascade_pid.inner, 300.0, 0.0, 0.0, 1000, 20000);//串级PID的内环，角速度环， 
     PID_Init(&motor_instance2[i].control_data->cascade_pid.outer, 1.0, 0.0, 0.0, 50, 100);//串级PID的外环，位置环

（4）DJI6020_PosSpdClose_mode 6020位置速度模式，单圈绝对值控制，串级PID
     DJI6020_PosSpdClose_MultiTurn 6020位置速度模式，多圈位置控制，串级PID

     PID_Init(&Arm_motor[i].control_data->cascade_pid.inner, 1.0, 0.0, 0.0, 1000, 20000);//串级PID的内环，电流环
     PID_Init(&Arm_motor[i].control_data->cascade_pid.outer, 85.0, 1.5, 0.5, 10000, 20000);//串级PID的外环，速度环
     PID_Init(&Arm_motor[i].control_data->set_pid, 80.0, 0.1, 0.0, 1000, 20000);//单环角度PID，位置环
（5）DJI6020_PosClose_mode  6020单圈位置环模式，单环（角度环）
     DJI6020_PosClose_MultiTurn 6020多圈位置模式，单环（角度环）
     PID_Init(&Arm_motor[i].control_data->set_pid, 80.0, 0.1, 0.0, 1000, 20000);//单环角度PID

（6）DJI3508_SpdClose_mode //单环速度环3508控制
     PID_Init(&motor_instance1[i].control_data->set_pid, 22.0, 0.0, 1.5, 200, 5000);//单环速度环PID

（7）DJI3508_SpdClose_mode2 //串级PID3508控制，外环速度环，内环电流环
     PID_Init(&motor_instance1[i].control_data->cascade_pid.inner, 1.0, 0.0, 0.5, 670, 10000);//串级PID的内环，电流环
     PID_Init(&motor_instance1[i].control_data->cascade_pid.outer, 22.0, 0.0, 0.5, 200, 5000);//串级PID的外环，速度环  

2.电机使能，失能：用来遥控器离线控制电机停止