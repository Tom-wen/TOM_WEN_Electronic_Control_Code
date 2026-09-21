框架里运用的一些小技巧：
1.函数指针：
（1）解释：
int add(int a, int b)
{
    return a + b;
}

int multiply(int a, int b)
{
    return a * b;
}

int (*operation)(int, int);//定义函数指针

operation = add;(operation = &add)//给函数指针赋值

int result = operation(3, 5);
result = 8
如果operation = multiply;(operation = &admultiplyd)//给函数指针赋值
result = 15,这就是函数指针的运行

（2）框架里函数指针用在里层与层之间的解耦：
我们在bsp层中断中接收到了数据，但是我们数据处理是在modules层中，我们不希望底层去包含高层的内容，因此我使用函数指针去实现功能，以下面can1中断数据处理为例：
我在bsp_can.h定义里函数指针，这里还把它定义为了一种类型，方便后面多次使用
// 定义回调函数指针类型
typedef void (*can_rx_callback_t)(CANRxData *rx_data);

然后我在bsp_can.c里进行声明（真正定义）
// 为每个CAN口声明独立的回调函数指针
static can_rx_callback_t can1_user_callback = NULL;//CAN1

然后我在定义了一个回调函数，这个函数是为了上层去给函数指针进行赋值
void bsp_can1_set_callback(can_rx_callback_t callback)
{
    can1_user_callback = callback;
}

在modules层的DM_Motor.c里我对函数指针进行赋值
bsp_can1_set_callback(dm_motor_can_callback);//开启回调处理函数
这样我的函数指针就是实现dm_motor_can_callback这个函数的功能

最后我在bsp_can.c里的can中断接收里进行调用函数指针，就可以对数据进行处理里
// 如果设置了用户回调函数，则调用它来处理数据
if(can1_user_callback != NULL)
{
    can1_user_callback(&Rx_data1[index]);
}

2.多态：
//创建电机（电机类型，ID，电机控制数据， 电机控制模式， 电机单环PID初始化，电机串级PID内环初始化，电机串级PID外环初始化，电机反馈解算， 电机CAN）
MotorInstance CreateMotor(uint16_t type, uint8_t id, MotorControlData *motor_data, 
                       void (*control_func)(FDCAN_HandleTypeDef *hfdcan, MotorControlData *motor_data),
                       void (*set_pid)(PID *pid, float p, float i, float d, float maxI, float maxOut), 
                       void (*set_cascade_inner)(PID *pid, float p, float i, float d, float maxI, float maxOut),
                       void (*set_cascade_outer)(PID *pid, float p, float i, float d, float maxI, float maxOut),                  
                       void (*callback_func)(CANRxData *rx_data, CAN_PORT can_port), CAN_PORT can_port)
这里通过函数指针传参
    motor.motor_control = control_func;
    motor.set_pid = set_pid;
    motor.set_cascade_inner = set_cascade_inner;
    motor.set_cascade_outer = set_cascade_outer;
调用的函数在这里被定义
//底盘电机状态更新
void chasssis_motor_updata(MotorInstance *motors1, MotorInstance *motors2)
{
    if(motors1 == NULL || motors2==NULL)
    {
        return;
    }
    //底盘3508电机给电流
    motors1->motor_control(&hfdcan2, motors1->motor_data);
    //底盘6020电机给电流
    motors2->motor_control(&hfdcan2, motors2->motor_data);
}
我在任务层里的控制电机函数统一抽象为motor_control这个接口，我只需要使用这个接口就行了。如果我需要改任务里控制电机函数，如果是传统的C语言，你需要对任务函数里所有使用这个控制函数的地方进行替换，但是如果你使用多态，我只需要在
motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], DJI3508_Spd_mode, PID_Init, PID_inner_Init, PID_outer_Init, DJI_motor_can_callback, CAN2);初始化这里对DJI3508_Spd_mode真正的控制函数进行替换就行了。
这就是多态的魅力
