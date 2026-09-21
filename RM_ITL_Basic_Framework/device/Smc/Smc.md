# SMC 使用说明

本文档基于当前仓库中的 [`smc.h`](./smc.h) 和 [`smc.c`](./smc.c) 实现说明如何接入和使用 SMC（滑模控制）算法。

## 1. 文件位置

- `device/Smc/smc.h`
- `device/Smc/smc.c`

当前 `SMC` 结构体已经被挂在 `MotorControlData` 里：

```c
SMC smc;
```

定义位置见 `modules/Motor/motor_types.h`。

## 2. 对外接口

SMC 模块对外只有 5 个接口：

```c
void SMC_Init(SMC *smc, float C, float K, float epsilon, float error_eps,
              float u_max, float J, float delta);
void SMC_SetRef(SMC *smc, float reference);
void SMC_Tick(SMC *smc, float angle_now, float angle_vel, float dt);
void SMC_Clear(SMC *smc);
float SMC_Sat(float s, float delta);
```

正常使用时主要只需要：

1. `SMC_Init`
2. `SMC_SetRef`
3. `SMC_Tick`（**注意：现在需要传入 `dt`，单位秒**）
4. 读取 `smc->u`

## 3. 控制器内部状态

`SMC` 结构体主要成员含义如下。

### 初始化参数

- `C`：滑模面系数
- `K`：指数趋近项增益
- `epsilon`：切换项增益，对应 `Sat(s)`
- `error_eps`：误差死区，小于该误差时直接输出 0
- `u_max`：输出限幅（必须 > 0，否则自动回退到 1.0）
- `J`：总输出增益（必须 != 0，否则自动回退到 1.0）
- `delta`：饱和函数 `Sat(s)` 的边界层厚度

### 运行中状态

- `ref`：当前目标
- `refl`：上一拍目标
- `dref`：目标速度前馈（°/s）
- `ddref`：目标加速度前馈（°/s²）
- `angle`：当前位置
- `ang_vel`：当前角速度
- `error`：位置误差
- `s`：滑模面
- `u`：最终控制输出

## 4. 控制律

核心计算如下：

```c
error = angle - ref;

dref  = (ref - refl) / dt;                    // 单位: °/s
ddref = ((ref - refl) / dt - dref_prev) / dt; // 单位: °/s²

s = C * error + (ang_vel - dref);

u = J * (ddref
       - C * (ang_vel - dref)
       - epsilon * SMC_Sat(s, delta)
       - K * s);
```

随后对 `u` 做限幅和一阶低通滤波（alpha = 0.85）：

```c
u in [-u_max, +u_max]
u = 0.85 * u_prev + 0.15 * u_raw
```

如果 `fabs(error) < error_eps`，则直接：

```c
u = 0, dref = 0, ddref = 0
```

## 5. 调用顺序

每个控制周期按下面顺序调用：

```c
SMC_SetRef(&xxx->smc, target_angle);
SMC_Tick(&xxx->smc, current_angle, current_angle_vel, dt);
output = xxx->smc.u;
```

`dt` 是本次控制周期的时间间隔，单位秒。固定周期任务直接传常量即可，例如 1 kHz 任务传 `0.001f`。

## 6. 最小接入示例

### 初始化

```c
SMC_Init(&motors[0].motor_data->smc,
         20.0f,      // C
         30.0f,      // K
         0.0f,       // epsilon
         0.001f,     // error_eps (deg)
         10000.0f,   // u_max
         0.8f,       // J
         0.5f);      // delta
```

### 控制周期内调用

```c
#define CONTROL_DT 0.001f  // 1 kHz 任务

SMC_SetRef(&motors[0].motor_data->smc, yaw_target);
SMC_Tick(&motors[0].motor_data->smc, ins->Yaw, ins->Gyro[Zt] * hudu, CONTROL_DT);
motors[0].motor_data->target_current = motors[0].motor_data->smc.u;
```

说明：

- `ins->Yaw` 单位 `deg`
- `ins->Gyro[Zt]` 单位 `rad/s`，乘 `hudu`（180/π）转换为 `deg/s`
- 角度和角速度单位必须一致，推荐统一用 `deg` / `deg/s`

## 7. 参数说明

### 7.1 `C`

出现在滑模面里：

```c
s = C * error + (ang_vel - dref);
```

- `C` 越大，位置误差在滑模面中的权重越高，系统更快把状态拉回滑模面
- 太大时容易放大角速度噪声

### 7.2 `K`

出现在控制律中的线性趋近项：

```c
-K * s
```

- 决定沿滑模面收敛的快慢
- 增大后响应更快，但过大时可能带来振荡或电流抖动

### 7.3 `epsilon`

出现在切换项：

```c
-epsilon * Sat(s, delta)
```

- 增大后抗扰能力更强，同时也更容易带来抖振
- 建议先设为 `0`，只用线性趋近项调稳后再加入

### 7.4 `delta`

`Sat(s, delta)` 的边界层厚度：

- `delta <= 0` 时，`Sat` 退化成 `sign`
- `delta > 0` 时，在边界层内用线性段代替硬切换，抖振更小
- 只有 `epsilon > 0` 时，`delta` 才真正影响主控制输出

### 7.5 `error_eps`

误差死区（单位与 `angle`/`ref` 一致，即 `deg`）：

- 误差小于该值时直接输出 0，避免目标附近来回抖动
- 死区太大时稳态精度变差

### 7.6 `u_max`

输出限幅，最终 `u` 不会超过这个绝对值。按电机和驱动允许范围设置。

### 7.7 `J`

总输出缩放系数，把控制律结果整体放大或缩小，相当于从"期望角加速度趋势"到"最终执行量"的换算增益。

## 8. 推荐接入步骤

### 第一步：初始化

```c
SMC_Init(&motor->smc, C, K, epsilon, error_eps, u_max, J, delta);
```

### 第二步：每周期更新目标

```c
SMC_SetRef(&motor->smc, ref);
```

### 第三步：喂入反馈

```c
SMC_Tick(&motor->smc, angle_now, angle_vel, dt);
```

### 第四步：取输出

```c
motor->target_current = motor->smc.u;
```

### 第五步：切模式时清状态

控制模式切换、重置云台、失能再使能时：

```c
SMC_Clear(&motor->smc);
```

## 9. 调参建议

建议从保守配置开始，不要一上来就开强切换项。

### 推荐顺序

1. 先把 `epsilon` 设为 `0`
2. 只调 `C` 和 `K`
3. 再根据输出量级调整 `J`
4. 最后如果需要更强抗扰，再小幅加入 `epsilon`
5. 如果加了 `epsilon` 后抖振明显，再适当增大 `delta`

### 参考起步值

```c
C         = 20.0f
K         = 30.0f
epsilon   = 0.0f
error_eps = 0.001f   // deg
u_max     = 10000.0f
J         = 0.8f
delta     = 0.5f
```

这组参数来自 `app/gimbal/robotic_swing_gimbal.c`，适合作为起点，不代表对所有轴都合适。

**注意**：由于 `dref`/`ddref` 现在已经除以 `dt` 统一了单位，如果你之前在旧版本上调好了参数，切换到新版本后 `J`、`C`、`K` 通常需要重新调整。

## 10. 注意事项

### 10.1 `dt` 必须与实际控制周期一致

`SMC_Tick` 的第四个参数 `dt` 是本次控制周期的时间间隔（秒）。

- 固定周期任务直接传常量，例如 1 kHz 传 `0.001f`
- 如果传入 `dt <= 0`，代码内部会自动回退到 `1e-3f`，但这不是正常用法
- 如果任务周期变化（例如从 1 ms 改成 2 ms），只需修改传入的 `dt`，参数 `C`/`K`/`J` 的物理意义不变

### 10.2 输入单位必须统一

`angle_now`、`ref`、`angle_vel`、`dref` 在同一套公式里计算，单位必须一致。

推荐统一为：

- 角度：`deg`
- 角速度：`deg/s`

如果角速度传 `rad/s` 而角度传 `deg`，控制效果会失真。

### 10.3 误差符号定义是 `angle - ref`

```c
error = angle - ref;
```

这和很多控制器常见的 `ref - angle` 相反。如果发现输出方向不对，先检查误差符号和电机正方向是否一致。

### 10.4 小误差时直接输出 0

这是硬清零，不是平滑衰减。如果快到目标时突然没力，或目标附近有静差，优先检查 `error_eps` 是否过大。

## 11. 完整使用模板

```c
#define CONTROL_DT 0.001f  // 1 kHz

void Some_Init(MotorInstance *motor)
{
    SMC_Init(&motor->motor_data->smc,
             20.0f,      // C
             30.0f,      // K
             0.0f,       // epsilon
             0.001f,     // error_eps (deg)
             10000.0f,   // u_max
             0.8f,       // J
             0.5f);      // delta
}

void Some_Control(MotorInstance *motor, float ref_deg, float pos_deg, float vel_rad_s)
{
    float vel_deg_s = vel_rad_s * hudu;  // rad/s -> deg/s

    SMC_SetRef(&motor->motor_data->smc, ref_deg);
    SMC_Tick(&motor->motor_data->smc, pos_deg, vel_deg_s, CONTROL_DT);

    motor->motor_data->target_current = motor->motor_data->smc.u;
}
```

## 12. 一句话总结

1. 初始化参数
2. 每周期设置目标
3. 输入当前位置、角速度和控制周期 `dt`
4. 读取 `smc.u` 作为控制输出

单位必须统一（推荐 `deg` / `deg/s`），`dt` 必须与实际任务周期一致。
