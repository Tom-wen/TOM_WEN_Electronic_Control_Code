# MPC_Preview3DistInitSimple 使用说明

## 1. 作用

`MPC_Preview3DistInitSimple` 是一套三状态预瞄跟踪 MPC 初始化接口，适合这类对象：

```text
x = [位置, 速度, 扰动]^T
```

离散模型写成：

```text
pos(k+1)  = pos(k) + dt * vel(k)
vel(k+1)  = A22 * vel(k) + dist(k) + B2 * u(k)
dist(k+1) = dist(k)
```

这里的 `dist` 是“常值或慢变化扰动”，可以把它理解成：

- 摩擦
- 重力偏置
- 线束阻力
- 零漂
- 模型失配

## 2. 接口说明

### 初始化接口

```c
void MPC_Preview3DistInitSimple(MPC_Preview3Dist *mpc, uint8_t N_p, uint8_t N_c,
                                float A22, float B2,
                                float Q1, float Q2, float Q3,
                                float R_u, float R_delta,
                                float Out_Max, float Delta_Out_Max,
                                float D_T, float Preview_Delay_S,
                                float Dist_Est_Alpha);
```

### 在线计算接口

```c
void MPC_Preview3DistCalcSimple(MPC_Preview3Dist *mpc,
                                float target_acc_rad_s2,
                                float target_vel_rad_s,
                                float target_pos_deg,
                                float feedback_pos_deg,
                                float feedback_vel_rad_s);
```

## 3. 当前实现里真正的模型

当前 `mpc.c` 里的实现不是泛化三状态，而是固定成了下面这个结构：

```text
A =
[1, dt, 0]
[0, A22, 1]
[0, 0,  1]

B =
[0 ]
[B2]
[0 ]
```

也就是说：

```text
位置下一拍 = 当前位置 + dt * 当前速度
速度下一拍 = A22 * 当前速度 + 当前扰动 + B2 * 当前控制
扰动下一拍 = 当前扰动
```

## 4. 单位要求

这个接口的单位是固定的，不能混：

- `target_pos_deg`：`deg`
- `feedback_pos_deg`：`deg`
- `target_vel_rad_s`：`rad/s`
- `target_acc_rad_s2`：`rad/s²`
- `feedback_vel_rad_s`：`rad/s`

内部会把速度、加速度从 `rad` 制转换到 `deg` 制再参与预测。

## 5. 参数含义

### 5.1 时域参数

- `N_p`：预测时域
- `N_c`：控制时域

常见起点：

- `N_p = 15 ~ 20`
- `N_c = 8 ~ 12`

规律：

- `N_p` 大：看得更远，但更吃模型精度
- `N_c` 大：控制更灵活，但更激进，也更耗算力

### 5.2 对象参数

- `A22`：速度衰减系数
- `B2`：控制输入到速度的增益

如果你先有连续模型：

```text
vel_dot = -a * vel + b * u + d
```

采样周期为 `dt` 时，可以换算成：

```text
A22 = exp(-a * dt)
B2  = (b / a) * (1 - exp(-a * dt))
```

### 5.3 代价函数参数

- `Q1`：位置误差权重
- `Q2`：速度误差权重
- `Q3`：扰动状态权重
- `R_u`：控制量大小惩罚
- `R_delta`：控制量变化率惩罚

调参直觉：

- `Q1` 大：更重视位置跟踪，容易为了追位置把速度打得很大
- `Q2` 大：更重视速度贴目标，位置允许更滞后
- `R_u` 大：整体更保守
- `R_delta` 大：输出更平滑，不容易猛冲

### 5.4 约束参数

- `Out_Max`：输出绝对限幅
- `Delta_Out_Max`：单周期输出变化限幅

现象对应：

- 抖、冲：减小 `Delta_Out_Max` 或增大 `R_delta`
- 太肉：增大 `Delta_Out_Max` 或减小 `R_u`

### 5.5 预瞄参数

- `D_T`：控制周期，单位 `s`
- `Preview_Delay_S`：预瞄补偿时间，单位 `s`

预瞄的作用是把当前参考提前一点：

```text
pos_ref_now = pos_target + vel_target * tau + 0.5 * acc_target * tau^2
vel_ref_now = vel_target + acc_target * tau
```

如果这个值过大，会显得很激进。

### 5.6 扰动观测器参数

- `Dist_Est_Alpha`：扰动估计更新增益

当前代码里的更新方式是：

```text
dist_hat = dist_hat + alpha * innovation
```

所以：

- `0.0`：不更新扰动
- `1.0`：每次都完整吃掉创新量

经验上：

- `0.05 ~ 0.3`：慢、稳
- `0.3 ~ 0.9`：快，但容易抖

## 6. 当前实现里的几个重要限制

这一节不是理论问题，而是当前代码实现的真实限制。

### 6.1 `Q3` 目前基本无效

虽然接口暴露了 `Q3`，但当前在线优化里只真正用了 `Q1` 和 `Q2`。

原因是：

- 第三个状态 `dist` 不可控
- Hessian 和线性项里都没有把它真正纳入可优化输出通道

所以现在改 `Q3`，基本不会改变控制结果。

### 6.2 扰动估计有硬编码限幅

当前 `mpc.c` 里把扰动估计限制在：

```text
[-50, 50] deg/s
```

这不是接口参数，而是写死在实现里的。

如果机构很重、摩擦很大、或者量纲变了，这个限幅可能会静默打满。

### 6.3 角度必须“连续”

这套算法默认参考角和反馈角在当前工作点附近是连续的。

如果你的目标角直接在 `-180 ~ 180` 之间包角跳变，预瞄 MPC 很容易被打坏。

控制层最好使用连续角，不要直接拿显示层包角值硬喂进去。

### 6.4 目标角速度必须和目标角一致

如果：

- `target_pos_deg` 的导数

和

- `target_vel_rad_s`

不一致，那么这套预瞄跟踪会天然自相矛盾，结果一定不好。

## 7. 推荐使用流程

### 第一步：先辨识 `A22` 和 `B2`

不要一开始就调 `Q/R`。

先从日志拟合对象：

- `A22`：速度自然衰减
- `B2`：控制到速度的增益

模型不准时，后面所有权重都只是在补锅。

### 第二步：给安全约束

先把：

- `Out_Max`
- `Delta_Out_Max`

设成安全值，再调性能。

### 第三步：从中等 `Q1` 和较小 `Q2` 起步

推荐顺序：

1. 先让位置不发散
2. 再逐步增加 `Q2`
3. 如果太冲，先增 `R_delta`
4. 最后再调 `Dist_Est_Alpha`

## 8. 典型初始化例子

```c
static MPC_Preview3Dist yaw_mpc = {0};

MPC_Preview3DistInitSimple(&yaw_mpc,
                           20, 10,
                           0.9920f, 0.00026f,
                           24000.0f, 900.0f, 0.0f,
                           0.0048f, 0.0010f,
                           25000.0f, 2600.0f,
                           0.002f, 0.0020f,
                           0.92f);

MPC_Preview3DistClear(&yaw_mpc);
```

## 9. 在线调用例子

```c
float target_accel_rad_s2 = ...;
float target_vel_rad_s    = ...;
float target_pos_deg      = ...;
float feedback_pos_deg    = ...;
float feedback_vel_rad_s  = ...;

MPC_Preview3DistCalcSimple(&yaw_mpc,
                           target_accel_rad_s2,
                           target_vel_rad_s,
                           target_pos_deg,
                           feedback_pos_deg,
                           feedback_vel_rad_s);

output = yaw_mpc.Out;
```

## 10. 切模态时要清零

如果发生这些情况，建议清零：

- 从遥控切到自瞄
- 从自瞄切回遥控
- 目标轨迹不连续
- 传感器刚恢复

调用：

```c
MPC_Preview3DistClear(&yaw_mpc);
```

会清掉：

- `Out`
- `Last_Out`
- 扰动估计
- 扰动观测器状态

## 11. 使用前检查清单

在怀疑算法之前，先确认：

1. `target_pos_deg` 是连续角，不是乱跳包角
2. `target_vel_rad_s` 和目标角导数一致
3. 反馈位置是 `deg`
4. 反馈速度是 `rad/s`
5. 当前控制路径真的在调用 `MPC_Preview3DistCalcSimple`
6. 当前输出真的用了 `yaw_mpc.Out`
7. `A22/B2` 是按当前电机模式和当前机构辨识出来的

## 12. 当前工程里的相关文件

- 接口声明：`device/Mpc/mpc.h`
- 算法实现：`device/Mpc/mpc.c`
- 云台侧接入点：`app/gimbal/robotic_swing_gimbal.c`
