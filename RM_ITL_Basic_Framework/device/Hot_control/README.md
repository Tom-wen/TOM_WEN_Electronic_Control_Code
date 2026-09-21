# Hot_control 热量控制模块

## 概述

`Hot_control` 是基于 RoboMaster 裁判系统热量数据的射速动态控制模块。它根据枪管实时热量和冷却速率，自动调节拨弹电机转速，在热量允许范围内最大化射速，同时避免热量超限导致扣血。

## 工作原理

模块将控制流程分为三个阶段：

1. **参数计算阶段** — 每轮连发的起始时刻，根据当前剩余热量 `m` 计算本次连发持续时间 `ShootTime` 和射速 `shoot_speed`
2. **连发阶段** — 在 `ShootTime` 时间内以计算出的射速持续发射
3. **冷却等待阶段** — 连发结束后，以冷却平衡速度运转，等待热量恢复到阈值后自动开启下一轮连发

当剩余热量不足以发射一发弹丸时，立即停止拨弹电机。

## 启用方法

在 `config.h` 中取消注释 `HOT_CONTROL` 宏：

```c
#define HOT_CONTROL                 // 热量控制
```

## 使用方法

### 1. 引入头文件

在 `shoot.c` 中（参照已完成的集成）：

```c
#include "Hot_control.h"
```

### 2. 在发射模式中调用

在 `RemoteControlShoot()` 的 `FIRE_MODE` 分支中，用 `hot_control()` 替换固定拨弹速度：

```c
case FIRE_MODE:
    Shoot_Cmd->left_speed  = -6450;   // 左摩擦轮速度
    Shoot_Cmd->right_speed =  6450;   // 右摩擦轮速度
#ifdef HOT_CONTROL
    hot_control();                     // 热量自适应控制
#else
    Shoot_Cmd->trigger_speed = 2300;  // 固定拨弹速度（无热量控制时）
#endif
    break;
```

> `hot_control()` 内部直接修改全局变量 `shoot_cmd_send.trigger_speed`，无需传参。

### 3. 可调参数

所有参数在 `Hot_control.c` 顶部通过宏定义，按实际机构调整：

| 宏定义 | 默认值 | 说明 |
|--------|--------|------|
| `HOT_HEAT_PER_SHOT` | 10.0f | 单发弹丸热量增量（17mm 为 10） |
| `HOT_SHOOT_SPEED_MAX` | 10.0f | 最大射速上限（发/秒） |
| `HOT_SPEED_SCALE` | 275.0f | 射速到电机转速的换算系数 |
| `HOT_SPEED_LIMIT` | 2200.0f | 拨弹电机转速上限 |
| `SHOOT_TIME_MIN` | 250U | 最小连发时间（ms） |
| `SHOOT_TIME_MAX` | 4000U | 最大连发时间（ms） |

### 4. 依赖

- `referee.h` — 读取裁判系统热量数据（`referee_data.power_heat.shooter_17mm_1_barrel_heat`、`heat_limit`、`cooling_value`）
- `shoot.h` — 访问全局控制结构体 `shoot_cmd_send`

## 注意事项

- 模块依赖裁判系统数据，若裁判系统离线则热量数据不更新，行为可能不符合预期
- 切换 42mm 弹丸时需同步修改 `HOT_HEAT_PER_SHOT`（42mm 单发热量为 100）
- 该模块仅控制拨弹电机转速，摩擦轮速度仍需在 `FIRE_MODE` 分支中单独设置
