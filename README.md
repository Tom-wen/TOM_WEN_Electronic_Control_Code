<div align="center">

# TOM_WEN_Electronic_Control_Code

**RoboMaster 机器人电控代码合集**

多年积累的实车主控代码 · 电机测试工程 · 雷达 / 哨兵专项工程 · 一套可复用的分层电控框架

`STM32H723` &nbsp;·&nbsp; `STM32H7 / F4 / F1` &nbsp;·&nbsp; `FreeRTOS` &nbsp;·&nbsp; `CMake` &nbsp;·&nbsp; `arm-none-eabi-gcc` &nbsp;·&nbsp; `C`

</div>

---

## 为什么看这个仓库

| | |
| :--- | :--- |
| **一套成型的五层架构** | `Core / device / bsp / modules / app`，层间靠回调函数指针解耦，低层不反向依赖高层 |
| **面向对象的 C** | 电机抽象成 `MotorInstance`，上层只调 `motor_control` 一个接口，换型号只改注册处一行 |
| **多机器人共用一套码** | 舵轮 / 麦轮 / 摆臂 / 哨兵 / 无人机 / 靶车，靠 `config.h` 宏开关裁剪编译 |
| **算法库齐全** | PID、ADP、MPC、SMC、LQR+ESO、四元数 EKF、卡尔曼、RLS 功率辨识，多数附数学推导文档 |
| **工程化配套** | CMake Presets、J-Link 烧录脚本、VS Code 调试配置自动同步脚本 |

---

## 目录

- [仓库结构](#仓库结构)
- [通用框架 RM_ITL_Basic_Framework](#通用框架-rm_itl_basic_framework)
  - [技术栈](#技术栈) · [分层架构](#分层架构) · [目录说明](#目录说明)
  - [编译裁剪 config.h](#编译裁剪-configh) · [编译与烧录](#编译与烧录)
- [历史工程（Keil MDK）](#历史工程keil-mdk)
- [开发环境](#开发环境)
- [代码阅读建议](#代码阅读建议)

---

## 仓库结构

| 目录 | 说明 | 主控 | 构建 |
| :--- | :--- | :---: | :---: |
| **`RM_ITL_Basic_Framework`** | **通用电控框架（推荐）** · 工程机器人（摆臂底盘）实车代码 | `STM32H723VGT6` | CMake |
| `RM_ITL` | 框架早期版本 · 步兵实车代码 · 附框架设计说明 | `STM32H723VGT6` | CMake |
| `RM_ITL_Old_Infantry` | 老步兵框架 · DT7 与 i6X 遥控两个版本（含升降机构） | `STM32H7` | Keil |
| `RM_ITL_Infantry_chaodianban` | 步兵 / 英雄超电板相关代码 | `STM32F4` | Keil |
| `ITL_Sentry_auto` | 全自动哨兵 | `STM32F4` | Keil |
| `nailong` | 奶龙机器人 · DT7、i6X 两个遥控版本 | `STM32H7` | Keil |
| `radar` / `new_radar` | 雷达站工程（三脚架控制、上位机串口通信） | `STM32F4` | Keil |
| `MotorM15_test` | 瓴控 M15 电机测试工程 | `STM32F4` | Keil |
| `GQ_Motor_test` | GQ 电机测试工程 | `STM32F4` | Keil |
| `huawu+OK` | 小车工程（循迹、OLED、JY61P 姿态、舵机、OpenMV） | `STM32F103ZE` | Keil |

> **`RM_ITL` 与 `RM_ITL_Basic_Framework` 是同一框架的两个阶段**
> 前者是框架成型期的步兵代码，后者是整理后的通用框架并补齐了 `config.h` 编译开关。
> **新项目建议以 `RM_ITL_Basic_Framework` 为起点。**

---

## 通用框架 `RM_ITL_Basic_Framework`

### 技术栈

| 类别 | 内容 |
| :--- | :--- |
| **MCU** | STM32H723VGT6（Cortex-M7） |
| **RTOS** | FreeRTOS · CMSIS-RTOS v2 接口，任务在 `Core/Src/freertos.c` 创建 |
| **HAL** | STM32CubeMX 生成的 HAL 驱动 + STM32 USB Device Library（CDC 虚拟串口） |
| **构建** | CMake ≥ 3.22 + Ninja + `arm-none-eabi-gcc`（见 `cmake/gcc-arm-none-eabi.cmake`） |
| **烧录** | J-Link（`flash.jlink`）或 VS Code + Cortex-Debug |

### 分层架构

```text
                    ┌──────────────────────────────────────┐
                    │  app/          业务任务层            │
                    │  chassis · gimbal · shoot · arm      │
                    │  ins · referee · ui · lerobot        │
                    └────────┬──────────────────┬──────────┘
                             │                  │
              ┌──────────────┴───────┐   ┌──────┴───────────────┐
              │  modules/  器件驱动层│   │  device/  算法组件层 │
              │  Motor · BMI088      │   │  Pid · Adp · Mpc     │
              │  DM_IMU · remote     │   │  Smc · Lqr · EKF     │
              └──────────┬───────────┘   └──────────────────────┘
                         │
              ┌──────────┴───────────┐
              │  bsp/     板级驱动层 │
              │  can · spi · usart   │
              │  usb · pwm · dwt     │
              └──────────┬───────────┘
                         │
              ┌──────────┴───────────┐
              │  Core/  CubeMX + HAL │
              └──────────────────────┘
```

**两个核心设计**

<table>
<tr><td width="50%" valign="top">

**① 回调函数指针 —— 层间解耦**

bsp 层在中断里收数据，但数据该由 modules 层处理，底层又不能反向 include 高层。于是在 `bsp_can.h` 定义回调类型：

```c
typedef void (*can_rx_callback_t)(CANRxData *rx_data);
```

modules 层在初始化时把自己注册进去：

```c
bsp_can1_set_callback(dm_motor_can_callback);
```

中断里只管调用：

```c
if (can1_user_callback != NULL)
    can1_user_callback(&Rx_data1[index]);
```

</td><td width="50%" valign="top">

**② 电机多态 —— 接口统一**

`MotorInstance` 内藏控制函数指针，上层任务看到的只有一个 `motor_control`：

```c
motors1->motor_control(&hfdcan2,
                       motors1->motor_data);
```

想换控制模式，不必去改任务层里每一处调用，只改注册处一个参数：

```c
motors[0] = CreateMotor(Motor3508, 1,
    &motors_data[0],
    DJI3508_Spd_mode, ...);
```

</td></tr>
</table>

> 完整代码示例见 `RM_ITL_Basic_Framework/README.md` 与 `RM_ITL/README.md`。

### 目录说明

<details open>
<summary><b><code>app/</code> — 业务任务层</b></summary>

<br>

| 目录 | 内容 |
| :--- | :--- |
| `Init/` | 按编译宏注册各类电机实例（底盘、云台、发射、机械臂、抬升） |
| `chassis/` | 舵轮底盘、麦轮底盘、英雄底盘、麦轮英雄底盘、摆臂底盘、AGV、哨兵底盘、靶车底盘 |
| `gimbal/` | 步兵云台、英雄云台、摆臂云台、哨兵云台、无人机云台 |
| `shoot/` | 步兵 / 英雄 / 摆臂 / 哨兵 / 无人机发射机构 |
| `arm/` · `arm_lift/` | 机械臂与机械臂抬升 |
| `ins/` | 姿态解算任务（IMU + 四元数 EKF） |
| `detect/` | 掉线 / 异常检测与蜂鸣器报警 |
| `data_processing/` | 数据集中处理与转发 |
| `referee/` | 裁判系统数据解析（含哨兵专用版本） |
| `ui/` | 裁判系统 UI 绘制（默认 UI、静态 UI、模式 UI、图形绘制接口） |
| `lerobot/` | 机械臂关节采集 / 遥操作，配合上位机做数据采集 |
| `test/` | 调试测试任务 |

</details>

<details>
<summary><b><code>modules/</code> — 器件驱动层</b></summary>

<br>

| 模块 | 内容 |
| :--- | :--- |
| `Motor/` | `DJI_Motor`（3508 / 6020 / 2006 / 4310）、`DM_Motor`（达妙）、`FT_Motor`、`GQ_Motor`、`RS_Motor`，统一由 `motor_types.h` 的 `MotorInstance` / `MotorControlData` / `MotorType` 抽象 |
| `BMI088/` | SPI 陀螺仪 + 加速度计 |
| `DM_IMU/` | 达妙 IMU |
| `remote_control/` | DT7（DBUS）、FS（富斯）、i6X、VT03 图传遥控 |
| `PID_Test/` | PID 调试相关 |

</details>

<details>
<summary><b><code>device/</code> — 通用算法与功能组件</b></summary>

<br>

| 模块 | 说明 |
| :--- | :--- |
| `Pid/` | 单环与串级 PID（含积分限幅、输出限幅） |
| `Adp/` | 自适应控制（含数学推导文档） |
| `Mpc/` | 模型预测控制（含使用说明与推导文档） |
| `Smc/` | 滑模控制 |
| `Lqr/` | LQR 基础实现、哨兵 YAW 自瞄 LQR+ESO 控制器（附 `yaw_auto_lqr_tune.m` 调参脚本） |
| `Algorithm/` | 四元数 EKF、通用卡尔曼滤波、控制器框架、`user_lib` 数学库 |
| `Lowpass/` | 低通滤波器 |
| `CRC/` · `Circular_buffer/` · `Signal/` | CRC 校验、环形缓冲、信号处理 |
| `Proto_link/` | 上位机通信协议 |
| `control_power/` | 功率控制与 RLS 参数辨识 |
| `Hot_control/` | 枪口热量控制 |
| `Torque_control/` | 力矩控制 |

</details>

<details>
<summary><b><code>bsp/</code> — 板级驱动层 &nbsp;·&nbsp; <code>Core/</code> — CubeMX 生成</b></summary>

<br>

| 模块 | 说明 |
| :--- | :--- |
| `can` | FDCAN，回调注册机制（附 `bsp_can.md`） |
| `spi` / `usart` / `usb` | SPI、USART（DMA 收发）、USB CDC 虚拟串口 |
| `pwm` / `dwt` / `led` / `buzzer` | PWM 输出、微秒级计时、SPI 驱动 WS2812（9 灯）、蜂鸣器 |

`Core/` 包含 `fdcan.c`、`spi.c`、`tim.c`、`usart.c`、`dma.c`、`freertos.c` 等外设初始化与中断。

</details>

### 编译裁剪 `config.h`

`config.h` 与 `CMakeLists.txt` 同级，通过宏开关选择编译哪一套机器人代码：

```c
// 板子 / 底盘 / 云台 / 发射 / 机械臂 / 功能 / 遥控器 / PID 类型
#define ROBOTIC_SWING_CHASSIS     // 摆臂底盘
#define ROBOTIC_SWING_GIMBAL      // 摆臂云台
#define ROBOTIC_SWING_SHOOT       // 摆臂发射
#define DJI_REMOTE                // 大疆遥控器
#define PRE_PID                   // 使用本仓库 PID
```

<details>
<summary><b>展开全部可用开关</b></summary>

<br>

| 分类 | 开关 |
| :--- | :--- |
| **板子** | `chassis_board`（底盘板）、`gimbal_board`（云台板） |
| **底盘** | `COMPILE_AGV_CHASSIS`（舵轮）、`COMPILE_MECANUM_CHASSIS`（麦轮）、`COMPILE_HERO_CHASSIS`（英雄）、`COMPILE_M_HERO_CHASSIS`（麦轮英雄）、`ROBOTIC_SWING_CHASSIS` + `ROBOTIC_SWING_HOISTING`（摆臂）、`TARGET_CHASSIS`（靶车） |
| **云台** | `COMPILE_GIMBAL`（步兵）、`ROBOTIC_SWING_GIMBAL`（摆臂）、`COMPILE_UAV_GIMBAL`（无人机）、`COMPILE_HERO_GIMBAL`（英雄）、`AUTO_SENTRY`（哨兵） |
| **发射** | `COMPILE_SHOOT`（步兵）、`ROBOTIC_SWING_SHOOT`（摆臂）、`COMPILE_UAV_SHOOT`（无人机）、`COMPILE_HERO_SHOOT`（英雄） |
| **机构与功能** | `COMPILE_ARM` / `COMPILE_ARM_LIFT`（机械臂）、`COMPILE_REFEREE`（裁判系统）、`COMPILE_UI`、`COMPILE_POWER` / `COMPILE_M_POWER`（功率控制）、`HOT_CONTROL`（热量控制）、`COMPILE_TEST` |
| **遥控器** | `DJI_REMOTE` / `FS_REMOTE` / `VT_03_REMOTE` |
| **PID** | `PRE_PID` / `ALG_PID` |

</details>

### 编译与烧录

```bash
# 1. 配置（可选 Debug / Release / RelWithDebInfo / MinSizeRel）
cmake --preset Debug

# 2. 编译
cmake --build build/Debug

# 3. 烧录
JLink -CommanderScript flash.jlink
```

> **工程默认把各构建类型的优化等级统一为 `-O0 -g3`**，方便打断点在线调试。

**配套脚本与文件**

| 文件 | 作用 |
| :--- | :--- |
| `update_config.sh` | 读取 `.ioc` 与 `CMakeLists.txt` 的芯片信息，自动同步 `.vscode/` 下 `c_cpp_properties.json`、`launch.json`、`tasks.json` 及 `flash.jlink` 的芯片型号与 ELF 路径。<br>切换芯片 / 工程名后执行一次：`chmod +x update_config.sh && ./update_config.sh` |
| `RM_ITL_Engineering_robot.ioc` | CubeMX 工程文件，改完外设配置后需在 CubeMX 中重新生成代码 |
| `STM32H723XG_FLASH.ld` | 链接脚本 |
| `STM32H723.svd` | 供调试器查看寄存器 |
| `CMakePresets.json` | 四种构建预设 |

---

## 历史工程（Keil MDK）

以下工程使用 **STM32CubeMX + Keil MDK-ARM**，目录组织为：

```text
Core/           CubeMX 生成的外设初始化与中断
Drivers/        HAL 与 CMSIS
Middlewares/    FreeRTOS 等第三方库
application/    电机收发 · 底盘行为 · 云台行为 · 遥控 · 结构体定义
Task/           chassis_task · gimbal_task · shoot_task · INS_task
                detect_task · referee_usart_task · hoisting_task
User/           主函数与用户代码
boards/         ┐ 沿用经典 RM 电控框架
components/     ┘ 的分层组织
```

> **注意：推送前务必执行 `keilkilll.bat`**
> 否则会把 `Objects/`、`Listings/` 等编译中间产物一并提交。
> 部分工程目录内保留了 `.zip` 压缩包（如 `GQ_Motor_test.zip`、`RM_ITL_Hoisting.zip`），为历史归档。

---

## 开发环境

| 项目 | 要求 |
| :--- | :--- |
| **工具链** | `arm-none-eabi-gcc`（CMake 工程）· Keil MDK-ARM（历史工程） |
| **构建工具** | CMake ≥ 3.22 · Ninja |
| **烧录调试** | J-Link · ST-Link |
| **推荐编辑器** | VS Code + Cortex-Debug + clangd（工程已开启 `CMAKE_EXPORT_COMPILE_COMMANDS`） |

---

## 代码阅读建议

1. 先读 `RM_ITL_Basic_Framework/README.md`，理解 **函数指针解耦** 与 **电机多态** 两个核心设计。
2. 再看 `app/Init/Init.c`：所有电机实例按编译宏在此注册，是理解整体结构的入口。
3. 顺着一条控制链读下去：

   ```text
   Core/Src/freertos.c          任务创建
        ↓
   app/chassis/chassis_task()   业务逻辑
        ↓
   modules/Motor/*              驱动收发
        ↓
   bsp/can/bsp_can.c            中断与回调
   ```

4. 需要算法时到 `device/` 下找对应模块，多数附有 `.md` 推导或使用文档。

<div align="center">

---

**如果这个仓库对你有帮助，欢迎点个 Star**

</div>
