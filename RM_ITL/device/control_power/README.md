# 超级电容功率管理集成文档

## 概述

本目录实现了超级电容功率控制板与现有电机级功率控制的集成，实现了系统级的功率管理和"削峰填谷"功能。

## 工作原理

```
裁判系统限制 (60W) 
    ↓
超级电容功率控制板
    ├── 电池输出：60W (不超裁判限制)
    ├── 电容补充：20W (放电时)
    └── 总输出：80W到底盘
    ↓
电机级功率控制 (现有代码)
    ├── 在80W范围内分配功率给各电机
    └── 实现精细化功率管理
    ↓
底盘电机 (3508/6020)
```

## 文件说明

### 核心文件
- `supercapacitor_power.h` - 超级电容管理头文件
- `supercapacitor_power.c` - 超级电容管理实现
- `control_power.h` - 已更新，包含超级电容相关函数声明

### 示例文件
- `supercapacitor_integration_example.c` - 集成示例代码

### 原有文件
- `control_power.c` - 电机级功率控制（保持不变）
- `control_power.h` - 电机级功率控制头文件（已更新）

## 快速开始

### 1. 添加文件到项目

将以下文件添加到你的项目编译列表中：
- `supercapacitor_power.c`

### 2. 修改底盘任务

在你的 `chassis_task` 中添加超级电容集成：

```c
void chassis_task(void *argument) {
    // 初始化底盘
    Chassis_Init(Chassis_3508, Chassis_6020);
    
    // 初始化超级电容管理器
    supercapacitor_manager_init();
    
    // 设置CAN回调（在bsp_can初始化后）
    bsp_can1_set_callback(supercapacitor_can_rx_callback);
    
    for(;;) {
        // ... 其他底盘控制代码 ...
        
        // ===== 超级电容功率管理集成 =====
        // 1. 获取裁判系统限制功率
        float referee_limit = get_referee_chassis_power_limit();
        
        // 2. 超级电容功率控制
        supercapacitor_power_control(referee_limit);
        
        // 3. 获取系统实际可用的功率限制（从功率控制板读取）
        // 这个值可能 > referee_limit（电容放电时）
        float system_power_limit = get_system_power_limit();
        
        // 4. 如果需要主动降功率（电容状态异常）
        if (get_active_reduce_power()) {
            system_power_limit *= get_power_reduce_factor();
        }
        
        // 5. 调用电机级功率控制
        // 关键：传入system_power_limit，不是referee_limit
        Chassis_Power_Total_Control(system_power_limit, 4, 4, chassis_3508);
        
        // 6. 应用衰减系数到各电机
        for (int i = 0; i < 4; i++) {
            float decay_number = Chassis_3508_Get_Limited_Number(i);
            Chassis_3508[i].motor_data->target_velocity *= decay_number;
            
            if (get_active_reduce_power()) {
                Chassis_3508[i].motor_data->target_velocity *= get_power_reduce_factor();
            }
        }
        
        // ... 其他代码 ...
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
```

### 3. 实现裁判系统接口

你需要实现以下两个函数（根据你的裁判系统协议）：

```c
/**
 * @brief 从裁判系统获取底盘功率限制
 * @return 裁判系统设定的底盘功率限制（单位：W）
 */
float get_referee_chassis_power_limit(void) {
    // TODO: 从裁判系统数据中解析底盘功率限制
    // 例如：从裁判系统协议中读取对应的字段
    return 60.0f;  // 示例值
}

/**
 * @brief 从裁判系统获取缓冲能量
 * @return 当前缓冲能量值（单位：J）
 * @note 仅在使用缓冲能量环PID模式时需要
 */
float get_referee_buffer_energy(void) {
    // TODO: 从裁判系统数据中解析缓冲能量
    return 30.0f;  // 示例值
}
```

## 工作模式

### 模式1：简单模式（推荐）

功率控制板自动管理电容充放电，无需手动计算。

```c
// 在初始化时设置
supercapacitor_set_buffer_energy_mode(false);
```

**特点**：
- 简单易用，无需实现裁判系统缓冲能量接口
- 功率控制板自动管理
- 适合大多数应用场景

### 模式2：缓冲能量环PID模式（高级）

主动管理电容能量，实现最优的能量利用。

```c
// 在初始化时设置
supercapacitor_set_buffer_energy_mode(true);
supercapacitor_set_target_buffer_energy(30.0f);  // 目标缓冲能量30J
```

**特点**：
- 需要实现 `get_referee_buffer_energy()` 函数
- 可以精确控制电容能量
- 最大化利用电容存储能力

## 主要功能

### 1. 削峰填谷

当底盘需求功率小于裁判限制时，多余功率存储到电容；当需求功率大于裁判限制时，电容和电池同时供电。

### 2. 主动降功率保护

当检测到以下情况时，自动降低底盘功率：
- 电容电流 > 8A
- 电容电压 < 10V

保护机制采用平滑衰减，避免底盘抖动。

### 3. 实时状态监控

提供以下状态查询接口：
- `get_capacitor_voltage()` - 电容电压
- `get_capacitor_current()` - 电容电流
- `get_capacitor_level()` - 电容电量百分比
- `get_battery_power()` - 电池功率
- `get_system_power_limit()` - 系统实际可用功率限制

## 参数配置

### 宏定义（在 `supercapacitor_power.h` 中）

```c
#define POWER_CONTROLLER_ID          0x0FF    // 功率控制板CAN ID
#define MIN_LIMIT_POWER              40.0f    // 最小限制功率 (W)
#define MAX_LIMIT_POWER              250.0f   // 最大限制功率 (W)
#define CAPACITOR_CHARGE_VOLTAGE     18.0f   // 电容充电完成电压 (V)
#define CAPACITOR_WARN_CURRENT       8.0f     // 电容警告电流 (A)
#define CAPACITOR_WARN_VOLTAGE       10.0f    // 电容警告电压 (V)
#define TARGET_BUFFER_ENERGY         30.0f    // 目标缓冲能量 (J)
#define MAX_BUFFER_ENERGY            60.0f    // 缓冲能量上限 (J)
```

### PID参数（在 `supercapacitor_power.c` 中）

```c
#define BUFFER_ENERGY_PID_KP   0.5f   // 比例系数
#define BUFFER_ENERGY_PID_KI   0.01f  // 积分系数
#define BUFFER_ENERGY_PID_KD   0.0f   // 微分系数
#define BUFFER_ENERGY_PID_MAX  20.0f   // 最大补充功率 (W)
```

**注意**：PID参数需要根据实际情况调试。

## 硬件连接

### CAN连接
- 功率控制板CAN口 → 主控CAN1（或根据实际使用的CAN口）
- 确保CAN终端电阻配置正确（功率控制板已有120Ω电阻）

### 上电顺序
1. 先给功率控制板上电
2. 等待电容充至18V（可以读取 `get_capacitor_voltage()`）
3. 再启动底盘

### 禁止操作
- 给功率控制板上电后，禁止带电插拔任何线
- 超级电容接上功率控制板前放电至2V以下
- UART1、SWD的输出电源仅供给调试器件

## CAN通信协议

### 接收协议（功率控制板 → 主控）

**CAN ID**: 0x0FF
**频率**: 50Hz

| 字节 | 内容 | 说明 |
|------|------|------|
| 0-1 | 电容电压 | 高8位 + 低8位，0~30000对应0~30V |
| 2-3 | 电容电流 | 高8位 + 低8位，-12000~12000对应-12~12A（负为充电，正为放电）|
| 4 | 电池功率 | 0~256W |
| 5 | 当前限制功率 | 0~256W |
| 6 | 电容电量百分比 | 0~100 |
| 7 | 状态指示 | bit0: PID运行标识，bit1: 输入欠压，bit2: 电容状态，bit3: 过流警告 |

### 发送协议（主控 → 功率控制板）

**CAN ID**: 0x0FF
**频率**: 不高于50Hz

| 字节 | 内容 | 说明 |
|------|------|------|
| 0 | 限制功率整数位 | 30~250对应30~250W |
| 1 | 限制功率小数位 | 0~99对应0~0.99W |
| 2-7 | 预留 | 0 |

## 调试建议

### 1. 使用调试监控函数

```c
// 在调试任务中调用
supercapacitor_debug_monitor();
```

输出示例：
```
========== 超级电容状态 ==========
电容电压: 18.50 V
电容电流: -2.30 A  (充电)
电容电量: 85.0 %
电池功率: 45.0 W
系统功率限制: 65.0 W  (电池45W + 电容放电20W)
主动降功率: 否
衰减系数: 1.00
==================================
```

### 2. 调试步骤

1. **硬件连接测试**
   - 检查CAN通信是否正常
   - 验证能否接收到功率控制板数据
   - 查看电容电压是否正确更新

2. **简单模式测试**
   - 使用简单模式
   - 观察系统功率限制是否动态变化
   - 验证底盘是否能获得超过裁判限制的功率

3. **高级模式测试**（可选）
   - 启用缓冲能量环PID
   - 调整PID参数
   - 观察能量管理效果

4. **保护功能测试**
   - 模拟电容过流/欠压情况
   - 验证主动降功率是否正常工作
   - 检查底盘是否平滑减速

### 3. 常见问题

**Q: 为什么系统功率限制还是裁判限制值？**
A: 可能原因：
- 电容电量不足，无法放电
- CAN通信异常
- 功率控制板未正常工作

**Q: 底盘抖动严重？**
A: 检查：
- 功率衰减系数变化是否过快
- PID参数是否合适
- 电容状态变化是否频繁

**Q: 电机被限制到很低的功率？**
A: 检查：
- 是否激活了主动降功率
- 电容电压/电流是否异常
- 裁判系统限制是否正确

## 技术支持

如有问题，请检查：
1. PDF使用说明书（`超级电容功率控制板V2.1使用说明.pdf`）
2. 示例集成代码（`supercapacitor_integration_example.c`）
3. 调试监控输出

## 更新日志

### V1.0 (2024-03-06)
- 初始版本
- 实现超级电容功率管理
- 集成现有电机级功率控制
- 提供简单模式和高级模式
- 实现主动降功率保护