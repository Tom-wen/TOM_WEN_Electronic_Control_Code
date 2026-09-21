# MPC 模型预测控制 使用说明

## 变体选择

在 `mpc.h` 中通过条件编译选择：

```c
#define PRE_MPC    // 基础MPC: 一阶误差动力学 + 积分增广 + 滚动QP
//#define ALG_MPC  // 拓展MPC: 二阶离散状态空间 + 滚动QP + 死区/积分分离
```

## 核心原理

与 AD/PID 的本质区别：**MPC 每步求解一个有限时域最优控制问题（滚动优化），而非固定增益反馈。**

```
在线每步:
  x = 当前状态                     // [e, ∫e] 或 [e, de]
  f = E * x                       // 构造 QP 线性项
  解 L·Lᵀ·U* = -f                 // Cholesky 三角回代求解 QP min 0.5U'HU + x'EU
  u = U*(0)                       // 取序列第一拍输出（滚动时域）
```

H, E, L 矩阵在 Init() 离线预计算，在线仅做 O(N²) 三角回代（N=N_p，通常 5~20）。

---

## PRE_MPC 参数表

### 函数原型

```c
void MPC_Init(MPC *mpc,
    uint8_t N_p,            // 预测时域
    float Q, float Q_i,     // 状态权重: 误差/积分
    float R,                // 控制权重
    float A, float B,       // 一阶模型: e(k+1) = A·e(k) + B·u(k)
    float maxI,             // 积分限幅
    float maxOut,           // 输出限幅
    float Kff,              // 前馈系数
    float dt,               // 采样周期 (s)
    float dead_zone);       // 误差死区
```

| 参数 | 含义 | 建议范围 | 调参方向 |
|------|------|---------|---------|
| `N_p` | 预测时域长度 | 5 ~ 20 | 越大越"远视"，响应更平滑但计算稍增；一般 8~10 即可 |
| `Q` | 瞬时误差代价权重 | 1 ~ 1000 | 增大→响应更快，但可能超调振荡 |
| `Q_i` | 积分误差代价权重 | 0.1 ~ 100 | 增大→消除静差更快，过大会积分饱和 |
| `R` | 控制量代价权重 | 0.01 ~ 100 | 增大→控制更"保守"，输出更平缓；减小→响应更激进 |
| `A` | 误差衰减系数 | 0.5 ~ 0.99 | 反映系统自然阻尼：A 越小衰减越快；离散化模型参数 |
| `B` | 控制输入增益 | 0.001 ~ 10 | 表示单位控制量产生的误差变化，需根据实际系统辨识 |
| `maxI` | 积分限幅 | 根据系统量级 | 防积分饱和，取预期最大输出的 30%~60% |
| `maxOut` | 输出限幅 | 物理限幅值 | 如 DJI 电机 CAN 电流 ±16384 |
| `Kff` | 前馈系数 | 0.0 ~ 1.0 | 参考值变化量前馈，增强跟踪 |
| `dt` | 采样周期 | 0.001 ~ 0.01 | 必须与实际控制周期一致 (s) |
| `dead_zone` | 误差死区 | 0.0 ~ 1.0 | 小误差时不输出，避免微振 |

### 增广系统模型

MPC 内部自动将系统增广为 2 阶状态 `[e, ∫e]^T`:

```
[e(k+1)]   [A   0] [e(k)]   [B]
[i(k+1)] = [dt  1] [i(k)] + [0] · u(k)
```

其中 dt 为采样周期，确保积分状态与物理时间一致。

---

## ALG_MPC 参数表

### 函数原型

```c
void MPC_Init(MPC *mpc,
    uint8_t N_p, uint8_t N_c,              // 预测/控制时域
    float A11, float A12, float A21, float A22,  // 2×2 状态矩阵
    float B1, float B2,                           // 2×1 输入矩阵
    float Q1, float Q2, float R, float K_F,       // 权重 + 前馈
    float I_Out_Max, float Out_Max,               // 积分/输出限幅
    float D_T,                                     // 采样周期
    float Dead_Zone);                              // 死区
```

| 参数 | 含义 | 建议范围 | 调参方向 |
|------|------|---------|---------|
| `N_p` | 预测时域 | 5 ~ 20 | 同 PRE_MPC |
| `N_c` | 控制时域 | = N_p | 当前简化为 N_c=N_p |
| `A11~A22` | 二阶离散状态矩阵 | 需辨识 | `[e(k+1); de(k+1)] = A·[e(k); de(k)] + B·u(k)` |
| `B1, B2` | 输入矩阵 | 需辨识 | 同上 |
| `Q1` | 位置/误差权重 | 1 ~ 1000 | 增大→位置跟踪更紧 |
| `Q2` | 速度/误差变化权重 | 0.1 ~ 100 | 增大→抑制超调，但响应变慢 |
| `R` | 控制代价 | 0.01 ~ 100 | 同 PRE_MPC |
| `K_F` | 前馈系数 | 0.0 ~ 1.0 | 同 PRE_MPC |
| `I_Out_Max` | 积分项限幅 | 见下方 | 0=不限幅；建议设为预期静态补偿量的 1~2 倍 |
| `Out_Max` | 总输出限幅 | 物理限值 | 0=不限幅 |
| `D_T` | 采样周期 | 与实物一致 | 单位 s，如 0.001 |
| `Dead_Zone` | 死区 | 0.0 ~ 1.0 | 误差在该范围内时置零 |

ALG_MPC 在当前实现中同样使用 N_c = N_p（简化控制时域）。

---

## 使用示例

### PRE_MPC 单环

```c
#include "mpc.h"

MPC mpc[3];  // [single, cascade_inner, cascade_outer]

// 初始化: N_p, Q, Q_i, R, A, B, maxI, maxOut, Kff, dt, dead_zone
MPC_Init(&mpc[mpc_single_loop],
    10,         // 预测时域 N_p = 10
    500.0f,     // Q (误差权重) —— 高增益快响应
    50.0f,      // Q_i (积分权重)
    10.0f,      // R (控制权重)
    0.95f,      // A (误差衰减)
    0.05f,      // B (控制增益)
    5000.0f,    // maxIntegral
    16384.0f,   // maxOutput (DJI 电机满量程)
    0.5f,       // Kff
    0.001f,     // dt = 1ms
    0.1f);      // dead_zone

// 在 1ms 中断中调用
void ControlLoop(void)
{
    float ref = GetReference();
    float fdb = GetFeedback();
    MPC_Calc(&mpc[mpc_single_loop], ref, fdb);
    SetOutput(mpc[mpc_single_loop].Out);
}
```

### PRE_MPC 串级（位置环 + 速度环）

```c
MPC mpc[3];

// 外环 — 位置控制
MPC_Init(&mpc[mpc_cascade_outer],
    8, 100.0f, 20.0f, 5.0f, 0.98f, 0.02f,
    2000.0f, 10000.0f, 0.3f, 0.001f, 0.05f);

// 内环 — 速度控制
MPC_Init(&mpc[mpc_cascade_inner],
    5, 300.0f, 10.0f, 8.0f, 0.90f, 0.10f,
    3000.0f, 16384.0f, 0.0f, 0.001f, 0.5f);

void CascadeControl(void)
{
    float pos_ref = GetPosRef();
    float pos_fdb = GetPosFdb();
    float vel_fdb = GetVelFdb();
    MPC_CascadeCalc(mpc, pos_ref, pos_fdb, vel_fdb);
    SetOutput(mpc[mpc_cascade_inner].Out);
}
```

### ALG_MPC 二阶模型（需先辨识 A, B 矩阵）

```c
MPC mpc;

// 启用 #define ALG_MPC 后使用
MPC_Init(&mpc,
    6, 6,                           // N_p, N_c
    0.90f, 0.01f, -4.5f, 0.85f,    // A 矩阵
    0.02f, 1.5f,                    // B 矩阵
    200.0f, 5.0f,                   // Q1(位置), Q2(速度)
    10.0f, 0.5f,                    // R(控制), K_F(前馈)
    3000.0f, 16384.0f,              // 积分/输出限幅
    0.001f, 0.1f);                  // D_T, Dead_Zone

MPC_Calc(&mpc, ref, fdb);
```

---

## 调参指南

### 1. 确定模型参数 A, B

**PRE_MPC** 使用一阶模型，可通过阶跃响应辨识：
1. 给系统一个阶跃输入 u0
2. 记录误差衰减曲线 e(k)
3. 拟合 e(k+1) = A·e(k)，得到 A ≈ e(∞)/e(0) 的衰减比
4. B ≈ Δe(1) / u0（第一个采样周期的误差变化 ÷ 输入）

通常先设 A=0.95, B=0.05，再根据响应调试。

**ALG_MPC** 需要完整的 2×2 矩阵，建议用 MATLAB 子空间辨识 (`n4sid` / `ssest`)：
```matlab
data = iddata(y, u, Ts);    % Ts = 采样周期
sys = ssest(data, 2);       % 辨识 2 阶离散模型
A = sys.A; B = sys.B;
```

### 2. 权重 Q, R 调试顺序

1. 先设 R=1，只调 Q（误差权重）
2. Q 偏大 → 响应快但振荡；Q 偏小 → 响应迟钝
3. 调好 Q 后，增大 R → 输出更平滑；减小 R → 更激进
4. 最后调 Q_i / Q2（积分/速度权重）：  
   Q_i 大 → 快速消静差，但易积分饱和  
   Q2 大 → 抑制超调，但响应变慢

### 3. 预测时域 N_p 选择

| N_p | 效果 |
|-----|------|
| 1~3 | 近似 P 控制，响应最快但可能振荡 |
| 5~10 | 推荐范围，兼顾性能与计算 |
| 10~20 | 更平滑但运算稍增 |

N_p 不影响控制量解析形式（无约束时），但影响 Cholesky 回代耗时（O(N²)）。

### 4. 观察变量

运行时可通过结构体字段监控 MPC 内部状态：

| 字段 | PRE_MPC | ALG_MPC | 含义 |
|------|---------|---------|------|
| `K_e` / `K[0]` | ✓ | ✓ | 等效瞬时反馈增益（只读, 根据当前状态反算） |
| `K_i` / `K[1]` | ✓ | ✓ | 等效积分/微分反馈增益 |
| `Out` | ✓ | ✓ | 控制器输出 |
| `error` / `Pre_Error` | ✓ | ✓ | 当前/上次误差 |
| `integral` / `Integral_Error` | ✓ | ✓ | 积分累加值 |

非约束 MPC 的 K_e, K_i 理论上不变；受死区/饱和影响时有微小波动，属正常现象。

### 5. 调试检查项

- ✅ Cholesky 是否求解成功：查看 Init 日志（失败时 H 不定，回退到 Q1/Q2 作为增益）
- ✅ N_p ≤ MPC_MAX_NP(20)：超限自动跳过矩阵构建
- ✅ 积分限幅设置合理：防止 windup 导致输出持续饱和
- ✅ 死区不要过大：过大导致无法消除小误差
