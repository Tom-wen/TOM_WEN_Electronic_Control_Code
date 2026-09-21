# PRE_ADP 数学公式推导

> 对应源码: `device/Adp/adp.c` (第4-286行, `#ifdef PRE_ADP`)

---

## 1. 状态空间 (State Space)

### 1.1 原始状态 (3维)

$$
\mathbf{x}_k = \begin{bmatrix} x_{0,k} \\ x_{1,k} \\ x_{2,k} \end{bmatrix}
= \begin{bmatrix} e_k \\ e_{int,k} \\ \dot{e}_k \end{bmatrix}
$$

### 1.2 误差

$$
e_k = r_k - y_k
$$

其中 $r_k$ 为参考输入(目标值)，$y_k$ 为反馈值(传感器测量值)。

### 1.3 微分 (一阶低通滤波)

$$
\dot{e}_k = (1 - \lambda) \cdot \dot{e}_{k-1} + \lambda \cdot \frac{e_k - e_{k-1}}{\Delta t}, \quad \lambda \in (0, 1]
$$

### 1.4 积分 (带 Anti-Windup)

$$
e_{int,k} = \begin{cases}
e_{int,k-1} + e_k \cdot \Delta t, & |u_{k-1}| < u_{\max} \\
e_{int,k-1}, & \text{otherwise}
\end{cases}
$$

然后限幅:

$$
e_{int,k} = \operatorname{clamp}(e_{int,k},\ -I_{\max},\ I_{\max})
$$

### 1.5 状态归一化

$$
\mathbf{x}_{n,k} = \begin{bmatrix}
x_{n0,k} \\ x_{n1,k} \\ x_{n2,k}
\end{bmatrix}
= \begin{bmatrix}
\dfrac{x_{0,k}}{s_0} \\[6pt] \dfrac{x_{1,k}}{s_1} \\[6pt] \dfrac{x_{2,k}}{s_2}
\end{bmatrix}, \quad s_i > 0
$$

其中 $\{s_0, s_1, s_2\}$ 为可配置的归一化尺度，默认值 $\{1.0, 1.0, 1.0\}$ (不归一化)。

---

## 2. Critic 网络 (值函数逼近)

### 2.1 二次型基函数

$$
\phi(\mathbf{x}_n) = \begin{bmatrix}
\phi_0 \\ \phi_1 \\ \phi_2 \\ \phi_3 \\ \phi_4 \\ \phi_5
\end{bmatrix}
= \begin{bmatrix}
x_{n0}^2 \\
x_{n0} \, x_{n1} \\
x_{n0} \, x_{n2} \\
x_{n1}^2 \\
x_{n1} \, x_{n2} \\
x_{n2}^2
\end{bmatrix}
$$

### 2.2 Critic 权重

$$
W_c = \begin{bmatrix}
W_{c0} & W_{c1} & W_{c2} & W_{c3} & W_{c4} & W_{c5}
\end{bmatrix}^\top
$$

初始值:

$$
W_c^0 = \begin{bmatrix} 1 & 0 & 0 & 1 & 0 & 1 \end{bmatrix}^\top
$$

### 2.3 值函数

$$
J(\mathbf{x}_n) = W_c^\top \phi(\mathbf{x}_n)
= \sum_{i=0}^{5} W_{ci} \cdot \phi_i(\mathbf{x}_n)
$$

### 2.4 值函数矩阵形式

令 $P$ 为对称矩阵:

$$
P = \begin{bmatrix}
W_{c0} & \dfrac{1}{2}W_{c1} & \dfrac{1}{2}W_{c2} \\[8pt]
\dfrac{1}{2}W_{c1} & W_{c3} & \dfrac{1}{2}W_{c4} \\[8pt]
\dfrac{1}{2}W_{c2} & \dfrac{1}{2}W_{c4} & W_{c5}
\end{bmatrix}
$$

则有:

$$
J(\mathbf{x}_n) = \mathbf{x}_n^\top P \mathbf{x}_n
$$

初始时 $P = I$，即 $J(\mathbf{x}_n) = \|\mathbf{x}_n\|^2$。

### 2.5 值函数的梯度 (对 Actor 更新使用)

$$
\nabla_{\mathbf{x}_n} J = \frac{\partial J}{\partial \mathbf{x}_n} = 2P\mathbf{x}_n
$$

展开为:

$$
\frac{\partial J}{\partial x_{n0}} = 2W_{c0} x_{n0} + W_{c1} x_{n1} + W_{c2} x_{n2}
$$

$$
\frac{\partial J}{\partial x_{n1}} = W_{c1} x_{n0} + 2W_{c3} x_{n1} + W_{c4} x_{n2}
$$

$$
\frac{\partial J}{\partial x_{n2}} = W_{c2} x_{n0} + W_{c4} x_{n1} + 2W_{c5} x_{n2}
$$

---

## 3. Actor 网络 (控制策略)

### 3.1 Actor 权重

$$
W_a = \begin{bmatrix}
W_{a0} & W_{a1} & W_{a2}
\end{bmatrix}^\top
$$

初始值:

$$
W_a^0 = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}^\top
$$

### 3.2 线性控制策略

$$
\boxed{u_{\text{adp}}(\mathbf{x}_n) = W_a^\top \mathbf{x}_n = W_{a0} x_{n0} + W_{a1} x_{n1} + W_{a2} x_{n2}}
$$

初始时: $u_{\text{adp}} = x_{n0} = e / s_0$ (纯比例控制)。

---

## 4. 总控制输出

### 4.1 死区处理

$$
e_{\text{db}} = \begin{cases}
e_k, & |e_k| \geq \varepsilon_{\text{db}} \\
0, & \text{otherwise}
\end{cases}
\quad \varepsilon_{\text{db}} = 0.05
$$

### 4.2 基础 PD 反馈分量

$$
u_{\text{basic}} = Q_0 \cdot e_{\text{db}} + Q_2 \cdot \dot{e}_k
$$

### 4.3 前馈分量

$$
u_{\text{ff}} = K_{\text{ff}} \cdot (r_k - r_{k-1})
$$

### 4.4 总输出

$$
\boxed{u_k = \underbrace{Q_0 e_{\text{db}} + Q_2 \dot{e}_k}_{\displaystyle u_{\text{basic}}} \;+\; \underbrace{W_a^\top \mathbf{x}_{n,k}}_{\displaystyle u_{\text{adp}}} \;+\; u_{\text{ff}}}
$$

### 4.5 输出饱和

$$
u_k \gets \operatorname{clamp}(u_k,\ -u_{\max},\ u_{\max})
$$

---

## 5. 在线学习 (Online Learning)

学习仅在第2个及以后的控制周期执行，首帧跳过(避免非物理初始状态冲击)。

### 5.1 效用函数 (即时代价)

使用**上一时刻**的状态和控制量:

$$
\boxed{U_{k-1} = \mathbf{x}_{n,k-1}^\top Q \mathbf{x}_{n,k-1} + R \cdot u_{k-1}^2}
$$

展开:

$$
U_{k-1} = Q_0 \cdot x_{n0,k-1}^2 + Q_1 \cdot x_{n1,k-1}^2 + Q_2 \cdot x_{n2,k-1}^2 + R \cdot u_{k-1}^2
$$

其中:

- $Q = \operatorname{diag}(Q_0, Q_1, Q_2)$ 为状态代价对角矩阵
- $R$ 为控制代价标量
- $u_{k-1}$ 为上一拍的总输出(物理尺度)

### 5.2 TD 误差 (Temporal Difference Error)

贝尔曼残差:

$$
\boxed{\delta_k = U_{k-1} + \gamma \cdot J(\mathbf{x}_{n,k}) - J(\mathbf{x}_{n,k-1})}
$$

其中 $\gamma \in (0, 1)$ 为折扣因子。

学习目标: $\delta_k \to 0$，即满足贝尔曼方程:

$$
J_{k-1} = U_{k-1} + \gamma J_k
$$

TD 误差限幅:

$$
\delta_k \gets \operatorname{clamp}(\delta_k,\ -\delta_{\max},\ \delta_{\max}), \quad \delta_{\max} = 10.0
$$

---

## 6. Critic 权重更新 — NLMS

### 6.1 梯度推导

Critic 更新目标: 最小化 $\frac{1}{2}\delta_k^2$

$$
\frac{\partial}{\partial W_c}\left(\frac{1}{2}\delta_k^2\right)
= \delta_k \cdot \frac{\partial \delta_k}{\partial W_c}
$$

$$
\frac{\partial \delta_k}{\partial W_c}
= \frac{\partial}{\partial W_c}\left( U_{k-1} + \gamma J_k - J_{k-1} \right)
= -\frac{\partial J_{k-1}}{\partial W_c} = -\phi(\mathbf{x}_{n,k-1})
$$

所以:

$$
\frac{\partial}{\partial W_c}\left(\frac{1}{2}\delta_k^2\right) = -\delta_k \cdot \phi_{k-1}
$$

梯度下降(带负号):

$$
\Delta W_c = +\alpha_c \cdot \delta_k \cdot \phi_{k-1}
$$

### 6.2 NLMS 归一化

为解耦学习率与基函数信号幅度，对基函数模平方归一化:

$$
\boxed{W_c \;\leftarrow\; \operatorname{clamp}\left(W_c + \frac{\alpha_c \cdot \delta_k}{\|\phi_{k-1}\|^2 + \varepsilon} \cdot \phi(\mathbf{x}_{n,k-1}),\ \pm W_{c,\max}\right)}
$$

其中:

$$
\|\phi_{k-1}\|^2 = \sum_{i=0}^{5} \phi_i^2(\mathbf{x}_{n,k-1}), \quad \varepsilon = 10^{-3}
$$

逐分量形式:

$$
W_{ci} \;\leftarrow\; \operatorname{clamp}\left(W_{ci} + \frac{\alpha_c \cdot \delta_k}{\sum_{j=0}^{5} \phi_j^2 + 10^{-3}} \cdot \phi_i,\ \pm W_{c,\max}\right)
$$

---

## 7. Actor 权重更新 — 策略梯度

### 7.1 仅当输出未饱和时更新

$$
\text{if } |u_k| < u_{\max} - 10^{-3} \quad \text{则执行 Actor 更新}
$$

### 7.2 梯度推导

优化目标: 最小化 Bellman 一步代价

$$
\mathcal{L} = R u_k^2 + \gamma J(\mathbf{x}_{n,k})
$$

对 $W_{ai}$ 求偏导:

$$
\frac{\partial \mathcal{L}}{\partial W_{ai}} = \frac{\partial}{\partial W_{ai}}\left(R u_k^2\right) + \gamma \cdot \frac{\partial J}{\partial W_{ai}}
$$

对于第一项:

$$
\frac{\partial (R u_k^2)}{\partial W_{ai}} = 2R u_k \cdot \frac{\partial u_k}{\partial W_{ai}}
$$

由于 $u_{\text{adp}} = W_a^\top \mathbf{x}_n$ 是 $u_k$ 随 $W_{ai}$ 变化的唯一通道:

$$
\frac{\partial u_k}{\partial W_{ai}} = x_{ni}
$$

因此:

$$
\frac{\partial (R u_k^2)}{\partial W_{ai}} = 2R u_k \cdot x_{ni}
$$

对于第二项(链式法则):

$$
\frac{\partial J}{\partial W_{ai}} = \frac{\partial J}{\partial x_{ni}} \cdot \frac{\partial x_{ni}}{\partial u_k} \cdot \frac{\partial u_k}{\partial W_{ai}}
$$

采用近似 $\frac{\partial x_{ni}}{\partial u_k} \approx -1$ (控制增大 → 误差减小):

$$
\frac{\partial J}{\partial W_{ai}} \approx -\frac{\partial J}{\partial x_{ni}} \cdot x_{ni}
$$

联立:

$$
\boxed{g_i = \frac{\partial \mathcal{L}}{\partial W_{ai}} = \left(2R \cdot u_k - \gamma \cdot \frac{\partial J}{\partial x_{ni}}\right) \cdot x_{ni}}
$$

### 7.3 权重更新

梯度限幅:

$$
g_i \gets \operatorname{clamp}(g_i,\ -10,\ 10)
$$

梯度下降:

$$
\boxed{W_{ai} \;\leftarrow\; \operatorname{clamp}\left(W_{ai} - \alpha_a \cdot g_i,\ \pm W_{a,\max}\right)}
$$

展开为:

$$
W_{ai} \;\leftarrow\; \operatorname{clamp}\left(W_{ai} - \alpha_a \cdot \left(2R u_k - \gamma \cdot \frac{\partial J}{\partial x_{ni}}\right) \cdot x_{ni},\ \pm W_{a,\max}\right)
$$

---

## 8. 参数汇总

| 符号 | 代码变量 | 含义 | 典型值 |
|------|---------|------|--------|
| $\alpha_c$ | `alpha_c` | Critic 学习率 | $5\times10^{-4}$ |
| $\alpha_a$ | `alpha_a` | Actor 学习率 | $5\times10^{-5}$ |
| $\gamma$ | `gamma` | 折扣因子 | $0.93$ |
| $Q_0$ | `Q[0]` | 比例误差代价 | $1000.0$ |
| $Q_1$ | `Q[1]` | 积分误差代价 | $0.05$ |
| $Q_2$ | `Q[2]` | 微分误差代价 | $150.0$ |
| $R$ | `R` | 控制能量代价 | $0.01$ |
| $I_{\max}$ | `maxIntegral` | 积分限幅 | $10.0$ |
| $u_{\max}$ | `maxOutput` | 输出限幅 | $10000.0$ |
| $K_{\text{ff}}$ | `Kff` | 前馈系数 | $0.0$ |
| $\Delta t$ | `dt` | 采样周期(s) | $0.002$ |
| $W_{c,\max}$ | `Wc_max` | Critic 权重限幅 | $100.0$ |
| $W_{a,\max}$ | `Wa_max` | Actor 权重限幅 | $100.0$ |
| $\lambda$ | `lambda_de` | 微分低通系数 | $0.1$ |
| $s_0, s_1, s_2$ | `x_scale[]` | 状态归一化尺度 | $\{0.1, 200, 100\}$ |
| $\delta_{\max}$ | `td_error_clip` | TD 误差限幅 | $10.0$ |
| $\varepsilon_{\text{db}}$ | (硬编码) | 死区阈值 | $0.05$ |

---

## 9. 算法伪代码

```
算法: PRE_ADP 在线自适应控制

每控制周期 Δt 执行:
  1.  e_k = r_k - y_k                              // 计算误差
  2.  ė_k = (1-λ)·ė_{k-1} + λ·(e_k - e_{k-1})/Δt   // 低通滤波微分
  3.  e_db = (|e_k| < 0.05) ? 0 : e_k              // 死区
  4.  if |u_{k-1}| < u_max:                         // 积分+反饱和
        e_int,k += e_k · Δt
      clamp(e_int,k, ±I_max)
  5.  for i in {0,1,2}: x_{ni} = x_i / s_i          // 状态归一化
  
  // 前向控制
  6.  Φ = [x²ₙ₀, xₙ₀xₙ₁, xₙ₀xₙ₂, x²ₙ₁, xₙ₁xₙ₂, x²ₙ₂]ᵀ
  7.  J_k = W_cᵀ Φ                                  // Critic 评估
  8.  u_adp = W_aᵀ x_n                              // Actor 控制
  9.  u_ff = K_ff · (r_k - r_{k-1})                 // 前馈
  10. u_k = Q₀·e_db + Q₂·ė_k + u_adp + u_ff         // 总输出
  11. clamp(u_k, ±u_max)
  
  // 在线学习 (跳过首帧)
  if not first_run:
    12. U_{k-1} = x_{n,k-1}ᵀ Q x_{n,k-1} + R·u²_{k-1} // 效用
    13. δ_k = U_{k-1} + γ·J_k - J_{k-1}               // TD 误差
    14. clamp(δ_k, ±10)
    
    // Critic 更新 (NLMS)
    15. W_c += α_c·δ_k / (||Φ_{k-1}||² + 10⁻³) · Φ_{k-1}
    16. clamp(W_c, ±W_{c,max})
    
    // Actor 更新 (仅当未饱和)
    if |u_k| < u_max - 10⁻³:
      17. ∇ₓJ = 2P x_{n,k}                          // 值函数梯度
      18. for i in {0,1,2}:
            g_i = (2R·u_k - γ·∇ₓJ[i]) · x_{ni}      // 策略梯度
            clamp(g_i, ±10)
            W_{ai} -= α_a · g_i                     // 梯度下降
            clamp(W_{ai}, ±W_{a,max})
  
  // 状态传递
  19. x_{k-1} ← x_k,  J_{k-1} ← J_k,  u_{k-1} ← u_k
```

---

## 10. 级联 ADP (Cascade)

当需要位置-速度双环控制时，使用级联结构:

```
ADP_Calc(ADP_outer, r_pos, y_pos)     → u_outer
ADP_Calc(ADP_inner, u_outer, y_vel)   → u_final
```

$$
\boxed{u_{\text{final}} = \operatorname{ADP}_{\text{inner}}\big(\operatorname{ADP}_{\text{outer}}(r_{\text{pos}}, y_{\text{pos}}),\ y_{\text{vel}}\big)}
$$
