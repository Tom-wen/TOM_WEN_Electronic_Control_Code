# PRE_MPC 数学公式推导

> 对应源码: `device/Mpc/mpc.c` (第71-247行, `#ifdef PRE_MPC`)

---

## 1. 增广状态空间模型 (Augmented State Space)

### 1.1 原始一阶误差模型

MPC 使用一阶误差动力学作为内部预测模型:

$$
e(k+1) = A \cdot e(k) + B \cdot u(k)
$$

其中:
- $e(k) = r(k) - y(k)$ 为跟踪误差
- $u(k)$ 为控制输入
- $A \in (0, 1]$ 为误差衰减系数 ($A=1$ 表示无自然衰减)
- $B$ 为控制增益

### 1.2 积分增广

为消除稳态误差，引入积分状态 $e_I(k) = \int_0^t e(\tau) d\tau$，离散化:

$$
e_I(k+1) = e_I(k) + \Delta t \cdot e(k)
$$

### 1.3 增广系统

定义增广状态向量 $\mathbf{x}_k = \begin{bmatrix} e(k) \\ e_I(k) \end{bmatrix}$:

$$
\boxed{\mathbf{x}_{k+1} = \mathbf{A}_{\text{aug}} \mathbf{x}_k + \mathbf{B}_{\text{aug}} u_k}
$$

其中:

$$
\mathbf{A}_{\text{aug}} = \begin{bmatrix}
A & 0 \\
\Delta t & 1
\end{bmatrix},
\quad
\mathbf{B}_{\text{aug}} = \begin{bmatrix} B \\ 0 \end{bmatrix}
$$

代价权重矩阵:

$$
\mathbf{Q}_{\text{aug}} = \begin{bmatrix}
Q & 0 \\
0 & Q_i
\end{bmatrix}
$$

> 源码对应: `mpc.c:106-108`

---

## 2. 有限时域预测 (Finite Horizon Prediction)

### 2.1 状态转移矩阵幂次

定义 $\mathbf{A}_{\text{aug}}$ 的 $i$ 次幂 ($i = 0, 1, \dots, N_p$):

$$
\mathbf{A}_{\text{aug}}^0 = \mathbf{I}_2, \quad
\mathbf{A}_{\text{aug}}^i = \mathbf{A}_{\text{aug}} \cdot \mathbf{A}_{\text{aug}}^{i-1}
$$

> 源码对应: `mpc.c:110-115`

### 2.2 预测递推

对预测步 $b = 1, \dots, N_p$:

$$
\mathbf{x}_{k+b|k} = \mathbf{A}_{\text{aug}}^b \mathbf{x}_k + \sum_{j=0}^{b-1} \mathbf{A}_{\text{aug}}^{b-1-j} \mathbf{B}_{\text{aug}} \cdot u_{k+j}
$$

### 2.3 堆叠形式 (Compact Form)

将预测时域上所有时刻的状态和输入堆叠为向量:

$$
\underbrace{\begin{bmatrix}
\mathbf{x}_{k+0|k} \\
\mathbf{x}_{k+1|k} \\
\vdots \\
\mathbf{x}_{k+N_p|k}
\end{bmatrix}}_{\mathbf{X} \in \mathbb{R}^{2(N_p+1)}}
= \underbrace{\begin{bmatrix}
\mathbf{I}_2 \\
\mathbf{A}_{\text{aug}} \\
\mathbf{A}_{\text{aug}}^2 \\
\vdots \\
\mathbf{A}_{\text{aug}}^{N_p}
\end{bmatrix}}_{\mathbf{M} \in \mathbb{R}^{2(N_p+1) \times 2}}
\mathbf{x}_k
\;+\;
\underbrace{\begin{bmatrix}
0 & 0 & \cdots & 0 \\
\mathbf{B}_{\text{aug}} & 0 & \cdots & 0 \\
\mathbf{A}_{\text{aug}}\mathbf{B}_{\text{aug}} & \mathbf{B}_{\text{aug}} & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
\mathbf{A}_{\text{aug}}^{N_p-1}\mathbf{B}_{\text{aug}} & \mathbf{A}_{\text{aug}}^{N_p-2}\mathbf{B}_{\text{aug}} & \cdots & \mathbf{B}_{\text{aug}}
\end{bmatrix}}_{\mathbf{C} \in \mathbb{R}^{2(N_p+1) \times N_p}}
\underbrace{\begin{bmatrix}
u_k \\
u_{k+1} \\
\vdots \\
u_{k+N_p-1}
\end{bmatrix}}_{\mathbf{U} \in \mathbb{R}^{N_p}}
$$

即:

$$
\boxed{\mathbf{X} = \mathbf{M} \mathbf{x}_k + \mathbf{C} \mathbf{U}}
$$

> 源码对应: `mpc.c:117-135` (M 与 C 矩阵的逐元素构建)

---

## 3. 代价函数与 QP 问题

### 3.1 有限时域代价

$$
J(\mathbf{U}) = \sum_{b=0}^{N_p} \mathbf{x}_{k+b|k}^\top \mathbf{Q}_{\text{aug}} \mathbf{x}_{k+b|k} + \sum_{j=0}^{N_p-1} R \cdot u_{k+j}^2
$$

### 3.2 矩阵形式

使用块对角矩阵 $\bar{\mathbf{Q}} = \mathbf{I}_{N_p+1} \otimes \mathbf{Q}_{\text{aug}}$ 和 $\bar{\mathbf{R}} = R \cdot \mathbf{I}_{N_p}$:

$$
J(\mathbf{U}) = \mathbf{X}^\top \bar{\mathbf{Q}} \mathbf{X} + \mathbf{U}^\top \bar{\mathbf{R}} \mathbf{U}
$$

### 3.3 代入预测模型

将 $\mathbf{X} = \mathbf{M}\mathbf{x}_k + \mathbf{C}\mathbf{U}$ 代入:

$$
\begin{aligned}
J(\mathbf{U}) &= (\mathbf{M}\mathbf{x}_k + \mathbf{C}\mathbf{U})^\top \bar{\mathbf{Q}} (\mathbf{M}\mathbf{x}_k + \mathbf{C}\mathbf{U}) + \mathbf{U}^\top \bar{\mathbf{R}} \mathbf{U} \\[4pt]
&= \mathbf{x}_k^\top \mathbf{M}^\top \bar{\mathbf{Q}} \mathbf{M} \mathbf{x}_k
   + 2\mathbf{x}_k^\top \underbrace{\mathbf{M}^\top \bar{\mathbf{Q}} \mathbf{C}}_{\mathbf{E}^\top} \mathbf{U}
   + \mathbf{U}^\top \underbrace{\left(\mathbf{C}^\top \bar{\mathbf{Q}} \mathbf{C} + \bar{\mathbf{R}}\right)}_{\mathbf{H}} \mathbf{U}
\end{aligned}
$$

忽略与 $\mathbf{U}$ 无关的常数项 $\mathbf{x}_k^\top \mathbf{M}^\top \bar{\mathbf{Q}} \mathbf{M} \mathbf{x}_k$，得标准二次规划 (QP) 形式:

$$
\boxed{\min_{\mathbf{U}} \quad \frac{1}{2} \mathbf{U}^\top \mathbf{H} \mathbf{U} + \mathbf{x}_k^\top \mathbf{E}^\top \mathbf{U}}
$$

其中:

$$
\boxed{\mathbf{H} = \mathbf{C}^\top \bar{\mathbf{Q}} \mathbf{C} + R \cdot \mathbf{I}_{N_p}} \quad (N_p \times N_p)
$$

$$
\boxed{\mathbf{E} = \mathbf{C}^\top \bar{\mathbf{Q}} \mathbf{M}} \quad (N_p \times 2)
$$

> 源码对应: `mpc.c:137-171`

### 3.4 H 矩阵的逐元素计算

展开 $\mathbf{H} = \mathbf{C}^\top \bar{\mathbf{Q}} \mathbf{C} + R\mathbf{I}$:

$$
H_{ij} = \sum_{b=1}^{N_p} \left(\mathbf{C}_{b,i}\right)^\top \mathbf{Q}_{\text{aug}} \left(\mathbf{C}_{b,j}\right) + R \cdot \delta_{ij}
$$

其中 $\mathbf{C}_{b,i} \in \mathbb{R}^2$ 是 $\mathbf{C}$ 矩阵的第 $b$ 块行、第 $i$ 列对应的 2 维向量，$\delta_{ij}$ 为 Kronecker delta。

> 源码对应: `mpc.c:138-153`

### 3.5 E 矩阵的逐元素计算

展开 $\mathbf{E} = \mathbf{C}^\top \bar{\mathbf{Q}} \mathbf{M}$:

$$
E_{ik} = \sum_{b=0}^{N_p} \left(\mathbf{C}_{b,i}\right)^\top \mathbf{Q}_{\text{aug}} \left(\mathbf{M}_{b,k}\right)
$$

其中 $\mathbf{M}_{b,k} \in \mathbb{R}^2$ 是 $\mathbf{M}$ 的第 $b$ 块行、第 $k$ 列对应的 2 维向量。

> 源码对应: `mpc.c:155-171`

---

## 4. 无约束 QP 的解析解

### 4.1 最优性条件

QP 问题 $\min_{\mathbf{U}} \frac{1}{2}\mathbf{U}^\top \mathbf{H}\mathbf{U} + \mathbf{x}^\top \mathbf{E}^\top \mathbf{U}$ 的一阶必要条件:

$$
\frac{\partial J}{\partial \mathbf{U}} = \mathbf{H}\mathbf{U} + \mathbf{E}\mathbf{x} = 0
$$

由于 $\mathbf{H}$ 正定 (当 $R > 0$ 或 $Q, Q_i > 0$ 时保证)，解析解为:

$$
\boxed{\mathbf{U}^* = -\mathbf{H}^{-1} \mathbf{E} \mathbf{x}_k}
$$

### 4.2 Cholesky 分解求解

避免直接求逆，对 $\mathbf{H}$ 进行 Cholesky 分解:

$$
\mathbf{H} = \mathbf{L} \mathbf{L}^\top
$$

其中 $\mathbf{L}$ 为下三角矩阵。求解分两步:

**前代 (Forward Substitution):**
$$
\mathbf{L} \mathbf{y} = -\mathbf{E}\mathbf{x}_k \quad \Rightarrow \quad \mathbf{y}
$$

**回代 (Backward Substitution):**
$$
\mathbf{L}^\top \mathbf{U}^* = \mathbf{y} \quad \Rightarrow \quad \mathbf{U}^*
$$

> 源码对应: `mpc.c:22-64` (共享工具函数), `mpc.c:173-174` (分解), `mpc.c:204-210` (求解)

### 4.3 Cholesky 分解算法

对于 $n \times n$ 的下三角矩阵 $\mathbf{L}$:

$$
L_{jj} = \sqrt{H_{jj} - \sum_{k=0}^{j-1} L_{jk}^2}
$$

$$
L_{ij} = \frac{1}{L_{jj}} \left( H_{ij} - \sum_{k=0}^{j-1} L_{ik} L_{jk} \right), \quad i > j
$$

数值稳定性保护: 若 $H_{jj} - \sum L_{jk}^2 \leq 10^{-12}$，分解失败返回。

> 源码对应: `mpc.c:22-42`

---

## 5. 在线控制律 (Online Control Law)

### 5.1 误差与死区

$$
e_k = r_k - y_k
$$

$$
e_{\text{db}}(k) = \begin{cases}
0, & |e_k| < \varepsilon_{\text{db}} \\
e_k, & \text{otherwise}
\end{cases}
$$

> 源码对应: `mpc.c:179,182`

### 5.2 积分与抗饱和

仅在输出未饱和时积分累加:

$$
e_I(k) = \begin{cases}
e_I(k-1) + e_{\text{db}}(k) \cdot \Delta t, & |u_{k-1}| < u_{\max} - 10^{-3} \\
e_I(k-1), & \text{otherwise}
\end{cases}
$$

积分限幅:

$$
e_I(k) = \operatorname{clamp}\big(e_I(k),\; -I_{\max},\; I_{\max}\big)
$$

> 源码对应: `mpc.c:185-190`

### 5.3 滚动优化

构造当前增广状态:

$$
\mathbf{x}_k = \begin{bmatrix} e_{\text{db}}(k) \\ e_I(k) \end{bmatrix}
$$

求解 QP 得最优控制序列:

$$
\mathbf{U}^* = \operatorname{CholeskySolve}\big(\mathbf{H},\; -\mathbf{E}\mathbf{x}_k\big)
$$

取序列第一拍输出 (滚动时域 Receding Horizon):

$$
\boxed{u_{\text{mpc}}(k) = U^*_0}
$$

> 源码对应: `mpc.c:196-213`

### 5.4 前馈补偿

$$
u_{\text{ff}}(k) = \begin{cases}
0, & \text{首帧} \\
K_{\text{ff}} \cdot (r_k - r_{k-1}), & \text{otherwise}
\end{cases}
$$

### 5.5 总输出

$$
\boxed{u_k = \operatorname{clamp}\big(u_{\text{mpc}}(k) + u_{\text{ff}}(k),\; -u_{\max},\; u_{\max}\big)}
$$

> 源码对应: `mpc.c:193,213-217`

---

## 6. 等效反馈增益 (监控用)

### 6.1 等效比例增益

$$
K_e = \begin{cases}
\dfrac{U^*_0}{x_0}, & |x_0| > 10^{-6} \\[8pt]
0, & \text{otherwise}
\end{cases}
$$

### 6.2 等效积分增益

$$
K_i = \begin{cases}
\dfrac{U^*_0 - K_e \cdot x_0}{x_1}, & |x_1| > 10^{-6} \\[8pt]
0, & \text{otherwise}
\end{cases}
$$

> 注: 无约束 MPC 中 $K_e, K_i$ 理论上是常数 (由 $\mathbf{H}^{-1}\mathbf{E}$ 决定)，仅受死区/饱和影响时有微小波动。

> 源码对应: `mpc.c:220-223`

---

## 7. 级联 MPC (Cascade)

双环串级结构:

```
MPC_Calc(outer, r_pos, y_pos)         → u_outer (速度指令)
MPC_Calc(inner, u_outer, y_vel)       → u_final (电流指令)
```

数学表达:

$$
\boxed{u_{\text{final}} = \operatorname{MPC}_{\text{inner}}\big(\operatorname{MPC}_{\text{outer}}(r_{\text{pos}}, y_{\text{pos}}),\; y_{\text{vel}}\big)}
$$

> 源码对应: `mpc.c:230-234`

---

## 8. 矩阵构建伪代码

```
Algorithm: MPC_Init — 离线构建 H, E, L 矩阵

Input:  N_p, Q, Q_i, R, A, B, dt
Output: H[N_p×N_p], E[N_p×2], L[N_p×N_p]

 1.  A_aug ← [[A, 0], [dt, 1]]
 2.  B_aug ← [B, 0]ᵀ
 3.  Q_aug ← diag(Q, Q_i)

 4.  for i = 0..N_p:
 5.      A_pow[i] ← (A_aug)^i              // 矩阵幂

 6.  M ← stack(A_pow[0], ..., A_pow[N_p])   // 纵向堆叠

 7.  for b = 1..N_p:
 8.      for j = 0..b-1:
 9.          C[block=b, col=j] ← A_pow[b-1-j] · B_aug

10.  Q_bar ← kron(I_{N_p+1}, Q_aug)

11.  H ← Cᵀ · Q_bar · C + R · I_{N_p}
12.  E ← Cᵀ · Q_bar · M

13.  L ← Cholesky(H)                        // H = L·Lᵀ
```

> 源码对应: `mpc.c:73-175`

---

## 9. 在线控制伪代码

```
Algorithm: MPC_Calc — 每控制周期执行

Input:  mpc (含 H, E, L), reference r, feedback y
Output: mpc.Out (控制量)

 1.  e_k  ← r - y
 2.  e_db ← (|e_k| < dead_zone) ? 0 : e_k
 3.  if |Out_{k-1}| < maxOutput - ε:
 4.      integral ← integral + e_db · dt
 5.  integral ← clamp(integral, ±maxIntegral)

 6.  ff ← first_run ? 0 : K_ff · (r - r_{k-1})

 7.  x ← [e_db, integral]ᵀ
 8.  f ← E · x                             // QP 线性项
 9.  y_tmp ← ForwardSub(L, -f)              // L·y = -f
10.  U_opt ← BackwardSub(Lᵀ, y_tmp)         // Lᵀ·U = y
11.  Out ← U_opt[0] + ff

12.  Out ← clamp(Out, ±maxOutput)

    // 等效增益 (仅监控)
13.  K_e ← (|x[0]| > 1e-6) ? U_opt[0]/x[0] : 0
14.  K_i ← (|x[1]| > 1e-6) ? (U_opt[0] - K_e·x[0])/x[1] : 0

    // 状态传递
15.  last_reference ← r; first_run ← false
```

> 源码对应: `mpc.c:177-228`

---

## 10. 参数汇总

| 符号 | 代码变量 | 含义 | 单位 |
|------|---------|------|------|
| $N_p$ | `N_p` | 预测时域长度 | - |
| $Q$ | `Q` | 误差状态权重 | - |
| $Q_i$ | `Q_i` | 积分状态权重 | - |
| $R$ | `R` | 控制代价权重 | - |
| $A$ | `A` | 误差衰减系数 | - |
| $B$ | `B` | 控制输入增益 | rad/(step·I) |
| $I_{\max}$ | `maxIntegral` | 积分限幅 | rad·s |
| $u_{\max}$ | `maxOutput` | 输出限幅 | I (电流单位) |
| $K_{\text{ff}}$ | `Kff` | 前馈系数 | - |
| $\Delta t$ | `dt` | 采样/控制周期 | s |
| $\varepsilon_{\text{db}}$ | `dead_zone` | 误差死区阈值 | rad |
| $\mathbf{H}$ | `H[400]` | QP Hessian 矩阵 | $N_p \times N_p$ |
| $\mathbf{E}$ | `E[40]` | QP 线性项矩阵 | $N_p \times 2$ |
| $\mathbf{L}$ | `L[400]` | Cholesky 下三角因子 | $N_p \times N_p$ |

> MPC_MAX_NP = 20, 故 H/L 最大为 20×20 = 400 元素, E 最大为 20×2 = 40 元素

---

## 11. 时间复杂度分析

| 阶段 | 操作 | 复杂度 |
|------|------|--------|
| `MPC_Init` | 构建 A_pow 序列 | $O(N_p)$ (2×2 矩阵乘法) |
| `MPC_Init` | 构建 C 矩阵 | $O(N_p^2)$ (2×2 矩阵-向量乘法) |
| `MPC_Init` | 构建 H 矩阵 | $O(N_p^3)$ (嵌套循环) |
| `MPC_Init` | Cholesky 分解 | $O(N_p^3/3)$ |
| `MPC_Calc` | 前代 + 回代 | $O(N_p^2)$ |

以 $N_p=10$ 为例:
- Init: 约 2000 次浮点运算, 一次完成
- Calc: 约 100 次浮点运算/步, 适合 500Hz 实时控制

---

## 12. 数学附录

### 12.1 H 矩阵正定性条件

$\mathbf{H}$ 正定的充分条件: $R > 0$ 且 $\bar{\mathbf{Q}} \succeq 0$ (即 $Q \geq 0,\; Q_i \geq 0$)。

实际使用时 $Q > 0$ 或 $R$ 足够大即可保证 Cholesky 分解不退化。

### 12.2 与 LQR 的关系

对于终端代价不加权的有限时域 LQR，其解为:

$$
u_k^* = -\mathbf{K}_{\text{Riccati}} \mathbf{x}_k
$$

而 MPC 的滚动优化等价于在每个时刻求解一个有限时域 LQR 并仅执行第一步。当 $N_p \to \infty$ 且终端代价取稳态 Riccati 方程解时，MPC 等价于无穷时域 LQR。

### 12.3 预测时域选择

| $N_p$ | 效果 |
|-------|------|
| 1 | 退化为比例控制: $u = -\frac{E[1,1]}{H[1,1]}e$ |
| 2~5 | 快速响应, 接近 P/PD |
| 8~12 | 推荐: 兼顾平稳性与计算量 |
| 15~20 | 更平滑, 但计算量增大 |
