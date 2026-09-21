# ALG_MPC 数学公式推导

> 对应源码: `device/Mpc/mpc.c` (第249-479行, `#ifdef ALG_MPC`)

---

## 1. 二阶离散状态空间模型

### 1.1 状态定义

与 PRE_MPC 的一阶误差模型不同，ALG_MPC 采用**二阶离散状态空间**:

$$
\mathbf{x}_k = \begin{bmatrix} e_k \\ \dot{e}_k \end{bmatrix}
$$

其中:
- $e_k = r_k - y_k$ 为跟踪误差
- $\dot{e}_k$ 为误差变化率 (导数近似)

### 1.2 离散动力学方程

$$
\boxed{\mathbf{x}_{k+1} = \mathbf{A} \mathbf{x}_k + \mathbf{B} u_k}
$$

其中 $\mathbf{A} \in \mathbb{R}^{2 \times 2}$, $\mathbf{B} \in \mathbb{R}^{2 \times 1}$:

$$
\mathbf{A} = \begin{bmatrix}
A_{11} & A_{12} \\
A_{21} & A_{22}
\end{bmatrix},
\quad
\mathbf{B} = \begin{bmatrix} B_1 \\ B_2 \end{bmatrix}
$$

展开为标量形式:

$$
\begin{aligned}
e_{k+1} &= A_{11} \cdot e_k + A_{12} \cdot \dot{e}_k + B_1 \cdot u_k \\[4pt]
\dot{e}_{k+1} &= A_{21} \cdot e_k + A_{22} \cdot \dot{e}_k + B_2 \cdot u_k
\end{aligned}
$$

> 源码对应: `mpc.c:294-296` — `A_mat[4] = {A11, A12, A21, A22}`, `B_vec[2] = {B1, B2}`

### 1.3 代价权重矩阵

$$
\mathbf{Q} = \begin{bmatrix}
Q_1 & 0 \\
0 & Q_2
\end{bmatrix}
$$

- $Q_1$: 误差位置权重 (对应 $e$)
- $Q_2$: 误差变化率权重 (对应 $\dot{e}$)

> 源码对应: `mpc.c:296` — `Q_mat[4] = {Q1, 0, 0, Q2}`

---

## 2. 控制时域简化

### 2.1 N_c 的声明与使用

`MPC_Init` 接受独立的 $N_c$ (控制时域) 参数，但在当前实现中**强制 $N_c = N_p$**:

$$
N_c \gets N_p
$$

这意味着预测时域和控制时域长度一致，简化了矩阵维度。未来可扩展为 $N_c < N_p$ 以减小计算量 (控制量在 $k+N_c$ 后冻结)。

> 源码对应: `mpc.c:291-292`

---

## 3. 有限时域预测

### 3.1 状态转移矩阵幂次

与 PRE_MPC 完全相同的构造方法，但使用二阶 $\mathbf{A} \in \mathbb{R}^{2 \times 2}$:

$$
\mathbf{A}^0 = \mathbf{I}_2, \quad \mathbf{A}^i = \mathbf{A} \cdot \mathbf{A}^{i-1} \quad (i = 1,\dots,N_p)
$$

> 源码对应: `mpc.c:298-303`

### 3.2 预测递推

对预测步 $b = 1, \dots, N_p$:

$$
\mathbf{x}_{k+b|k} = \mathbf{A}^b \mathbf{x}_k + \sum_{j=0}^{b-1} \mathbf{A}^{b-1-j} \mathbf{B} \cdot u_{k+j}
$$

### 3.3 堆叠形式

$$
\boxed{\mathbf{X} = \mathbf{M} \mathbf{x}_k + \mathbf{C} \mathbf{U}}
$$

其中:

$$
\mathbf{X} = \begin{bmatrix}
\mathbf{x}_{k+0|k} \\
\mathbf{x}_{k+1|k} \\
\vdots \\
\mathbf{x}_{k+N_p|k}
\end{bmatrix} \in \mathbb{R}^{2(N_p+1)},
\quad
\mathbf{U} = \begin{bmatrix}
u_k \\
u_{k+1} \\
\vdots \\
u_{k+N_p-1}
\end{bmatrix} \in \mathbb{R}^{N_p}
$$

$\mathbf{M} \in \mathbb{R}^{2(N_p+1) \times 2}$:

$$
\mathbf{M} = \begin{bmatrix}
\mathbf{I}_2 \\
\mathbf{A} \\
\mathbf{A}^2 \\
\vdots \\
\mathbf{A}^{N_p}
\end{bmatrix}
$$

$\mathbf{C} \in \mathbb{R}^{2(N_p+1) \times N_p}$ (Toeplitz 下三角结构):

$$
\mathbf{C} = \begin{bmatrix}
\mathbf{0} & \mathbf{0} & \cdots & \mathbf{0} \\
\mathbf{B} & \mathbf{0} & \cdots & \mathbf{0} \\
\mathbf{A}\mathbf{B} & \mathbf{B} & \cdots & \mathbf{0} \\
\vdots & \vdots & \ddots & \vdots \\
\mathbf{A}^{N_p-1}\mathbf{B} & \mathbf{A}^{N_p-2}\mathbf{B} & \cdots & \mathbf{B}
\end{bmatrix}
$$

> 源码对应: `mpc.c:305-323`

---

## 4. 代价函数与 QP 形式

### 4.1 块对角权重矩阵

$$
\bar{\mathbf{Q}} = \mathbf{I}_{N_p+1} \otimes \mathbf{Q} = \begin{bmatrix}
\mathbf{Q} & & & \\
& \mathbf{Q} & & \\
& & \ddots & \\
& & & \mathbf{Q}
\end{bmatrix} \in \mathbb{R}^{2(N_p+1) \times 2(N_p+1)}
$$

$$
\bar{\mathbf{R}} = R \cdot \mathbf{I}_{N_p} \in \mathbb{R}^{N_p \times N_p}
$$

### 4.2 有限时域代价

$$
J(\mathbf{U}) = \mathbf{X}^\top \bar{\mathbf{Q}} \mathbf{X} + \mathbf{U}^\top \bar{\mathbf{R}} \mathbf{U}
$$

展开:

$$
J(\mathbf{U}) = \sum_{b=0}^{N_p} \underbrace{\left(Q_1 e_{k+b|k}^2 + Q_2 \dot{e}_{k+b|k}^2\right)}_{\text{状态代价}} \;+\; R \sum_{j=0}^{N_p-1} \underbrace{u_{k+j}^2}_{\text{控制代价}}
$$

### 4.3 代入预测模型

将 $\mathbf{X} = \mathbf{M}\mathbf{x}_k + \mathbf{C}\mathbf{U}$ 代入 $J$:

$$
\begin{aligned}
J(\mathbf{U}) &= (\mathbf{M}\mathbf{x}_k + \mathbf{C}\mathbf{U})^\top \bar{\mathbf{Q}} (\mathbf{M}\mathbf{x}_k + \mathbf{C}\mathbf{U}) + \mathbf{U}^\top \bar{\mathbf{R}} \mathbf{U} \\[4pt]
&= \underbrace{\mathbf{x}_k^\top \mathbf{M}^\top \bar{\mathbf{Q}} \mathbf{M} \mathbf{x}_k}_{\text{const}} \;+\;
   2\mathbf{x}_k^\top \underbrace{\mathbf{M}^\top \bar{\mathbf{Q}} \mathbf{C}}_{\mathbf{E}^\top} \mathbf{U} \;+\;
   \mathbf{U}^\top \underbrace{(\mathbf{C}^\top \bar{\mathbf{Q}} \mathbf{C} + \bar{\mathbf{R}})}_{\mathbf{H}} \mathbf{U}
\end{aligned}
$$

### 4.4 QP 标准形式

忽略常数项，乘以 $1/2$ 得标准 QP:

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

> 源码对应: `mpc.c:325-359`

### 4.5 矩阵构建小结

| 矩阵 | 维度 | 含义 |
|------|------|------|
| $\mathbf{M}$ | $2(N_p+1) \times 2$ | 状态自由响应矩阵 |
| $\mathbf{C}$ | $2(N_p+1) \times N_p$ | 控制卷积矩阵 |
| $\bar{\mathbf{Q}}$ | $2(N_p+1) \times 2(N_p+1)$ | 块对角状态权重 |
| $\mathbf{H}$ | $N_p \times N_p$ | QP Hessian (正定) |
| $\mathbf{E}$ | $N_p \times 2$ | QP 线性项矩阵 |
| $\mathbf{L}$ | $N_p \times N_p$ | Cholesky 下三角因子 |

---

## 5. Cholesky 分解与 QP 求解

### 5.1 分解

$$
\mathbf{H} = \mathbf{L} \mathbf{L}^\top
$$

若 $H_{jj} - \sum_{k=0}^{j-1} L_{jk}^2 \leq 10^{-12}$: 分解失败 → `Solver_Ready = 0`，后续 `MPC_Calc` 将输出置零以保安全。

> 源码对应: `mpc.c:361`, `mpc.c:366-372`

### 5.2 解析解

$$
\mathbf{U}^* = -\mathbf{H}^{-1} \mathbf{E} \mathbf{x}_k
$$

### 5.3 两阶段三角回代

**前代:**
$$
\mathbf{L} \mathbf{y} = -\mathbf{E} \mathbf{x}_k
$$

**回代:**
$$
\mathbf{L}^\top \mathbf{U}^* = \mathbf{y}
$$

> 源码对应: `mpc.c:410-420`

---

## 6. 在线控制律

### 6.1 误差与误差变化率

$$
e_k^{\text{raw}} = r_k - y_k
$$

$$
\dot{e}_k = \begin{cases}
0, & \text{首帧} \\[6pt]
\dfrac{e_k^{\text{raw}} - e_{k-1}^{\text{pre}}}{\Delta T}, & \text{otherwise}
\end{cases}
$$

> 源码对应: `mpc.c:374-383`

### 6.2 死区处理

不同于 PRE_MPC 仅将死区内误差置零，ALG_MPC 在死区内**同时将参考值替换为反馈值**:

$$
\text{if } |e_k^{\text{raw}}| < \varepsilon_{\text{db}}:
\quad
\begin{cases}
r_k \gets y_k \\[4pt]
e_k \gets 0
\end{cases}
$$

此举使得死区内前馈归零 ($r_k - r_{k-1}^{\text{pre}}$ 中 $r_k = y_k$ 与上次目标无关)，避免死区边界的抖振。

> 源码对应: `mpc.c:385-390`

### 6.3 外部积分器 (带抗饱和)

ALG_MPC 将积分设计为**独立于 QP 优化的外部环节** (不同于 PRE_MPC 将积分作为增广状态纳入优化):

$$
e_I(k) = \begin{cases}
e_I(k-1) + e_k \cdot \Delta T, & u_{\max} = 0 \;\text{或}\; |u_{k-1}^{\text{pre}}| < u_{\max} - 10^{-3} \\[6pt]
e_I(k-1), & \text{otherwise (饱和时不累加)}
\end{cases}
$$

积分限幅:

$$
e_I(k) \gets \operatorname{clamp}\big(e_I(k),\; -I_{\max},\; I_{\max}\big), \quad I_{\max} > 0
$$

注意: $I_{\max}=0$ 时等效于无积分限幅 (即不限幅)。

> 源码对应: `mpc.c:392-400`

### 6.4 前馈补偿

$$
u_{\text{ff}}(k) = \begin{cases}
0, & \text{首帧} \\[4pt]
K_F \cdot (r_k - r_{k-1}^{\text{pre}}), & \text{otherwise}
\end{cases}
$$

其中 $r_{k-1}^{\text{pre}}$ 为上一拍前馈所使用的参考值 (含死区修改)。

> 源码对应: `mpc.c:402-404`

### 6.5 滚动优化

$$
\mathbf{x}_k = \begin{bmatrix} e_k \\ \dot{e}_k \end{bmatrix}
$$

$$
\mathbf{U}^* = \operatorname{CholeskySolve}\big(\mathbf{H},\; -\mathbf{E}\mathbf{x}_k\big)
$$

### 6.6 总控制输出

ALG_MPC 的总输出 = **QP 最优解 + 外部积分 + 前馈**:

$$
\boxed{u_k = U^*_0 + e_I(k) + u_{\text{ff}}(k)}
$$

> 源码对应: `mpc.c:421`

### 6.7 输出饱和

$$
u_k \gets \operatorname{clamp}\big(u_k,\; -u_{\max},\; u_{\max}\big), \quad u_{\max} > 0
$$

当 $u_{\max} = 0$ 时跳饱和 (输出不限制)。

> 源码对应: `mpc.c:423-427`

---

## 7. 等效反馈增益 (监控用)

$$
K_0 = \begin{cases}
\dfrac{U^*_0}{e_k}, & |e_k| > 10^{-6} \\[8pt]
0, & \text{otherwise}
\end{cases}
$$

$$
K_1 = \begin{cases}
\dfrac{U^*_0 - K_0 \cdot e_k}{\dot{e}_k}, & |\dot{e}_k| > 10^{-6} \\[8pt]
0, & \text{otherwise}
\end{cases}
$$

$K_0$ 可视为等效比例增益，$K_1$ 可视为等效微分增益。无约束情况下二者为常数。

> 源码对应: `mpc.c:429-431`

---

## 8. 安性保护

### 8.1 Solver_Ready 标志

若 Cholesky 分解失败 ($\mathbf{H}$ 不满秩或非正定)，`Solver_Ready` 置 0。

在线计算时检查该标志: 若求解器未就绪，输出置零并重置首帧标志，避免非法控制量输出。

> 源码对应: `mpc.c:361`, `mpc.c:366-372`

---

## 9. 与 PRE_MPC 的核心差异对比

| 特性 | PRE_MPC | ALG_MPC |
|------|---------|---------|
| 状态空间 | $[e,\; \int e]^\top$ (1阶+积分) | $[e,\; \dot{e}]^\top$ (2阶) |
| 系统矩阵 | $A_{\text{aug}} = \begin{bmatrix}A & 0 \\ \Delta t & 1\end{bmatrix}$ | $\mathbf{A} = \begin{bmatrix}A_{11} & A_{12} \\ A_{21} & A_{22}\end{bmatrix}$ (通用) |
| 控制矩阵 | $\begin{bmatrix}B \\ 0\end{bmatrix}$ | $\begin{bmatrix}B_1 \\ B_2\end{bmatrix}$ (通用) |
| 积分方式 | 作为增广状态纳入 QP 优化 | 外部累加, 直接叠加到输出 |
| 权重维度 | $Q, Q_i$ (两个标量) | $Q_1, Q_2$ (对角阵 diag) |
| 死区处理 | $e_{\text{db}} = 0$ | $e = 0$, $\,r \gets y$ (冻结前馈) |
| 积分抗饱和 | 每次 Calc 判断 `fabsf(Out)` | 每次 Calc 判断 `fabsf(Pre_Out)` |
| N_c 支持 | 无 (直接用 $N_p$) | 接受参数但简化为 $N_c = N_p$ |
| 安全性 | 无额外保护 | `Solver_Ready` 标志 |
| 适用场景 | 一阶惯性/积分类系统 | 二阶震荡/欠阻尼系统 |

---

## 10. Init 伪代码

```
Algorithm: MPC_Init (ALG_MPC)

Input:  N_p, N_c, A11, A12, A21, A22, B1, B2, Q1, Q2, R, K_F,
        I_Out_Max, Out_Max, D_T, Dead_Zone
Output: 初始化 mpc 结构体

 1.  保存所有标量参数到 mpc 字段
 2.  nc ← N_p                          // 控制时域简化为 N_p

 3.  A_mat ← [[A11, A12], [A21, A22]]
 4.  B_vec ← [B1, B2]ᵀ
 5.  Q_mat ← diag(Q1, Q2)

 6.  for i = 0..nc:                     // 计算矩阵幂
 7.      A_pow[i] ← (A_mat)^i

 8.  M ← stack(A_pow[0], ..., A_pow[nc])  // 自由响应矩阵
 9.  for b = 1..nc:                     // 控制卷积矩阵
10.      for j = 0..b-1:
11.          C[block b, col j] ← A_pow[b-1-j] · B_vec

12.  Q_bar ← kron(I_{nc+1}, Q_mat)
13.  H ← Cᵀ · Q_bar · C + R · I_{nc}
14.  E ← Cᵀ · Q_bar · M

15.  Solver_Ready ← Cholesky(H, L)      // 非正定则置 0
```

> 源码对应: `mpc.c:257-362`

---

## 11. Calc 伪代码

```
Algorithm: MPC_Calc (ALG_MPC)

Input:  reference r_k, feedback y_k
Output: mpc.Out

 1.  if N_p out of range or !Solver_Ready:
 2.      Out ← 0;  Pre_Out ← 0;  First_Run ← 1;  return

 3.  e_raw ← r_k - y_k

    // 误差变化率 (离散导数)
 4.  if First_Run:  ė ← 0
 5.  else:          ė ← (e_raw - Pre_Error) / D_T

    // 死区 (冻结参考值)
 6.  if |e_raw| < Dead_Zone:
 7.      r_k ← y_k;  e ← 0
 8.  else:
 9.      e ← e_raw

    // 外部积分 + 抗饱和
10.  if Out_Max == 0 or |Pre_Out| < Out_Max - ε:
11.      Integral += e · D_T
12.  if I_Out_Max != 0:
13.      clamp(Integral, ±I_Out_Max)

    // 前馈
14.  ff ← First_Run ? 0 : K_F · (r_k - Pre_Target)

    // QP 滚动优化
15.  x ← [e, ė]ᵀ
16.  f ← E · x
17.  U_opt ← CholeskySolve(L, -f)
18.  Out ← U_opt[0] + Integral + ff

    // 输出限幅
19.  if Out_Max != 0:  clamp(Out, ±Out_Max)

    // 等效增益 (仅监控)
20.  K[0] ← (|e| > 1e-6)   ? U_opt[0]/e        : 0
21.  K[1] ← (|ė| > 1e-6)   ? (U_opt[0]-K[0]·e)/ė : 0

    // 状态传递
22.  Pre_Error  ← e_raw
23.  Pre_Error_Dot ← ė
24.  Pre_Target ← r_k          // 注意: 经死区修改后的 r_k
25.  Pre_Out    ← Out
26.  First_Run  ← 0
```

> 源码对应: `mpc.c:364-438`

---

## 12. 参数汇总

| 符号 | 代码变量 | 含义 | 单位 |
|------|---------|------|------|
| $N_p$ | `N_p` | 预测时域长度 | - |
| $N_c$ | `N_c` | 控制时域 (当前强制 $= N_p$) | - |
| $A_{11}\sim A_{22}$ | `A11..A22` | 二阶离散状态矩阵元素 | - |
| $B_1, B_2$ | `B1, B2` | 输入矩阵元素 | rad/(step·I) |
| $Q_1$ | `Q1` | 误差位置权重 | - |
| $Q_2$ | `Q2` | 误差变化率权重 | - |
| $R$ | `R` | 控制代价权重 | - |
| $K_F$ | `K_F` | 前馈系数 | - |
| $I_{\max}$ | `I_Out_Max` | 积分限幅 ($0=$ 不限幅) | I (同输出单位) |
| $u_{\max}$ | `Out_Max` | 输出限幅 ($0=$ 不限幅) | I |
| $\Delta T$ | `D_T` | 采样/控制周期 | s |
| $\varepsilon_{\text{db}}$ | `Dead_Zone` | 死区阈值 | rad |
| $\mathbf{H}$ | `H[400]` | QP Hessian | $N_p \times N_p$ |
| $\mathbf{E}$ | `E[40]`  | QP 线性项矩阵 | $N_p \times 2$ |
| $\mathbf{L}$ | `L[400]` | Cholesky 下三角因子 | $N_p \times N_p$ |

> `MPC_MAX_NP = 20`, `MPC_MAX_NC = 10`

---

## 13. 时间复杂度

| 阶段 | 操作 | 复杂度 |
|------|------|--------|
| `MPC_Init` | 构建 A_pow | $O(N_p)$ (2×2 矩阵乘法) |
| `MPC_Init` | 构建 C | $O(N_p^2)$ |
| `MPC_Init` | 构建 H | $O(N_p^3)$ (三重循环) |
| `MPC_Init` | 构建 E | $O(N_p^2)$ |
| `MPC_Init` | Cholesky | $O(N_p^3/3)$ |
| `MPC_Calc` | 前代 + 回代 | $O(N_p^2)$ |
| `MPC_Calc` | 状态更新 | $O(1)$ |

---

## 14. 数学附录

### 14.1 二阶模型辨识方法

ALG_MPC 需要完整的 $2 \times 2$ 系统矩阵。推荐辨识流程:

1. **开环阶跃测试**: 给电机施加阶跃电流 $u_0$，记录误差 $e(t)$
2. **计算误差变化率**: $\dot{e}(t) \approx (e_{k} - e_{k-1}) / \Delta T$
3. **MATLAB 子空间辨识**:
   ```matlab
   data = iddata([e, de], u, Ts);
   sys = ssest(data, 2);      % 辨识 2 阶离散模型
   A_mat = sys.A;  B_vec = sys.B;
   ```

### 14.2 外部积分 vs 增广积分

| 方案 | 优势 | 劣势 |
|------|------|------|
| **增广积分** (PRE_MPC) | 积分权重纳入全局优化, 理论最优 | 积分限幅不易在 QP 中表达 |
| **外部积分** (ALG_MPC) | 独立限幅/抗饱和灵活, QP 维度更低 | 积分与优化解耦, 非全局最优 |

ALG_MPC 的外部积分策略在工程实践中等价于: 在 QP 最优解的基础上叠加一个限幅后的 PI 分量。

### 14.3 非限定输与无限幅

当 $u_{\max}=0$ (或 $I_{\max}=0$) 时对应功能关闭:
- `Out_Max = 0` → 输出不限幅
- `I_Out_Max = 0` → 积分不限幅

### 14.4 死区冻结参考的原因

死区内 $r_k \gets y_k$ 而非简单地 $e \gets 0$:

1. 前馈项 $K_F \cdot (r_k - r_{k-1}^{\text{pre}})$ 中 $r_k$ 临时切换为 $y_k$，使前馈不引入虚假的参考跳变
2. 死区退出时,$r_k$ 自动恢复为用户提供的原始参考,无延迟
