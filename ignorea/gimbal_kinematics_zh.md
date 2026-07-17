# 旋转矩形上线段的云台运动学

**两轴枢轴旋转与齐次变换**

分类：内部文档

作者：Lorenzo Feng  
日期：2026 年 7 月 7 日

> 本文译自 `ignore/gimbal_kinematics.pdf`（ChiefTechLabs / 南京优根机器人）。图示见原 PDF，此处仅保留图注说明。

---

## 目标

将单轴线段旋转推广为**两轴云台**：线段先绕第一云台轴 \(u_1\) 转过 \(\varphi\)，再绕与之垂直的第二轴 \(u_2\) 转过 \(\omega\)（\(\omega \in (0, \pi/2)\)），两轴均过线段**上端点**。推导中心点参数轨迹 \(C(\theta, \varphi, \omega)\)，以及从世界系到体固连系的 \(4\times 4\) 齐次变换矩阵。

---

## 目录

1. [问题设定](#1-问题设定)
2. [参考位置](#2-参考位置)
3. [Rodrigues 公式下的旋转矩阵](#3-rodrigues-公式下的旋转矩阵)
4. [线段中心轨迹](#4-线段中心轨迹)
5. [验证](#5-验证)
6. [物理解释](#6-物理解释)
7. [目标坐标系](#7-目标坐标系)
8. [齐次变换矩阵](#8-齐次变换矩阵)
9. [小结](#9-小结)
10. [顺序运动算例](#10-顺序运动算例)

---

## 1. 问题设定

矩形绕其左侧竖直边旋转，该边与 \(z\) 轴重合。一条线段（垂直于矩形平面，即平行于 \(z\) 轴）固连在矩形上。线段绕过其上端点的两轴云台旋转。

- 矩形转轴为 \(z\) 轴 \((0,0,z)\)。
- \(\theta = 0\) 时矩形位于 \(xz\) 平面；线段中心在 \((r, 0, h)\)，到 \(z\) 轴距离为 \(r\)。
- \(h\) 为中心高度，\(L\) 为半长（全长 \(2L\)）。
- **第一云台轴**（世界系，随矩形转动）：\(u_1 = (\cos\theta, \sin\theta, 0)\)。线段绕过上端点的该轴转 \(\varphi\)。
- **第二云台轴**（与第一轴垂直；\(\theta\) 给定后在世界系中固定）：\(u_2 = (-\sin\theta, \cos\theta, 0)\)。线段绕过上端点的该轴转 \(\omega\)，且 \(\omega \in (0, \pi/2)\)。
- \(u_1 \cdot u_2 = 0\)；二者均为 \(xy\) 平面内水平单位向量。\(u_2\) 由 \(u_1\) 绕 \(z\) 轴转 \(+90^\circ\) 得到。

需求：三次旋转 \((\theta, \varphi, \omega)\) 后线段中心的参数方程、目标坐标系各轴，以及齐次变换矩阵。

---

## 2. 参考位置

### 2.1 仅矩形旋转后的中心点

\[
P(\theta) = \bigl(r\cos\theta,\ r\sin\theta,\ h\bigr).
\]

### 2.2 上端点（云台中心）

线段初始竖直，上端点在中心之上 \(+L\)：

\[
T(\theta) = P(\theta) + (0,0,L) = \bigl(r\cos\theta,\ r\sin\theta,\ h+L\bigr). \tag{1}
\]

### 2.3 从上端点到中心的向量

\[
d = P - T = (0,0,-L). \tag{2}
\]

---

## 3. Rodrigues 公式下的旋转矩阵

绕单位轴 \(u\) 转角 \(\alpha\) 的 Rodrigues 公式：

\[
R_u(\alpha) = I + \sin\alpha\,[u]_\times + (1-\cos\alpha)\,[u]_\times^2, \tag{3}
\]

其中 \([u]_\times\) 为反对称叉乘矩阵。

记 \(s_\theta=\sin\theta,\ c_\theta=\cos\theta,\ s_\varphi=\sin\varphi,\ c_\varphi=\cos\varphi,\ s_\omega=\sin\omega,\ c_\omega=\cos\omega\)。

### 3.1 第一次旋转 — \(R_1 = R_{u_1}(\varphi)\)

\[
R_1 =
\begin{bmatrix}
1-(1-c_\varphi)s_\theta^2 & (1-c_\varphi)s_\theta c_\theta & s_\varphi s_\theta \\
(1-c_\varphi)s_\theta c_\theta & 1-(1-c_\varphi)c_\theta^2 & -s_\varphi c_\theta \\
-s_\varphi s_\theta & s_\varphi c_\theta & c_\varphi
\end{bmatrix}. \tag{4}
\]

### 3.2 第二次旋转 — \(R_2 = R_{u_2}(\omega)\)

\[
R_2 =
\begin{bmatrix}
1-(1-c_\omega)c_\theta^2 & -(1-c_\omega)s_\theta c_\theta & s_\omega c_\theta \\
-(1-c_\omega)s_\theta c_\theta & 1-(1-c_\omega)s_\theta^2 & s_\omega s_\theta \\
-s_\omega c_\theta & -s_\omega s_\theta & c_\omega
\end{bmatrix}. \tag{5}
\]

**约束：** \(\omega \in (0,\pi/2)\)。第二云台角限制在第一象限（\(0^\circ\)–\(90^\circ\)），对应物理铰链：不能越过零闭合，也不能超过水平张开。

### 3.3 合成旋转 — \(R = R_2 R_1\)

先 \(\varphi\) 再 \(\omega\) 得到线段体姿态。第三列（线段方向 \(\hat{x}_T\)）形式简洁：

\[
\hat{x}_T = R\,e_z =
\begin{pmatrix}
s_\theta s_\varphi + c_\theta c_\varphi s_\omega \\
-c_\theta s_\varphi + s_\theta c_\varphi s_\omega \\
c_\varphi c_\omega
\end{pmatrix}. \tag{6}
\]

完整 \(3\times 3\) 乘积 \(R_2 R_1\) 的闭式冗长；实现时建议数值相乘 \(R_1\) 与 \(R_2\)。

---

## 4. 线段中心轨迹

体坐标系下中心为 \((0,0,-L)\)。经合成旋转并平移 \(T\) 后：

\[
C(\theta,\varphi,\omega) = T + R(0,0,-L)^\mathrm{T} = T - L\,\hat{x}_T
\]

\[
C(\theta,\varphi,\omega) =
\begin{pmatrix}
r\cos\theta - L\sin\theta\sin\varphi - L\cos\theta\cos\varphi\sin\omega \\
r\sin\theta + L\cos\theta\sin\varphi - L\sin\theta\cos\varphi\sin\omega \\
h + L\bigl(1 - \cos\varphi\cos\omega\bigr)
\end{pmatrix}. \tag{7}\text{–}(8)
\]

**表 1：参数定义**

| 符号 | 含义 | 范围 |
|------|------|------|
| \(\theta\) | 矩形绕 \(z\) 轴转角 | \([0, 2\pi)\) |
| \(\varphi\) | 绕 \(u_1\)（过 \(T\)）的第一云台角 | \([0, 2\pi)\) |
| \(\omega\) | 绕 \(u_2\)（过 \(T\)）的第二云台角 | \((0, \pi/2)\) |
| \(r\) | 线段到 \(z\) 轴距离 | — |
| \(h\) | \(\theta=\varphi=\omega=0\) 时中心高度 | — |
| \(L\) | 线段半长 | — |

---

## 5. 验证

**情形 1：** \(\varphi=\omega=0\)（无云台旋转）

\[
C(\theta,0,0) = (r\cos\theta,\ r\sin\theta,\ h) = P(\theta).
\]

中心回到旋转矩形上的位置。

**情形 2：** \(\omega=0\)（仅绕 \(u_1\) 单轴）

\[
C(\theta,\varphi,0) =
\begin{pmatrix}
r\cos\theta - L\sin\theta\sin\varphi \\
r\sin\theta + L\cos\theta\sin\varphi \\
h + L(1-\cos\varphi)
\end{pmatrix},
\]

与配套文档的单轴结果一致。

**情形 3：** \(\varphi=0\)（仅绕 \(u_2\) 单轴）

\[
C(\theta,0,\omega) =
\begin{pmatrix}
r\cos\theta - L\cos\theta\sin\omega \\
r\sin\theta - L\sin\theta\sin\omega \\
h + L(1-\cos\omega)
\end{pmatrix}.
\]

\(\omega\) 使线段向 \(-\,u_1\) 方向倾斜（因 \(u_2\) 相对 \(u_1\) 为 \(+90^\circ\)）。\(\theta=0\) 时 \(C(0,0,\omega)=(r-L\sin\omega,\ 0,\ h+L(1-\cos\omega))\)，中心在 \(xz\) 平面内半径为 \(L\) 的圆弧上。

**情形 4：** \(\varphi=\pi/2\)，\(\omega\) 任意（先倾倒再铰转）

\[
C(\theta,\pi/2,\omega) =
\begin{pmatrix}
r\cos\theta - L\sin\theta \\
r\sin\theta + L\cos\theta \\
h+L
\end{pmatrix}.
\]

\(z\) 变为 \(h+L\)（线段水平）；因线段已垂直于 \(u_2\) 作用平面，\(\omega\) 不再影响位置。

**情形 5：** \(\omega=\pi/2\)（第二铰完全打开）

\[
C(\theta,\varphi,\pi/2) =
\begin{pmatrix}
r\cos\theta - L\sin\theta\sin\varphi - L\cos\theta\cos\varphi \\
r\sin\theta + L\cos\theta\sin\varphi - L\sin\theta\cos\varphi \\
h+L
\end{pmatrix}.
\]

此时线段平躺在 \(xy\) 平面（高度 \(h+L\)）。\(x,y\) 可理解为：\(T\) 处 \(\varphi\) 倾倒后，再将 \((-L,0,0)\) 绕 \(z\) 转 \(\theta\)。

**情形 6：** 距离不变

对固定 \((\theta,\varphi,\omega)\)，\(C\) 到上端点 \(T\) 的距离恒为 \(L\)：\(\|C-T\|=\|L\hat{x}_T\|=L\)。中心在以 \(T(\theta)\) 为球心、半径 \(L\) 的球面上运动。

---

## 6. 物理解释

固定 \(\theta\) 时，\((\varphi,\omega)\mapsto C\) 是对 \((0,0,-L)\) 绕正交轴两次旋转的复合，落点在以 \(T(\theta)\) 为心、半径 \(L\) 的球面上。\(z\) 坐标 \(h+L(1-c_\varphi c_\omega)\) 的范围：从 \(h\)（如 \(\varphi=\pm\pi/2\) 或 \(\omega=\pi/2\)）到 \(h+2L\)（\(\varphi=\omega=\pi\) 时；但 \(\omega<\pi/2\)，故实际上限常在 \(\varphi=\pi\)、\(\omega\to 0\) 附近）。

固定 \(\varphi,\omega\) 而变 \(\theta\)，轨迹近似在绕 \(z\) 轴半径 \(r\) 的柱面上扫过，并叠加两云台角引起的复合正弦起伏，幅度约 \(L(\sin\varphi+\cos\varphi\sin\omega)\)。

---

## 7. 目标坐标系

### 7.1 坐标系定义

在线段上固连右手系 \(\{T\}\)：

- **原点：** 线段中心 \(C(\theta,\varphi,\omega)\)。
- \(\hat{x}_T\)：沿线段，自下而上。即 \(R e_z\)——两次旋转后的体 \(z\) 轴。
- \(\hat{y}_T\)：\(u_1\) 经第二次旋转后的像。先 \(\varphi\) 带动体，再绕 \(u_2\) 做 \(\omega\)，故 \(\hat{y}_T = R_2(\omega)\,u_1\)。
- \(\hat{z}_T\)：\(\hat{x}_T\times\hat{y}_T\)（补全右手系）。

### 7.2 世界系下各轴

\[
\hat{x}_T =
\begin{pmatrix}
\sin\theta\sin\varphi + \cos\theta\cos\varphi\sin\omega \\
-\cos\theta\sin\varphi + \sin\theta\cos\varphi\sin\omega \\
\cos\varphi\cos\omega
\end{pmatrix}, \tag{9}
\]

\[
\hat{y}_T = R_2(\omega)\,u_1 =
\begin{pmatrix}
\cos\theta\cos\omega \\
\sin\theta\cos\omega \\
-\sin\omega
\end{pmatrix}, \tag{10}
\]

\[
\hat{z}_T = \hat{x}_T\times\hat{y}_T
=
\begin{pmatrix}
-\cos\theta\sin\varphi\sin\omega - \sin\theta\cos\varphi \\
\cos\theta\cos\varphi - \sin\theta\sin\varphi\sin\omega \\
\sin\varphi\cos\omega
\end{pmatrix}. \tag{12}
\]

（中间形式经 \(\cos^2\omega+\sin^2\omega=1\) 及交叉项抵消后化简为上式。）

**验证：** \(\|\hat{y}_T\|=1\)；\(\hat{x}_T\cdot\hat{y}_T=0\)。

### 7.3 旋转矩阵

将各轴作为列向量（目标系 \(\to\) 世界系）：

\[
R_T = \bigl[\hat{x}_T\ \hat{y}_T\ \hat{z}_T\bigr]. \tag{13}
\]

世界向量 \(w\) 在目标系中为 \(w_T = R_T^\mathrm{T} w\)（\(R_T\) 正交）。

---

## 8. 齐次变换矩阵

### 8.1 平移分量

世界点 \(p\) 到目标系：

\[
p_T = R_T^\mathrm{T}\bigl(p - C(\theta,\varphi,\omega)\bigr).
\]

用闭式轴与 \(C\) 计算 \(R_T^\mathrm{T} C\)：

\[
R_T^\mathrm{T} C =
\begin{pmatrix}
-h\,c_\varphi c_\omega + L(1-c_\varphi c_\omega) \\
r \\
h\,s_\varphi c_\omega - L\,c_\varphi s_\omega
\end{pmatrix}. \tag{14}
\]

平移分量中 \(\theta\) 完全消掉（符合预期：以上端点为枢轴，对矩形整体绕 \(z\) 的转动“盲目”）。

### 8.2 完整 \(4\times 4\) 矩阵

世界系 \(\to\) 目标系的齐次变换：

\[
{}^{world}T_{target} =
\begin{bmatrix}
R_T^\mathrm{T} & -R_T^\mathrm{T} C \\
0_{1\times 3} & 1
\end{bmatrix}. \tag{15}
\]

对世界点 \(p_{world}\)：

\[
\begin{pmatrix} p_{target} \\ 1 \end{pmatrix}
=
{}^{world}T_{target}
\begin{pmatrix} p_{world} \\ 1 \end{pmatrix}.
\]

### 8.3 验证

- 原点 \(C\) 映射到 \((0,0,0)\)。
- 下端点 \(B=C-L\hat{x}_T\) 映射到 \((-L,0,0)\)。
- 上端点 \(T=C+L\hat{x}_T\) 映射到 \((L,0,0)\)。
- 点 \(C+\hat{y}_T\) 映射到 \((0,1,0)\)。
- \(\varphi=\omega=0\) 时：\(R_T=[e_z\ u_1\ e_z\times u_1]\)，原点在 \(P(\theta)\)；\(z\) 轴与线段对齐。
- \(\omega=0\) 时退化为配套文档的单轴情形。

---

## 9. 小结

线段中心轨迹 \(C(\theta,\varphi,\omega)\) 见式 (8)。齐次变换 \({}^{world}T_{target}\) 由轴 (9)–(12) 与平移 (14) 构成。

**实现步骤建议：**

1. 由 (1) 计算 \(T\)。
2. 由 (4)、(5) 构造 \(R_1\)、\(R_2\)。
3. 数值合成 \(R=R_2 R_1\)。
4. 中心位置 \(T - L R e_z\)；实现上优先用闭式 (8) 减少舍入误差。
5. 由 (9)–(12) 构造 \(R_T\)；平移列为 (14) 的 \(-R_T^\mathrm{T} C\)。
6. 组装 \(4\times 4\) 齐次矩阵 (15)。

---

## 10. 顺序运动算例

### 10.1 运动模型

三自由度依次驱动、从静止到终姿态。参数 \(r=2,\ L=1.5,\ h=0\)，分三阶段：

| 阶段 | \(\theta\) | \(\varphi\) | \(\omega\) | 说明 |
|------|------------|-------------|------------|------|
| 1 | \(0\) | \(0\to\pi/4\) | \(0\) | 第一云台打开 |
| 2 | \(0\) | \(\pi/4\) | \(0\to\pi/2\) | 第二云台打开 |
| 3 | \(0\to 2\pi/3\) | \(\pi/4\) | \(\pi/2\) | 矩形转到终姿态 |

全程遵守 \(\theta\in(0,2\pi/3)\)、\(\omega\in(0,\pi/2)\)。

### 10.2 可视化概览

**图 1：** 顺序运动中的四个关键姿态。左上：初始（\(\theta=0,\varphi=0,\omega=0\)）。右上：阶段 1 结束（\(\varphi=\pi/4\)）。左下：阶段 2 结束（\(\omega=\pi/2\)）。右下：终姿态（\(\theta=2\pi/3,\varphi=\pi/4,\omega=\pi/2\)）。目标系 \(\{\hat{x}_T,\hat{y}_T,\hat{z}_T\}\) 画在线段中心。（图见原 PDF。）

### 10.3 中心点轨迹

**图 2：** 全程中心点 \(C\) 轨迹。左：三维视图；右：分量随阶段演化及 XY 投影。蓝色曲线（阶段 3）因矩形扫过 \(\theta\) 而主导横向位移。（图见原 PDF。）

各阶段终点坐标：

**阶段 1 结束**（\(\theta=0,\ \varphi=\pi/4,\ \omega=0\)）：

\[
C = \Bigl(r,\ r + L\tfrac{\sqrt{2}}{2},\ h + L\bigl(1-\tfrac{\sqrt{2}}{2}\bigr)\Bigr).
\]

**阶段 2 结束**（\(\theta=0,\ \varphi=\pi/4,\ \omega=\pi/2\)）：

\[
C = \Bigl(r - L\tfrac{\sqrt{2}}{2},\ r + L\tfrac{\sqrt{2}}{2},\ h+L\Bigr).
\]

**阶段 3 结束**（\(\theta=2\pi/3,\ \varphi=\pi/4,\ \omega=\pi/2\)）：

\[
C_f =
\begin{pmatrix}
-\dfrac{r}{2} + L\dfrac{\sqrt{2}-\sqrt{6}}{4} \\[0.5em]
\dfrac{r\sqrt{3}}{2} - L\dfrac{\sqrt{2}+\sqrt{6}}{4} \\[0.5em]
h+L
\end{pmatrix}.
\]

### 10.4 时变刚体变换

\[
{}^{world}T_{target} =
\begin{bmatrix}
R_T^\mathrm{T} & -R_T^\mathrm{T} C \\
0 & 1
\end{bmatrix},
\]

轴用 (9)–(12)，平移用 (14)。代入各阶段参数轨迹得 \(T(t)\)。记 \(S_t=\sin t,\ C_t=\cos t,\ \alpha=\sqrt{2}/2\)。

#### 10.4.1 阶段 1 — \(\theta=0,\ \varphi=t,\ \omega=0\)

第一云台打开，矩形与第二铰静止。退化为单轴情形：

\[
T_1(t)=
\begin{bmatrix}
0 & -S_t & C_t & L(1-C_t)-h C_t \\
1 & 0 & 0 & -r \\
0 & C_t & S_t & -(L+h)S_t \\
0 & 0 & 0 & 1
\end{bmatrix},
\quad t\in[0,\pi/4]. \tag{16}
\]

#### 10.4.2 阶段 2 — \(\theta=0,\ \varphi=\pi/4,\ \omega=t\)

第二云台打开；第一云台保持 \(\pi/4\)：

\[
T_2(t)=
\begin{bmatrix}
\alpha S_t & -\alpha & \alpha C_t & L(1-\alpha C_t)-h\alpha C_t \\
C_t & 0 & -S_t & -r \\
-\alpha S_t & \alpha & \alpha C_t & h\alpha S_t \\
0 & 0 & 0 & 1
\end{bmatrix},
\quad t\in[0,\pi/2]. \tag{17}
\]

#### 10.4.3 阶段 3 — \(\theta=t,\ \varphi=\pi/4,\ \omega=\pi/2\)

矩形转到终位；两云台均在极限：

\[
T_3(t)=
\begin{bmatrix}
\alpha(S_t+C_t) & \alpha(S_t-C_t) & 0 & L - h\alpha C_t \\
0 & 0 & -1 & -r \\
\alpha(C_t-S_t) & \alpha(S_t+C_t) & 0 & h\alpha S_t \\
0 & 0 & 0 & 1
\end{bmatrix},
\quad t\in[0,2\pi/3]. \tag{18}
\]

**图 3：** 三阶段顺序运动中 \({}^{world}T_{target}(t)\) 各非平凡元素随进度参数的变化。上行：旋转块 \(R_{ij}^\mathrm{T}\)；下行：平移 \(t_x,t_y,t_z\)。竖虚线为阶段分界。参数：\(r=2,\ L=1.5,\ h=0\)。（图见原 PDF。）

---

## 翻译说明

- 原文：`ignore/gimbal_kinematics.pdf`（LaTeX，2026-07-07，Internal）
- 译文：本文件 `ignore/gimbal_kinematics_zh.md`
- 图 1–3 未嵌入，请对照原 PDF 查看
- 公式编号尽量与原文一致；个别矩阵排版按 Markdown 可读性做了整理
