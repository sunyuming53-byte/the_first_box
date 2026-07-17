# dais_motor SDK 后续 Issue 清单

仓库：https://github.com/ChiefTechLabs/dais_motor  
创建日期：2026-07-17  
拆分原则：单 issue 可独立实现与验收；有硬依赖的能力合并；ROS 适配层不放本仓库。

| # | 标题 | 类型 | 互不耦合说明 |
|---|------|------|--------------|
| [#2](https://github.com/ChiefTechLabs/dais_motor/issues/2) | H0B_24 相位电流位宽 | bug | 只改电流读法 |
| [#3](https://github.com/ChiefTechLabs/dais_motor/issues/3) | 故障恢复 clear_fault | enhancement | 只补恢复路径，不改已有保护 |
| [#4](https://github.com/ChiefTechLabs/dais_motor/issues/4) | InPosition / motion_done | enhancement | 只暴露到位状态 |
| [#5](https://github.com/ChiefTechLabs/dais_motor/issues/5) | 失能下切换 ControlMode | enhancement | 只改模式切换契约 |
| [#6](https://github.com/ChiefTechLabs/dais_motor/issues/6) | 相对位置指令 | enhancement | 不依赖回零 |
| [#7](https://github.com/ChiefTechLabs/dais_motor/issues/7) | Homing / 清零 | enhancement | 不依赖相对位置 |
| [#8](https://github.com/ChiefTechLabs/dais_motor/issues/8) | 软停 vs 失能 | enhancement | 与故障保护路径分离 |
| [#9](https://github.com/ChiefTechLabs/dais_motor/issues/9) | 离线单测 | test | 不改控制语义 |
| [#10](https://github.com/ChiefTechLabs/dais_motor/issues/10) | 外部脉冲模式 | enhancement | 与 H11 通信位置并行 |
| [#11](https://github.com/ChiefTechLabs/dais_motor/issues/11) | 转矩模式 | enhancement | 与速度/位置并行 |

## 有意未挂到本仓库的项

- `DaisHardware` 导出 `HW_IF_POSITION`、ROS 侧急停策略 → 属于 `omrobot` / `omr_hardware`，不是 `dais_motor` SDK 范围。
- 流式轨迹插补 → 依赖产品需求，暂不建 issue，避免与现有 H11 点到点耦合。
