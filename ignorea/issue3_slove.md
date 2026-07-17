# Issue #37 解决报告 — SSH 端口横幅错误 + Ed25519 密钥误命名

## Bug 1: SSH 端口横幅错误

### 现象
运行时容器启动横幅显示 SSH 端口为 22，但实际 sshd 监听 2022：
```
echo "  sshd:22     — remote access"   # 错误
```

### 影响
- **运维人员连接失败**：看到横幅后尝试 `ssh root@<robot-ip> -p 22`，收到 `Connection refused`
- **排查时间浪费**：需翻阅 Dockerfile 才发现实际端口是 2022（Dockerfile:273 `Port 2022`）
- **deploy 脚本不一致**：`deploy-remote`、`sync-remote`、`ssh-remote` 均使用端口 2022，与横幅矛盾
- **影响范围**：每次容器启动都打印错误信息，所有首次部署的运维人员都会遇到

### 修复
```diff
- echo "  sshd:22     — remote access"
+ echo "  sshd:2022   — remote access"
```

## Bug 2: Ed25519 密钥误命名为 id_rsa

### 现象
Dockerfile:229 生成 Ed25519 密钥，但文件名使用 `id_rsa`：
```dockerfile
ssh-keygen -t ed25519 -f /home/ubuntu/.ssh/id_rsa -N '' -C "omrobot-dev"
```
条目脚本引用 `~/.ssh/id_rsa`（entrypoint-dev.sh:14）。

### 影响
- **密钥类型混淆**：`ssh-keygen -lf ~/.ssh/id_rsa` 显示算法为 ED25519，与文件名 `id_rsa`（暗示 RSA）矛盾
- **排查误导**：故障排查时运维人员看到 `id_rsa` 文件名，可能错误地尝试 RSA 相关的排查步骤
- **多密钥环境混乱**：如果用户同时有 RSA 和 Ed25519 密钥，`id_rsa` 文件名导致无法区分
- **ssh 默认行为风险**：`ssh` 客户端默认按 `id_rsa`→`id_ed25519` 顺序查找密钥，命名正确可避免歧义
- **影响范围**：影响所有开发容器，每个开发者看到错误的密钥名称

### 修复
```diff
Dockerfile:229:
-     ssh-keygen -t ed25519 -f /home/ubuntu/.ssh/id_rsa -N '' -C "omrobot-dev" && \
+     ssh-keygen -t ed25519 -f /home/ubuntu/.ssh/id_ed25519 -N '' -C "omrobot-dev" && \

Dockerfile:281:
-     /home/ubuntu/.ssh/id_rsa.pub /root/.ssh/authorized_keys
+     /home/ubuntu/.ssh/id_ed25519.pub /root/.ssh/authorized_keys

entrypoint-dev.sh:14:
-   echo "  SSH key:     ~/.ssh/id_rsa  (deploy with: deploy-remote <robot-ip>)"
+   echo "  SSH key:     ~/.ssh/id_ed25519  (deploy with: deploy-remote <robot-ip>)"
```

## 变更汇总

| 文件 | 修改 |
|------|------|
| `scripts/entrypoint-runtime.sh:17` | `sshd:22` → `sshd:2022` |
| `Dockerfile:229` | `id_rsa` → `id_ed25519` |
| `Dockerfile:281` | `id_rsa.pub` → `id_ed25519.pub` |
| `scripts/entrypoint-dev.sh:14` | `~/.ssh/id_rsa` → `~/.ssh/id_ed25519` |

## 验证

- 全仓库 `id_rsa` 残留：0 处
- 全仓库 `sshd:22` 残留：0 处
- Dockerfile `id_ed25519`：2 处（生成 + 复制）
- 入口脚本密钥引用：1 处（已更新）

## 修复后 banner 效果

### Runtime 容器启动：
```
OMRobot runtime starting...
  sshd:2022   — remote access          ← 正确端口
  controller_manager — ros2_control arm driver
  calib_node  — calibration pipeline services
```

### Dev 容器启动：
```
OMRobot development container ready.
  Workspace:   /ws
  SDK:         src/omr_hardware/third_party/realman_arm/third_party/RM_API2
  SSH key:     ~/.ssh/id_ed25519       ← 正确密钥名
```

## 非影响范围
- SSH 服务配置（`sshd_config`）未修改 — 端口一直正确配置为 2022
- deploy 脚本（`deploy-remote` 等）未修改 — 它们通过 ssh 默认 `~/.ssh/id_*` 查找密钥，不受文件名影响
- Docker 镜像无需重建 — 密钥文件名变更需重新构建镜像才生效（下次构建自动使用新名称）
