# 自研六轴机械臂控制实验室

独立的模型审计、MuJoCo 六轴适配和低速控制实验项目。原项目 `robot-arm-compliant-control-lab` 保持只读；只复用提交 `7aec01379ff9a8b135cbac75f18102eb9a27ea8f` 的控制算法模块。当前支持无接触仿真的运动学、Jacobian、重力补偿和笛卡尔阻抗，尚无实物通信入口。

实物电机由用户确认为达妙 4310，用户提供了已在实物验证重力补偿的 Robot-Defender 代码和视频。它们作为独立参考保存，不能据此认定本项目控制器已经通过实物验证。电机反馈的物理定义、单位、减速器侧别、固件模式仍待核对。

## 重力补偿实物展示

[展示页](showcase/index.html)播放作者提供的原始实物录像，支持慢放、关键帧跳转和手机浏览。实物测试在其他设备上进行，目前没有配套遥测数据；页面只展示录像与原理说明。

```bash
rtk proxy python3 /home/lyh/custom-6dof-control-lab/tools/prepare_showcase_media.py
rtk proxy python3 -m http.server 8766 --bind 127.0.0.1 --directory /home/lyh/custom-6dof-control-lab/showcase
```

访问 <http://127.0.0.1:8766>。素材准备使用本机已有 OpenCV，详情见 [展示页说明](showcase/README.md)。视频和截帧仅保存在本地，不上传 GitHub。

缺少实测日志时，可先开展仿真算法改进；[候选方案与验收指标](docs/algorithm_options.md)列出了重力模型敏感性分析、力矩余量约束、奇异位形检查和扰动观测的实施顺序。这些是待验证方案，不是已取得的性能提升。

## 安装与运行

当前验证环境：Linux x86_64、Python 3.10、MuJoCo 3.12.0。依赖固定在 `requirements.lock`，仅安装进本目录 `.venv`。`-I` 隔离系统 ROS/PYTHONPATH 与用户包；不要在原项目环境中安装或运行。

```bash
cd /home/lyh/custom-6dof-control-lab
rtk proxy bash tools/install.sh
rtk proxy .venv/bin/python -I -m custom6dof.audit --output reports/model_audit.json
rtk proxy .venv/bin/python -I tools/build_model.py
rtk proxy .venv/bin/python -I -m pytest
rtk proxy .venv/bin/python -I -m custom6dof.experiments --mode gravity --duration 2 --output reports/runs/gravity
rtk proxy .venv/bin/python -I -m custom6dof.experiments --mode impedance --output reports/runs/impedance
rtk proxy .venv/bin/python -I -m custom6dof.experiments --mode impedance --perturb --output reports/runs/perturb
```

可视化（本机图形会话）：

```bash
rtk proxy .venv/bin/python -I -m custom6dof.viewer
rtk proxy env MUJOCO_GL=egl .venv/bin/python -I -m custom6dof.viewer --snapshot reports/model_preview.png
```

GitHub 私有仓库保存代码、模型派生描述、算法依赖 wheel 和校验清单。原始 CAD/视频及解压参考代码仅保存在本地；新克隆需要先恢复输入。首次导入（脚本按本机用户给定路径读取，目标存在时拒绝覆盖）：

```bash
rtk proxy python3 tools/preserve_inputs.py
rtk proxy python3 tools/preserve_defender.py
```

如果从 GitHub 克隆，`inputs/manifest.json` 已存在，使用 `rtk proxy python3 tools/restore_inputs.py` 校验并恢复本机输入；可用 `--zip`、`--processed`、`--advice` 指定其他位置。添加成对的 `--defender /path/Robot-Defender.zip --video /path/demo.mp4` 可恢复离线固件参考；不恢复时该项检查标为跳过。脚本验证 SHA256、拒绝不同内容与符号链接目标，不删除已有工作。算法 wheel 已随项目保存；仅在重新构建依赖时需要原仓库 Git 对象。不能 editable 安装原项目，也不将它的 `src` 加入搜索路径。

## 模型和控制范围

`models/tool4_v2/scene.xml` 为当前版本：6 个 hinge、6 个单位传动比理想力矩执行器、18 个关节位置/速度/执行器力矩仿真传感器。保留 URDF 的轴、原点、质心和惯量；去掉 `Link_tool` 的重复质量和几何，保留固定关节变换与工具坐标。`tool_site` 在导出的工具坐标原点，还不是经过实测的接触 TCP。

原底座 425161 面无法直接导入本机 MuJoCo。派生模型采用已有简化网格做显示，**全部接触关闭**；叉形空腔、摩擦、自碰撞和环境接触尚未验证。视觉网格精度不代表碰撞精度。仿真中的零摩擦、零电机惯量表示理想化省略，不是测量结果。

原始限位 ±3.14 rad、速度 1 rad/s、effort 100 N·m 未获硬件验证。新仿真设每轴 ±10 N·m 作为显式数值实验限幅，不代表电机额定/峰值能力；建议文档的 7.9 N·m 也不是电机规格。增益和阻尼在 `configs/simulation.json` 单独设置，未继承 Panda 的 12 N 任务或默认增益。

阻抗控制律为 `τ = Jᵀw + g(q) − Dq̇`，随后按执行器映射限幅。`w` 由固定版本上游 `FrankaImpedanceController` 产生，采用世界系工具原点上的 `[F; M]`；Jacobian 对应 `[线速度; 角速度]`。六轴不添加零空间姿态项。重力在独立零速度 MuJoCo 状态中计算，避免把科氏项混叫重力；另用 URDF 势能差分独立核验。

初始姿态经过数值 Jacobian 条件检查，替换了近奇异零位；它只是无接触仿真初始值，不是已经确认的实物安全姿态。工具坐标旋转与力矩参考点都保留。上游姿态误差为小角形式，本项目限制为小位移/小转角，不声称大角度全局收敛。

日志分为 `sim_*` 仿真观测、`requested_tau_*` 控制命令、`eval_external_tau_*` 评价扰动真值。没有外力观测和外力估计时写 `null`；上游 `normal_force=0` 只是阻抗接口兼容占位。评价扰动不输入控制器，不将仿真执行器力矩解释为末端外力。

## 源码入口

| 内容 | 入口 |
|---|---|
| 原始输入/来源与哈希 | `inputs/manifest.json`、`references/robot_defender/manifest.json` |
| 审计工具与审计报告 | `src/custom6dof/audit.py`、`reports/model_audit.json` |
| 独立 URDF 运动学/势能 | `src/custom6dof/kinematics.py` |
| 版本模型生成 | `tools/build_model.py`、`models/*/manifest.json` |
| 六轴 MuJoCo 适配器 | `src/custom6dof/adapter.py` |
| 仿真任务/验收与日志 | `src/custom6dof/experiments.py`、`tests/` |
| 原版算法来源 | `dependencies/upstream-lock.json`、`tools/build_upstream_wheel.py` |
| 安装后的算法 | `.venv/lib/python3.10/site-packages/compliant_control_lab/franka_control.py` |
| 参数来源与未知项 | `docs/parameter_provenance.md`、`configs/hardware_unknowns.json` |
| Robot-Defender/STM32 对照 | `docs/robot_defender_review.md`、`tools/defender_parity.py` |

详细验收与后续工作见 `docs/phase1_report.md`。后续顺序为：先确认电机映射/力矩语义与模型质量分布，再建立达妙离线记录格式和回放；接着验证外力估计，最后增设经过验证的接触几何、导纳与混合控制。BC/PPO 权重和七轴 C++ 不迁移，本阶段不评价强化学习优劣。
