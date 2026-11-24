# G2 直接采集 LeRobot 数据集

这个程序在遥操作过程中订阅机器人数据，直接在电脑上保存 **LeRobot v3.0** 数据集，无须先生成 `.pbdat` 再转换。使用本机已经安装的 **LeRobot 0.6.2**。

## 1. 工作方式

```text
Genie Link → 机器人执行遥操作
                  ↓ 只读 GDK 订阅
机器人上的 Python → SSH 加密传输 → 电脑上的 LeRobot 写入器
```

默认通过 SSH 在机器人本机运行读取进程。程序代码随命令传入，不在机器人上安装软件或留下脚本文件；数据集写入电脑。SSH 密码由终端提示输入，不保存在程序中。

程序不会发出运动指令，也不会自动切换机器人的控制模式。Genie Link 正常用于遥操作；**本程序的开始/结束与 Genie Link 的“数据采集”按钮相互独立**。只需要本程序生成的数据时，不必再开启 APP 的原始数据录制。若想保留两份数据，可以同时开启，但 episode 边界不会自动同步。

## 2. 先检查数据流

在终端中执行：

```bash
cd /home/lpx/桌面/lerobot_dataset_collection
bash run.sh --probe --ssh agi@192.168.31.11
```

按提示输入机器人 SSH 账号的密码。IP 以当前机器人实际地址为准。使用 10G 有线连接时，可以把地址换成 `agi@10.42.1.101`。

成功后显示 `PROBE OK`，表示已经收到并解码了 3 帧同步的状态、真实控制指令和相机图像。**仅看到 SSH 登录成功，不代表所有采集接口已经验证。**

若提示没有 action，先检查机器人是否处于正常遥操作状态、Genie Link 是否已连接。程序不会用状态值代替 action，也不会自行启动机器人。若 SDK 的 topic 名称不同，可通过 `--state-topic` 和 `--action-topic` 指定，必须确认消息类型及语义一致。

## 3. 开始采集

例如采集“用左手拿起杯子”，每段 20 秒，共 5 段：

```bash
bash run.sh ./datasets/pick_cup \
  --ssh agi@192.168.31.11 \
  --task "用左手拿起杯子" \
  --duration 20 --episodes 5
```

每段开始前按 Enter，看到 `RECORDING` 后开始遥操作。每段到时自动保存，再等待下一次 Enter。

手动决定一段的长度：

```bash
bash run.sh ./datasets/pick_cup_manual \
  --ssh agi@192.168.31.11 \
  --task "用左手拿起杯子"
```

按 Enter 开始，采集过程中按一次 **Ctrl+C**，程序停止读取并保存已完成的帧。等待 `Saved episode` 和 `Dataset:` 输出后再退出。正在保存时再次强制退出，可能导致该段不完整。

输出目录必须是不存在的新目录，程序拒绝覆盖已有数据集。`--episodes` 可以在一次运行中向同一数据集写入多段；当前版本不支持跨次运行追加。

## 4. 保存的内容

| 字段 | 内容 |
| --- | --- |
| `observation.state` | `/hal/joint_state.motor_position`，实测关节位置，单位 rad |
| `action` | `/hal/joint_cmd.position`，绝对目标关节位置，单位 rad |
| `observation.images.head_color` | 头部彩色图像 |
| `observation.images.hand_left_color` | 左手彩色图像 |
| `observation.images.hand_right_color` | 右手彩色图像 |
| `source_timestamp_ns` | 当前帧在源时钟上的采样时间 |
| `state_timestamp_ns` / `action_timestamp_ns` | 状态和控制指令消息自身的时间戳 |
| `camera_timestamp_ns.*` | 每路图像原始时间戳 |
| `task_index` 等标准字段 | 由 LeRobot 生成的任务、episode、帧号和相对时间 |

关节顺序取初始化时状态与指令中共有的关节名，并写入 feature 的 `names`。未包含的状态关节记录在 `capture_manifest.json` 的 `omitted_state_joints` 中。左右夹爪实测位置（弧度）另存为 `observation.gripper`，顺序为左、右，时间戳分别存为 `gripper_left_timestamp_ns` 和 `gripper_right_timestamp_ns`。夹爪目标命令未采集，原有 `action` 维度不变。底盘轮速、深度、IMU、雷达及力矩等字段未采集。

action 是时间戳不晚于采样时间的最新真实控制指令，不是下一帧状态、位置增量或归一化值。状态和指令优先使用 `header.stamp`；Header 无有效时间时才回退到 DDS 的 `source_ts_ns`，所选来源记录在清单中。这台机器人的实测情况是 DDS 时间戳为零、Header 有效。程序会保留各路时间戳并检查时钟范围；软件对齐不等同于硬件同步曝光。

固定频率采样，状态/图像取最近帧，默认最大偏差 50 ms；action 最大允许年龄 100 ms。数据流卡住、超出同步容差或持续跟不上时，会停止并报错。有完整帧时会保存有效前缀，并在清单中标记 `interrupted_by_error`，同时返回非零退出码。强制断电或写盘失败不保证能恢复当前 episode。

```text
datasets/pick_cup/
├── meta/                   # LeRobot 元信息、任务、统计和 episode 索引
├── data/                   # Parquet，关节状态、action、时间戳等
├── videos/                 # MP4，多相机视频
└── capture_manifest.json   # 来源、采集配置和中断情况
```

## 5. 帧率、分辨率和性能

默认 30 FPS、3 路相机、图像最大宽度 640，保持比例。机器人端做 JPEG 95 质量的传输压缩，电脑端直接进行 CPU H.264 流式编码，避免存储大量临时 PNG。**不使用 GPU，不占用 TurboVLA 的显存。** 视频编码每路 1 个线程，仍会消耗 CPU 和网络资源。

网络或机器人算力不足时，先减少相机数量、分辨率或帧率：

```bash
bash run.sh ./datasets/test_head \
  --task "测试采集" --ssh agi@192.168.31.11 \
  --cameras head_color --fps 15 --max-width 480 --duration 10
```

保留原始尺寸、取消额外 JPEG 传输压缩：

```bash
bash run.sh ./datasets/native \
  --task "测试采集" --max-width 0 --transport raw --duration 10
```

`raw` 仅指 SSH 传输使用原始 RGB；最终 MP4 仍为有损 H.264。原尺寸 RGB 带宽需求很高，应使用足够快的有线网络。已通过这台机器人无线 SSH、3 相机、30 FPS、最大宽度 640 的 8 秒采集测试；长时间采集仍建议先检查网络稳定性。

支持的相机还包括 `head_stereo_left`、`head_stereo_right` 及三路头部鱼眼。必须是机器人当前已经启用的流；程序不会修改相机配置。

## 6. 验证数据集

文件夹内的 `datasets/debug_capture_20261123/` 是本次实机调试生成的 8 秒数据：22 个关节、3 路相机、240 帧。采集程序没有发出运动指令，机器人基本静止，这条用于检查格式和链路，不是有意义的任务演示。

```bash
/home/lpx/anaconda3/envs/lerobot/bin/python src/verify_dataset.py ./datasets/pick_cup
```

检查官方 LeRobot 加载、每段首尾视频解码、帧数、固定时间间隔和 action 因果时间关系。测试时不上传到 Hugging Face。

不连接机器人即可测试完整写入流程：

```bash
bash run.sh ./datasets/synthetic_test \
  --simulate --task "测试" --fps 10 --duration 1 --episodes 2 --yes
/home/lpx/anaconda3/envs/lerobot/bin/python src/verify_dataset.py ./datasets/synthetic_test
```

`--simulate` 是人工生成的测试数据，任务和清单中均有标记，不能当作机器人演示数据。

## 7. 环境与文件

本机直接使用 `run.sh` 即可，它默认调用 `/home/lpx/anaconda3/envs/lerobot/bin/python`。其他电脑可先准备带有 **LeRobot 0.6.2、NumPy、OpenCV、PyAV** 的环境，通过 `LEROBOT_PYTHON=/绝对路径/python bash run.sh ...` 指定解释器。LeRobot 版本校验是因为当前程序依赖并验证了 0.6.2 的流式编码队列接口，用于防止编码队列满时静默丢帧。

机器人端要求可以 SSH 登录、`python3` 能导入 NumPy/OpenCV，并且 `/home/agi/app/env.sh` 配置了该机器人的 GDK。可用 `--remote-env` 更改脚本路径。密码由 SSH 提示输入；自动运行请使用自己配置的 SSH 密钥。

若已经在本机配置好了匹配的 GDK Python 环境，也可以传 `--local-gdk --gdk-python /对应环境/python`。本机 GDK 与 LeRobot 不需要使用同一个 Python。该路径尚未做实机验证，默认建议使用机器人本机 GDK 的 SSH 方式。

| 文件 | 作用 |
| --- | --- |
| `src/collect.py` | 电脑端启动、episode 管理、LeRobot 写入 |
| `src/gdk_source.py` | 机器人端只读 GDK 订阅、同步和图像传输 |
| `run.sh` | 使用本机已有 LeRobot 环境的启动入口 |
| `src/verify_dataset.py` | 官方读回与时间戳检查 |
| `src/test_collection.py` | 对齐、通信和中断保存测试 |
| `docs/VALIDATION.md` | 当前已验证项目与实机验证限制 |

运行测试：`/home/lpx/anaconda3/envs/lerobot/bin/python -m unittest discover -s src -p 'test_collection.py'`。
