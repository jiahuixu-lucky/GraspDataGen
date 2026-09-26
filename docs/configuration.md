# 配置约定

所有配置内路径都相对于仓库根目录。运行命令前先进入仓库根目录。
配置只使用明确的文件引用和一层参数覆盖，不支持递归继承、环境变量插值或任意新字段。

| 文件 | 负责的内容 |
| --- | --- |
| `configs/objects/our_assets.yaml` | 当前物体的源 USD、原始坐标系朝上方向、质量和材质 |
| `configs/objects/bottle_matryoshka.yaml` | 旧 bottle/matryoshka 数据集的同类定义 |
| `configs/robots/*.yaml` | 导出使用的机器人名称、源 USD、TCP 和夹爪关节/刚体名称 |
| `configs/grippers/*.yaml` | 提取范围、碰撞体、夹爪坐标轴、接触区域、材质和标定文件路径 |
| `configs/calibration.yaml` | 两个夹爪共用的物理标定参数 |
| `configs/parameters.yaml` | 共用的采样、抓取姿态判据、物理验证参数及场景摆放偏移 |
| `configs/runs/*.yaml` | 本次使用哪些资产、输出到哪里、使用多少资源，以及任务特有的参数差异 |
| `configs/experiments/*.yaml` | 历史实验的任务参数；使用前检查资产、GPU 和标注目录 |
| `configs/audit.yaml` | 物理反例和独立敏感性检查；供 `audit --config` 使用 |

## 物理参数的唯一来源

物体 YAML 的 `physics` 是实际质量、静摩擦、动摩擦和恢复系数的唯一输入。
`physics.source` 记录数值的来源或覆盖原因，不再要求另一份 JSON 重复保存相同数值。
源 USD 仍用于读取几何、碰撞设置和惯性等属性，原有惯性一致性检查保持有效。
例如奶茶杯保留配置中的摩擦系数 0.8，并明确注明它覆盖了源 metadata 中的 1.0。

`up_axis` 是物体原始根坐标系中的单位向量；`wrist_up_axis_base` 是夹爪基座
坐标系中的安装侧朝上单位向量。两者随资产定义保存，不在各个运行任务中重新声明。
姿态允许范围和底面间隙属于任务判据，放在共享参数的 `posture` 中。

机器人驱动和关节限制固定继承 USD。已删除必须写成 `null` 的四个 actuator 字段、
重复的 actuator 关节列表，以及没有计算消费者的 `ee_body` / `T_ee_B` 和 URDF 记录链路。
TCP 父路径及其完整变换链仍会校验。

## 运行参数与覆盖

`parameters` 必须指向一个完整的共享参数文件。生产任务直接使用共享值；GUI 任务
需要更多 roll，因此仅写出差异：

```yaml
parameters: configs/parameters.yaml
manifest: configs/objects/our_assets.yaml
objects: [shampoo_000]
sampling:
  rolls_per_contact: 16
```

这是任务配置的片段；其余资产、资源和预算字段参见 `configs/runs/gui-test.yaml`。

- `sampling`、`posture`、`validation` 可以省略：省略时使用共享文件的完整参数；
  指定时只覆盖该节已有的字段。GUI 和历史实验使用这一机制表达实际差异。
- `objects` 可以省略：省略时处理整个物体清单；指定时选择清单中已有的唯一名称。
  GUI 选择单个物体、旧实验选择 bottle，无需再复制一份物体及其物理参数。
  执行顺序保持物体清单的顺序。
- `grasp_regions` 可以省略：省略时进行全表面采样，解析后的 `None` 专门表示这一情况。
  指定时目录必须存在；存在匹配的物体 NPZ 时限制其可采样表面，缺少该物体 NPZ 时
  仍对该物体进行全表面采样。文件存在但几何不匹配时会报错。
- `calibration` 始终是文件路径，不同时接受路径、字典或 `null`。
  `prepare` 与 `generate` 都从该文件读取标定步频；验证步频单独由 `validation` 控制。

物理验证阈值、求解器设置和 GPU 容量都有实际消费者。集中维护这些字段不代表删除
其控制能力；不要把 `surface_rounds` 和 `candidate_budget` 合并，前者限制几何采样，
后者限制实际进入物理验证的候选数量。

`target_successes`、`candidate_budget`、`time_budget_s` 都按物体/夹爪组合计算。
时间预算不含资产准备，运行中的一个批次也不会被中途强行截断。

## 快照、缓存与旧任务

加载后会保存展开的有效参数快照，记录共享参数与任务差异合并后的实际值。
多 GPU 启动器从任务文件生成分片配置，各分片仍走同一加载流程。
恢复身份包含共享参数、标定文件、物体清单、夹爪与机器人配置以及实现代码；
修改这些输入需要使用新的输出目录。

本次配置格式迁移保留原任务的有效采样、物理参数、资产和预算，但改变了配置格式
及实现代码，因此不能对迁移前的任务继续使用 `--resume`。旧数据和缓存不删除，
保留它们仍可用于既有数据的回放。新任务会按新配置身份建立新的准备缓存。

旧 `configs/objects/production.yaml` 已改名为 `bottle_matryoshka.yaml`；独立的
`gui-test.yaml`、`region_test.yaml` 物体清单由 `objects` 选择取代。
审计参数移动到 `configs/audit.yaml`；两份历史 bottle/profile 任务移到
`configs/experiments/`，其中配置的区域标注目录需要另行准备。
