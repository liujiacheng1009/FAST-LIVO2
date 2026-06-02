# FAST-LIVO2 实现可改进点分析

> 基于代码仓库 `/home/jc/workspace/fastlvio2_ws/src/FAST-LIVO2` 的分析
> 日期: 2026-06-02 | 总代码量: ~7,627 行 C++ (src) + ~1,420 行头文件 (include)

---

## 一、工程层面改进

### 1.1 内存管理：从裸指针迁移到智能指针

**现状：** 核心数据结构大量使用原始 `new`/`delete` 手动管理生命周期。

| 文件 | 问题 |
|------|------|
| `vio.h` 中 `feat_map` | `unordered_map<VOXEL_LOCATION, VOXEL_POINTS*>` 存储裸指针 |
| `voxel_map.h` 中 `VoxelMapManager` | `unordered_map<VOXEL_LOCATION, VoxelOctoTree*>` 存储裸指针 |
| `voxel_map.cpp` | `BuildVoxelMap` 中 `new VoxelOctoTree` 需要手动 `delete` |
| `vio.cpp` | `VisualPoint`/`Feature` 对象大量 `new`/`delete` |
| `visual_point.cpp` | `visual_points` 向量的 `delete` 分散在各处 |
| `visual_point.cpp` | `updateReferencePatch` 中的 `delete f` |

**建议：**
- `VOXEL_POINTS*` → `std::unique_ptr<VOXEL_POINTS>` 或 `std::shared_ptr<VOXEL_POINTS>`（如果多处共享）
- `VoxelOctoTree*` → `std::unique_ptr<VoxelOctoTree>`（单一所有者）
- `VisualPoint` / `Feature` 的裸指针使用 `unique_ptr` 管理，消除手动 `delete`
- 这能消除多个自定义析构函数中的 `delete` 逻辑，减少内存泄漏风险

### 1.2 代码组织：拆分超大源文件

**现状：** 两个文件占据了 ~43% 的总代码量：

| 文件 | 行数 | 主要功能 |
|------|------|----------|
| `vio.cpp` | 1,875 | VIO 估计器核心（检索、Jacobian、EKF、生成点、更新参考patch） |
| `LIVMapper.cpp` | 1,370 | 主调度器 + 数据同步 + 回调函数 |

**建议拆分 `vio.cpp`：**
- `vio_retrieve.cpp` — `retrieveFromVisualSparseMap` (约300行)
- `vio_jacobian.cpp` — `computeJacobianAndUpdateEKF` (约450行)
- `vio_map_points.cpp` — `generateVisualMapPoints` + `updateVisualMapPoints` (约250行)
- `vio_reference.cpp` — `updateReferencePatch` (约250行)
- `vio_main.cpp` — `processFrame` + `plotTrackedPoints`

**建议拆分 `LIVMapper.cpp`：**
- `data_sync.cpp` — `sync_packages` + 传感器回调函数
- `vio_handler.cpp` — `handleVIO` 相关逻辑
- `lio_handler.cpp` — `handleLIO` 相关逻辑
- `runner.cpp` — `run` 主循环 + 初始化

### 1.3 全局静态状态移除

**现状：** 存在全局/静态变量，限制了多实例能力：

```cpp
// voxel_map.cpp
static int voxel_plane_id = 0;        // 全局唯一 ID

// LIVMapper.cpp
static int voxel_plane_id;            // 重复声明
bool preprocess_first_scan = true;
```

**建议：**
- 将 `voxel_plane_id` 改为 `VoxelMapManager` 或 `VoxelOctoTree` 的成员变量
- 将 `preprocess_first_scan` 改为 `Preprocess` 类的成员变量
- 这使得同时运行多个 SLAM 实例成为可能（例如多机器人场景）

### 1.4 线程安全加固

**现状：** 共享状态访问缺乏同步保护：

| 位置 | 问题 |
|------|------|
| `_state` (StatesGroup) | 主循环和 IMU 传播回调同时访问，无锁保护 |
| `sync_packages` 中的 `img_time_buffer` | 回调写入+主线程弹出，存在 race condition |
| `IMU_Processing.cpp` 中 `last_prop_end_time` | 在 `Process2` 中使用全局时间变量 |

**建议：**
- 为 `StatesGroup` 添加 `std::shared_mutex`，区分读/写锁
- 主循环使用 `std::atomic<bool>` 互斥信号量控制 LIO/VIO 交替
- IMU 回调中使用 `try_lock` 而非直接加锁，避免阻塞传感器输入

### 1.5 配置系统增强

**现状：** 配置检查不足：

```cpp
// LIVMapper.cpp: 典型用法
nh.param<int>("lidar_type", lidar_type, 1);
// 无值域检查 — lidar_type=99 也会静默接受
```

**建议：**
- 添加启动时参数校验：`lidar_type` 范围、`voxel_size` > 0、`max_iterations` 合理范围
- 验证相机内参 K 和畸变系数的基本数学合理性
- 检查 LiDAR-Camera 外参 `extrinsicRot`/`extrinsicTrans` 的数值范围
- 提供默认配置文件生成器，辅助用户为新传感器套件创建配置
- 将硬编码阈值提取到配置中：
  - `vio.cpp` 中的 `0.5` (深度连续性)
  - `vio.cpp` 中的 `fix_n_points` / `min_view_angle` / `max_view_angle`
  - `voxel_map.cpp` 中的平面拟合各阈值

### 1.6 注释与死代码清理

**现状：** 大量注释掉的代码（估计占总行数 15-20%），尤其在：

- `vio.cpp` — 保留了大量 debug 打印和多套备选实现
- `voxel_map.cpp` — 注释掉的 `PlaneDetection` 变体
- `IMU_Processing.cpp` — 调试输出代码块
- `LIVMapper.cpp` — 备用的 `IMU_prop` 和发布逻辑

**建议：**
- 使用条件编译（`#ifdef DEBUG_OUTPUT`）控制调试代码
- 创建 LEGACY.md 归档被注释的功能
- 使用代码版本控制：不再需要的备选实现应删除（可在 git 历史中找到）
- 清理后预计可减少 1000-1500 行死代码

### 1.7 编译与依赖管理

**现状：**
- 依赖 `rpg_vikit` 的非标准 fork，需额外构建，且 API 不向后兼容
- Sophus 使用非模板版本（旧 API）
- `ROOT_DIR` 编译时硬编码，运行时日志/保存路径不可配置
- CPU 类型检测逻辑在 CMakeLists.txt 中使用 `-march=native`，降低移植性

**建议：**
- 考虑将必要的 vikit 功能内联（相机模型、图像扭曲约 300 行关键代码），消除外部依赖
- 升级到 Sophus 模板版本或使用 Eigen 直接实现 SE(3) 操作
- `ROOT_DIR` 改为 ROS 参数或环境变量
- CMakeLists.txt 提供 `-DUSE_NATIVE_ARCH=OFF` 选项，方便跨平台交叉编译

### 1.8 const 正确性与输入验证

**现状：** 许多函数参数未使用 `const`：

```cpp
// vio.h
void retrieveFromVisualSparseMap(MatImage &img);      // 输入不应该被修改
void plotTrackedPoints(MatImage &img);                  // 不应修改图像
```

**建议：**
- 输入参数加 `const &`，明确接口语义
- 成员函数区分 `const`/非 `const` 版本
- 指针参数使用 `const *` 或添加 null 检查

### 1.9 错误处理与日志改进

**现状：**
- 仅有 `ROS_ERROR`/`ROS_WARN`，无结构化日志
- 稀疏地图检索为空或点数不足时仅跳过，输出信息不足
- 部分条件判断失败后静默返回，无错误传播

**建议：**
- 引入日志级别：`DEBUG`/`INFO`/`WARN`/`ERROR`
- 关键操作（EKF 更新、地图构建）添加性能计时日志
- 使用 `std::optional` 或 `expected<T, Error>` 模式代替静默返回
- 在 `retrieveFromVisualSparseMap` 等关键入口添加统计打印（投影点数量、内点比例）

**重构前轨迹日志基线样本（用于回归对照）：**

来源：`/home/jc/Documents/workspace/fastlivo_ws/logs/fastlivo.INFO`（行 `3547-3549`）

```text
I0602 04:23:31.789686 154019 value_logger.h:53] [VALUE:] [px, py, pz : -0.0186119, 0.0294715, -0.0023518]
I0602 04:23:31.789700 154019 value_logger.h:53] [VALUE:] [roll_deg, pitch_deg, yaw_deg : 0.16202, 0.144104, 7.17295]
I0602 04:23:31.789702 154019 value_logger.h:53] [VALUE:] [vx, vy, vz : -0.0164333, 0.00939661, 0.0140374]
```

对照建议：
- 重构后保持键名与顺序一致：`px,py,pz` / `roll_deg,pitch_deg,yaw_deg` / `vx,vy,vz`
- 优先对比时间戳邻域内位姿与速度变化趋势，而非逐字符完全一致

### 1.10 ROS 接口与消息类型

**现状：**
- 使用自定义 `livox_ros_driver` 消息头文件（stubs），而非官方 Livox ROS 驱动
- 稠密点云发布使用 `sensor_msgs::PointCloud2`，对大场景可能造成带宽压力
- 无服务接口（Service）触发保存、重置等操作
- `pcd_save_en` 只能通过配置文件启动时设定

**建议：**
- 提供 Service 接口：保存点云/重置估计器/切换模式
- 添加 Dynamic Reconfigure 支持运行时调整关键参数（如 `voxel_size`、`max_iterations`）
- 稠密点云发布增加降采样率和 ROI 过滤选项
- 提供 `tf2` 兼容的静态变换发布

---

## 二、算法层面改进

### 2.1 VIO 更新策略优化

**现状：** 当前 VIO 更新使用**全图投影**策略：

```
每帧 VIO 更新：
1. 将所有可视地图点投影到当前帧 (retrieveFromVisualSparseMap)
2. 对所有投影点计算 Jacobian (computeJacobianAndUpdateEKF)
3. 单次 EKF 更新迭代
```

**问题：**
- 每次 VIO 更新都需要对所有可视点计算 photometric error 和 Jacobian（数百到数千点），计算量大
- 使用**固定网格**（`grid_size` 像素）限制每格最多1个点，但点的质量差异未区分

**改进建议：**

1. **滑动窗口局部优化**：
   - 引入局部滑动窗口（最后 N=5-10 帧），仅对窗口内的帧和地图点做 BA
   - 支持边缘化旧帧/旧点，而不是每帧全局 EKF 更新
   - 可参考 SVO 2.0 或 VINS-Mono 的边缘化策略

2. **分级点选择策略**：
   - 高响应 Shi-Tomasi 角点 + 良好收敛状态 → 高优先级点，保留全分辨率 patch
   - 低质量或新建点 → 低优先级，跳过或使用降采样 patch
   - 每帧自适应选择固定预算的点数（如 200 高优先级 + 100 低优先级）

3. **逆合成模式优化**（IA, Inverse Compositional）：
   - 当前代码已预留 `USE_IA` 宏，但实际未启用或未充分优化
   - IA 模式将 Hessian 计算移到参考帧预计算，可大幅减少实时计算
   - 在 patch 更新频率不高时 IA 效率显著提升

4. **单像素级光度误差优化退出**：
   - 如果 high-level pyramid（低分辨率层）大部分点已是 inlier，可跳过 low-level 的迭代
   - 参考 DSO 的 early termination 策略

### 2.2 LIO 更新策略优化

**现状：** LIO 使用基于八叉树的点到面 ICP 进行 EKF 更新：

```
每次 LIO 更新：
1. 对所有下采样后的 LiDAR 点建立点到面残差（BuildResidualListOMP）
2. 构建稠密 Jacobian 矩阵（6 × 状态维数）
3. 迭代 EKF 更新（固定迭代次数 max_iterations）
```

**问题：**
- 所有 LiDAR 点都被用于 EKF 更新，但地面/墙面等平面点对估计的贡献度不同
- 固定的最大迭代次数（默认5），不考虑收敛状态
- 点到面 Mahalanobis 距离的协方差矩阵计算较为复杂

**改进建议：**

1. **自适应点采样（surfels / 关键点）**：
   - 在平面检测后进行**面元级别**的数据关联，而非单个点级别
   - 每个 VoxelPlane 只保留代表性点（平面中心 + 法向量 + 协方差）
   - 可减少 50-80% 的残差计算量，同时提升数值稳定性

2. **迭代收敛自适应控制**：
   - 监控状态更新量（`dx` 的范数）在两次迭代之间的变化
   - 当 `|dx_k - dx_{k-1}| < ε` 时提前终止
   - 当状态更新振荡时（`dx` 符号在迭代间来回跳转）使用阻尼因子

3. **基于 LiDAR 扫描模式的动态采样**：
   - 对于 Livox Avia 的非重复扫描，边缘区域的点密度低、噪声大
   - 根据点所在的扫描时间或扫描区域调整残差权重
   - 保留点云边缘的贡献但降低其权重，避免错误关联

4. **并行化构建增量式 kd-tree / 体素近邻搜索**：
   - 当前八叉树建图是增量的，但近邻搜索在最坏情况下退化为 O(n)
   - 可考虑使用 iVox (incremental Voxel) 或 ikd-Tree 实现更高效的搜索
   - FAST-LIO2的 ikd-Tree 未直接用于此

### 2.3 传感器融合与退化处理

**现状：** LIO/VIO 交替更新的融合策略：

```
sync_packages 维护状态机:
  WAIT → LIO（处理到图像时间戳） → VIO（光度更新） → WAIT
```

**问题：**
- LIO 和 VIO 完全交替，无法应对单传感器退化（如纯旋转、光照变化、动态物体）
- VIO 更新依赖于 LIO 提供的深度先验，当 LiDAR 退化时 VIO 也随之退化
- 无明确的传感器退化检测与恢复策略

**改进建议：**

1. **退化检测与模式切换**：
   - 基于 EKF 更新后的**新息**（innovation）或**后验协方差**自动检测退化
   - 如果 VIO 内点率 < 30% 或光度误差均值 > 阈值，降低 VIO 权重或完全跳过 VIO 更新
   - 如果 LIO 面匹配率 < 20%，转入纯视觉+IMU 模式（类似于 VINS）
   - 在退化时重新启用**地图重用**（回环检测 + 重定位）

2. **异构传感器在不同场景下的自适应权重**：
   - 快速运动时提高 IMU 权重
   - 弱纹理环境降低视觉权重
   - 特征丰富场景提高视觉权重，降低 LiDAR 扫描线间距带来的误差
   - 权重自适应基于各传感器新息的协方差矩阵

3. **延迟边缘化策略**：
   - 当前 LIO 和 VIO 更新后直接更新 `_state`，丢弃了历史信息
   - 引入滑动窗口边缘化：当滑出窗口时，将旧状态的信息矩阵作为先验保留
   - 这有助于保持长时间尺度上的估计一致性

4. **外参在线校准**：
   - 目前 LiDAR-Camera 外参为固定值，仅依赖重投影校验
   - 在线校准 LiDAR-IMU 时间偏移（当前代码已预留 `time_offset_lidar`，但为定值）
   - 利用 VIO 的 visual-inertial BA 结果校验并微调 LiDAR-Camera 外参

### 2.4 视觉特征管理

**现状：** 视觉点管理包括生成、更新、收敛判断：

```
generateVisualMapPoints: Shi-Tomasi 角点评分 → 选优 → 创建 VisualPoint
updateVisualMapPoints: 基于基线判断是否添加新观测帧
updateReferencePatch: NCC 评分选最优参考 patch
```

**问题：**
- 每帧生成新的视觉点，地图点数量持续增长，无明确的去冗余策略
- 最大观测帧数固定（`fix_n_points = 30`），对长期运行场景偏低
- 视觉点收敛判断为固定阈值（0.3 像素），不考虑观测噪声
- 新建视觉点直接使用 LiDAR 深度，假设深度测量是准确的

**改进建议：**

1. **视觉地图点维护策略**：
   - 添加**点的淘汰机制**：连续跟踪失败次数 > K 的点从地图移除
   - 基于**信息量评分**定期清理冗余点：对贡献小的点（低梯度、一直被遮挡）移除
   - 参考 DSO 的**点激活/去激活**机制，保持长期运行时的地图规模可控

2. **深度不确定性传递**：
   - 创建 VisualPoint 时，记录 LiDAR 深度的不确定性（基于激光束的角度和距离）
   - 在光度优化中将深度不确定性作为先验约束，而不是固定深度
   - 多帧三角化后逐步降低深度不确定性，实现从"LiDAR 初始化"到"视觉精化"的平滑过渡

3. **自适应 patch 大小与金字塔层数**：
   - 当前固定 `patch_size = 8` 和金字塔层数（3-4）
   - 对高纹理区域使用更小的 patch（如 4×4），低纹理区域使用更大的 patch（如 12×12）
   - 在快速运动时使用更少的金字塔层以减计算负载

4. **动态物体的视觉处理**：
   - 通过光度误差的时空一致性检测动态物体
   - 被标记为动态的视觉点从地图中移除，或降低其在 EKF 中的权重
   - 对于缓慢移动的物体（如行人），在窗口中跟踪其速度并作为异常值处理

### 2.5 IMU 处理增强

**现状：** IMU 处理在 `IMU_Processing.cpp` 中实现：

```
Process2: 前向传播（状态 + 协方差）+ 反向畸变校正
IMU_init: 初始化时 30 帧的静止假设
```

**问题：**
- IMU 初始化假设静止，对在运动启动的系统（如手持/车载）不准
- 反畸变使用线性插值，对高速旋转场景精度不足
- 协方差传播中未建模**时间偏移**的不确定性
- IMU bias 的随机游走噪声参数固定，未根据实际运动动态调整

**改进建议：**

1. **运动中的 IMU 初始化**：
   - 实现"视觉辅助初始化"：利用 VIO 的视觉-惯性对齐确定初始姿态
   - 使用窗口优化（而非简单平均）来估计初始 gyro bias 和重力方向
   - 参考 ORB-SLAM3 或 VINS-Mono 的初始化策略

2. **高阶 IMU 积分**：
   - 当前使用零阶保持（每个时间间隔内角速度/加速度恒定）
   - 可升级到一阶保持（线性插值两个相邻 IMU 测量之间的值）
   - 对高动态运动，考虑使用 RK4 积分代替欧拉积分

3. **IMU 偏置估计增强**：
   - 添加对**尺度因子**和**轴不对齐**参数的在线估计
   - 当 IMU 工作在极端温度或振动环境下有实际意义
   - 长期运行中定期触发偏置修正（例如静止检测时）

### 2.6 建图优化

**现状：** 使用八叉树结构管理 LiDAR 地图：

```
BuildVoxelMap: 逐点插入 → 八叉树细分 → 平面拟合 → 更新法向量
```

**问题：**
- 全局地图随运行时间持续增长，无明确的旧体素淘汰机制
- `mapSliding` 当前仅移除超出距离阈值的体素，不保留长期结构信息
- 八叉树深度固定（2），对大尺度环境精细度不足
- 平面拟合的协方差计算在远距离点误差较大

**改进建议：**

1. **多分辨率地图**：
   - 短期：高分辨率局部地图（~0.2m 体素，用于实时配准）
   - 长期：低分辨率全局地图（~1.0m 体素，用于回环和全局一致性）
   - 局部地图随传感器移动滑动，全局地图持续累积但不用于实时更新

2. **基于信息量的地图维护**：
   - 每个体素的平面估计附带**置信度**度量（基于观测次数和点分布）
   - 删除置信度高但与当前估计不一致的旧体素
   - 保留交叉区域的多平面体素（墙角等），提升退化环境的鲁棒性

3. **八叉树深度自适应**：
   - 在点云密集区域自动增加八叉树深度
   - 稀疏区域减少八叉树深度以减少内存开销
   - 参考 **FAST-LIO2** 中 iVox 的动态体素管理思路

4. **彩色点云地图**：
   - 当前有 RGB 着色能力，但仅用于发布
   - 可将视觉光度约束引入 LiDAR 点的颜色估计，生成一致性更好的彩色地图
   - 支持基于颜色的重定位和回环检测

### 2.7 回环检测与图优化

**现状：** 当前无任何回环检测或全局优化模块。

**问题：**
- 纯里程计有长期累积漂移，无法构建全局一致的地图
- 在大尺度环境或多层建筑中长时间运行后误差发散

**改进建议：**

1. **集成 LiDAR 回环检测**：
   - 使用 ScanContext 或类似全局描述子进行 LiDAR 回环检测
   - 回环匹配时进行 ICP 精化，得到相对位姿约束
   - 结合 iSAM2 / g2o 进行位姿图优化

2. **利用现有视觉特征做视觉回环**：
   - 已有 Shi-Tomasi 角点 + image patch 信息
   - 可扩展为视觉词袋模型（DBoW2），利用现有的视觉特征做重定位
   - 视觉回环比 LiDAR 扫描回环对视角变化更鲁棒

3. **长期地图管理**：
   - 将地图分为在线滑动窗口和离线全局图
   - 回环检测成功后合并全局图和当前窗口
   - 支持地图的保存和加载功能，复用之前的建图结果

---

## 三、架构层面改进

### 3.1 模块解耦与接口定义

**现状：** 模块间通过共享的 `StatesGroup` 状态耦合：

```
LIVMapper (调度器)
 ├── state_ (StatesGroup) —— 被所有模块共享
 ├── VoxelMapManager (LIO)
 ├── VIOManager (VIO)
 ├── ImuProcess (IMU)
 └── Preprocess (LiDAR 预处理)
```

**问题：**
- `StatesGroup` 是所有模块的共享可变状态，修改影响不可控
- 测试困难：无法单独测试 VIO 或 LIO 模块

**建议重构：**
```
LIVMapper (轻量级调度器)
 ├── StateEstimator  (封装 EKF 状态与协方差)
 │   ├── VIOUpdater   (纯 VIO 更新逻辑)
 │   └── LIOUpdater   (纯 LIO 更新逻辑)
 ├── ImuProcess      (IMU 传播)
 ├── VoxelMap        (LiDAR 地图管理)
 ├── VisualMap       (视觉地图管理)
 └── DataSync        (数据同步)
```

每个模块通过明确定义的接口（抽象基类）通信，`StateEstimator` 是唯一可以直接修改状态的模块。

### 3.2 测试与持续集成

**现状：** 整个项目无任何单元测试或集成测试。

**建议：**
- 为以下核心功能添加 GTest 单元测试：
  - `so3_math.h` 中的 SE(3) / SO(3) 运算（指数映射、对数映射）
  - `ImuProcess::Process2` 的 IMU 传播正确性
  - `VoxelOctoTree` 的体素插入和平面拟合
  - 状态更新（`StatesGroup::operator+` 的流形更新）
  - Jacobian 计算的数值验证
- 添加集成测试：在仿真数据或公开数据集上回放，验证轨迹误差
- 设置 CI/CD 管道（GitHub Actions）：每次 push 自动构建并运行测试

### 3.3 性能基准与回归测试

**建议：**
- 建立持续性能基准：在固定数据集上记录每帧的处理时间、内存使用量、EKF 收敛速度
- 当修改代码时自动比较性能基准，防止退化
- 提供 benchmark 脚本，一键在标准数据集（NTU VIRAL、HILTI 2022）上运行并生成评估结果

---

## 四、总结与优先级

### 🚀 高优先级（收益高，改动相对较小）

| 序号 | 改进项 | 预计收益 |
|------|--------|----------|
| 1 | 拆分超大源文件 (`vio.cpp` / `LIVMapper.cpp`) | 可维护性大幅提升，利于团队协作 |
| 2 | 清理死代码和注释块 | 减少 15-20% 代码量，降低阅读成本 |
| 3 | 全局静态变量 → 成员变量 | 支持多实例运行 |
| 4 | 配置参数校验 | 早期发现配置错误，减少调试时间 |
| 5 | 裸指针 → `unique_ptr` | 消除内存泄漏风险 |

### 🔧 中优先级（显著改进，需一定改动量）

| 序号 | 改进项 | 预计收益 |
|------|--------|----------|
| 6 | VIO 分级点选择 + 自适应 patch | 降低 VIO 计算量 30-50% |
| 7 | LIO 自适应点采样（surfel 级别） | 降低 LIO 计算量 50-80% |
| 8 | IMU 运动初始化 | 支持运动启动，提升初始化鲁棒性 |
| 9 | 退化检测与自适应模式切换 | 在退化场景中保持定位可用 |
| 10 | const 正确性 + 输入验证 | 提升代码健壮性和可读性 |
| 11 | 线程安全加固 | 消除潜在的数据竞争 bug |

### 🏗️ 高难度（架构级改动，但收益深远）

| 序号 | 改进项 | 预计收益 |
|------|--------|----------|
| 12 | 模块解耦与接口抽象 | 支持单独测试和替换各模块 |
| 13 | 滑动窗口 + 边缘化 | 长期运行估计一致性显著提升 |
| 14 | 回环检测与位姿图优化 | 消除长期累积漂移，构建全局一致地图 |
| 15 | 外参/时间偏移在线校准 | 减少标定误差，提升融合精度 |
| 16 | 多分辨率地图管理 | 平衡实时性能和全局一致性 |
| 17 | 单元测试 + CI/CD | 防止回归，确保修改不破坏已有功能 |

---

---

## 五、测试策略与回归保障体系

### 5.1 现状量化评估

对整个仓库和 ROS 工作空间进行全面搜索后的结果：

| 检查项 | 结果 |
|--------|------|
| 含 "test" 的文件 | **0 个** |
| 仿真环境/脚本 | **0 个** |
| GTest/Boost.Test/Catch 引用 | **0 处** |
| `CMakeLists.txt` 中的 test target | **0 个** |
| CI/CD 配置 (GitHub Actions/Jenkins) | **0 个** |
| 评估脚本 | 仅 1 个手动运行、依赖 `evo` 的 `evaluate_viral.py` |
| `package.xml` 中的 test_depend | 写了 `rostest`/`rosbag`，但无任何 target 消费，是死声明 |

**结论：项目零测试覆盖，质量保障完全依赖人工跑 bag + 肉眼观察。** 在这种状态下重构，任何改动都可能引入回退而无法被及时发现。

### 5.2 重构安全的核心原则

对于 SLAM 这种**状态耦合深、浮点敏感、非线性强**的系统，需要**分层渐进**的安全策略：

```
原 理：做一层, 锁一层, 再动下一层
             ↓
每步改动都用自动化手段验证"和改之前一样"
             ↓
确认无误后提交代码, CI 自动保护已改部分
```

核心思想：**不要求重构后的轨迹"更好"，只要求它和重构前"相同"**（对同一个确定性 bag 输入）。这是回归测试，不是精度评估。

### 5.3 四层安全保障体系

#### 第一层：单元测试（数学基元，隔离 ROS）

这些模块输入输出清晰、无 ROS 依赖，最适合最先覆盖：

| 模块 | 测试点 | 验证方法 |
|------|--------|----------|
| `so3_math.h` | `Exp`/`Log` 互逆性 | `Exp(Log(R)) ≈ R`，误差 < 1e-12 |
| `so3_math.h` | `leftJacobian` / `rightJacobian` | 数值差分 vs 解析 Jacobian |
| `StatesGroup::operator+` | SE(3) 流形更新正确性 | `+` 后 `Log` 回增量一致 |
| `IMU_Processing.cpp` 前向传播 | 恒定角速度下的姿态积分 | 和解析解对比，误差 < 1e-6 |
| `visual_point.cpp` 仿射 warp | 单应矩阵生成 | 验证角点经 warp 后位置匹配 |

GTest 集成方式（CMakeLists.txt 中加入）：

```cmake
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_so3_math test/test_so3_math.cpp)
  target_link_libraries(test_so3_math ${catkin_LIBRARIES} ${Sophus_LIBRARIES} ${EIGEN3_LIBRARIES})
endif()
```

#### 第二层：Bag 回放回归测试（核心防线）

这是**最重要**的安全策略。利用已有的公开数据集（NTU VIRAL、MARS LVIG），录制一个标准化测试 bag，对比重构前后的输出。

**标准测试集要求：**
- 时长：30-60 秒（足够暴露问题，不会拖慢开发迭代）
- 场景：包含快速旋转、加减速、弱纹理区域
- 大小：尽量 < 500 MB，方便放入版本管理或 CI 缓存

**对比指标（按优先级）：**

| 指标 | 对比方法 | 允许偏差 |
|------|----------|----------|
| 位姿轨迹 (TUM 格式) | `evo_ape tum baseline.txt new.txt -a` | RMSE < 1cm |
| 每帧状态量 | `diff` 各分量 (位置/姿态/速度/bias) | 位置 < 1mm, 姿态 < 0.01° |
| EKF 协方差 | Frobenius 范数对比 | < 1% 变化 |
| 每帧 VIO 内点数 | 统计分布对比 | 均值/方差变化 < 10% |
| 每帧处理耗时 | 均值/最大/P99 | 退化 < 15% |

**自动化脚本示例 (`scripts/regression_test.py`)：**

```python
#!/usr/bin/env python3
"""回归测试脚本 — 确保重构不破坏已有功能"""
import subprocess, sys, json

BAG = "test_data/regression_fragment.bag"
BASELINE = "test_data/baseline_traj.txt"
RESULT = "/tmp/regression_result/traj.txt"
THRESHOLD_APE = 0.01  # 10mm

def run_slam(bag_path, output_dir):
    """启动 ROS 节点，播放 bag，收集轨迹"""
    subprocess.run("roslaunch fastlivo_mapping mapping_avia.launch", ...)
    subprocess.run(f"rosbag play {bag_path} --clock", ...)

def compare(baseline, result):
    """用 evo 比较两条轨迹"""
    r = subprocess.run(
        f"evo_ape tum {baseline} {result} -a --no_warnings",
        capture_output=True, shell=True
    )
    stats = json.loads(r.stdout)
    return stats["ape_rmse"]

if __name__ == "__main__":
    run_slam(BAG, RESULT)
    ape = compare(BASELINE, RESULT)
    if ape > THRESHOLD_APE:
        print(f"❌ 回归: APE={ape*100:.1f}cm (阈值={THRESHOLD_APE*100:.1f}cm)")
        sys.exit(1)
    else:
        print(f"✅ 通过: APE={ape*100:.1f}cm")
```

#### 第三层：系统级精度评估（保证不退化）

比第二层更进一步——不只看"和原来一致"，还看**绝对精度**是否退化：

| 数据集 | 基线 RMSE | 允许退化 |
|--------|-----------|----------|
| NTU VIRAL outdoor_1 | 2.11 cm | < 10%（即新增误差 < 2.1mm） |
| NTU VIRAL outdoor_2 | 3.56 cm | < 10% |
| NTU VIRAL indoor_1 | 2.15 cm | < 10% |
| HILTI 2022 | 需建立基线 | < 10% |

建立方式：在 ref commit 上运行 `scripts/run_eval.py --all`，自动遍历所有数据集 sequence，计算 APE，结果存入 `test_data/baseline/`。每次重构后在 CI 中对比。

#### 第四层：CI/CD 管道

```yaml
# .github/workflows/regression.yml
name: Regression Test
on: [push, pull_request]
jobs:
  regression:
    runs-on: ubuntu-20.04
    steps:
      - uses: actions/checkout@v3
      - name: Build
        run: |
          source /opt/ros/noetic/setup.bash
          catkin_make -DCMAKE_BUILD_TYPE=Release
      - name: Unit Tests
        run: catkin_make run_tests
      - name: Bag Regression
        run: |
          python3 scripts/regression_test.py
          echo "APE=$(cat /tmp/regression_ape.txt)" >> $GITHUB_ENV
      - name: Check Threshold
        if: env.APE > 0.01
        run: exit 1
```

### 5.4 增量重构执行策略（Strangler Fig Pattern）

最安全的做法是**逐步替换，新旧并行**：

```
Step 1: 选中一个函数 → GTest 覆盖 → 确认测试通过
Step 2: 新实现写在旁边 (#ifdef NEW_IMPL / #else OLD_IMPL)
Step 3: 编译，跑回归测试 → 确认两条路径输出一致
Step 4: 运行一段时间，收集数据 → 确认无差异
Step 5: 移除旧实现和 #ifdef → 提交
```

**按此策略的操作顺序建议：**

| 阶段 | 改动内容 | 安全机制 |
|------|----------|----------|
| **0 — 准备** | 录制标准测试 bag（30s，覆盖典型运动） | — |
| **0 — 准备** | 跑一次 baseline，固定轨迹和关键中间数据 | — |
| 1 | 写 SO(3)/SE(3) 单元测试，并 CI 集成 | GTest + GitHub Actions |
| 2 | 建 bag 回放回归脚本 | `evo_ape` + 自定义 diff |
| **3 — 开始重构** | `vio.cpp` 纯拆分（不改逻辑） | 回归脚本验证轨迹比特一致 |
| 4 | const 正确性 + 死代码清理 | 回归脚本 + 编译器检查 |
| 5 | 裸指针 → `unique_ptr` | 回归脚本 + AddressSanitizer |
| 6 | 全局静态变量去除 | 回归脚本 + 多实例测试 |
| 7 | 配置校验 | 回归脚本 + 非法配置测试 |
| 8 | 模块解耦 + 接口抽象 | 回归脚本 + 单元测试增强 |
| 9+ | 算法改进（滑动窗口、回环等） | 前 8 层全部 CV |

### 5.5 快速开始（最小可行方案）

如果不想一次性投入做完整 CI，最简起步路径：

```bash
# 1. 切一段 30s bag
rosbag filter original.bag regression_fragment.bag \
  "t.to_sec() >= 100.0 and t.to_sec() <= 130.0"

# 2. 跑一次记录 baseline
rosbag play regression_fragment.bag --clock
# 另存轨迹为 test_data/baseline_traj.txt

# 3. 每次改动后手动对比
rosbag play regression_fragment.bag --clock
evo_ape tum test_data/baseline_traj.txt /tmp/new_traj.txt -a
```

这个流程不需要额外工具，只需现有数据集 + `evo`。建议把这个命令封装成 `make regression` 或一个 shell 脚本，每次重构前跑一遍。

---

## 附录：关键代码位置速查

| 功能 | 文件 | 行号参考 |
|------|------|----------|
| 主循环 | `LIVMapper.cpp` | `run()` ~L534 |
| 数据同步状态机 | `LIVMapper.cpp` | `sync_packages()` ~L884 |
| LIO EKF 更新 | `voxel_map.cpp` | `StateEstimation()` ~L672 |
| 残差构建 | `voxel_map.cpp` | `BuildResidualListOMP()` ~L330 |
| VIO 地图点投影 | `vio.cpp` | `retrieveFromVisualSparseMap()` ~L160 |
| VIO EKF 更新 | `vio.cpp` | `computeJacobianAndUpdateEKF()` ~L872 |
| 视觉点生成 | `vio.cpp` | `generateVisualMapPoints()` ~L1195 |
| 参考帧更新 | `vioul.cpp` → `vio.cpp` | `updateReferencePatch()` ~L1043 |
| IMU 传播 | `IMU_Processing.cpp` | `Process2()` ~L258 |
| LiDAR 预处理 | `preprocess.cpp` | `process()` ~L520 |
| 状态定义 | `common_lib.h` | `StatesGroup` ~L126 |
| 八叉树体素 | `voxel_map.h` | `VoxelOctoTree` ~L129 |
| 视觉点定义 | `visual_point.h` | `VisualPoint` ~L38 |
| 特征定义 | `feature.h` | `Feature` ~L65 |
| 相机帧定义 | `frame.h` | `Frame` ~L25 |
