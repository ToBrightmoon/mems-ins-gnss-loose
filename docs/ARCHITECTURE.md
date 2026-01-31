# 项目架构说明

## 模块划分

### 1. 状态管理 (state/)

- **state.h**: 定义系统状态结构
  - `InsState`: 惯导状态（位置、速度、姿态）
  - `NominalState`: 名义状态（包含零偏）
  - `GIState`: 误差状态（用于滤波）

### 2. 测量数据 (measurement/)

- **imu.h**: IMU数据结构定义
- **gnss.h**: GNSS数据结构定义
- **nav_result.h**: 导航结果数据结构
- **ref.h**: 参考数据（真值）结构
- **imu_time_aligner.h/cpp**: IMU时间对齐工具

### 3. 惯导解算 (ins/)

- **pins.h/cpp**: 纯惯导解算(Pure INS)实现
  - 基于IMU数据进行位置、速度、姿态积分
  - 考虑地球自转效应
  - 使用简单的积分方法

### 4. 滤波器 (filter/)

- **filter_model.h**: 滤波模型定义
  - IMU噪声模型
  - GNSS噪声模型
  - 初始状态噪声模型

- **gins_config.h/cpp**: GINS配置管理
  - 从YAML文件加载配置
  - 构建噪声矩阵
  - 构建初始状态

- **gi_loose_filter.h/cpp**: 松耦合滤波器
  - 基于误差状态卡尔曼滤波(ESKF)
  - 实现状态传播和更新

### 5. 姿态估计器 (estimator/)

- **pose_estimator.h/cpp**: 组合导航姿态估计器
  - 集成INS和滤波器
  - 处理IMU和GNSS数据
  - 实现松耦合组合导航

### 6. 输入输出 (io/)

- **imu_data_parser.h**: IMU数据解析器
- **gnss_data_parser.h**: GNSS数据解析器
- **nav_result_parser.h**: 导航结果解析器
- **ref_data_parser.h**: 参考数据解析器
- **nav_result_saver.h/cpp**: 导航结果保存器
- **sensor_data_provider.hpp**: 传感器数据提供者（模板类）

### 7. 工具函数 (utils/)

- **constants.h**: 常量定义
- **earth_model.h/cpp**: 地球模型相关计算（WGS84 椭球、子午圈曲率半径等）
- **rotation.h**: 旋转相关工具函数
  - 欧拉角与旋转矩阵转换
  - 欧拉角与四元数转换
  - 旋转向量转换
- **logger.h/cpp**: 日志系统（基于 spdlog，替代 cout/cerr）
- **exceptions.h**: 异常类定义
  - `ConfigException`、`FileException`、`DataException`、`ValidationException`、`TimeException` 等

### 8. 评估工具 (eval/)

- **ins-eval.cpp**: INS单独评估程序
- **estimator_eval.cpp**: 估计器评估程序
- **pose_evaluator.cpp**: 姿态评估程序
- **nav_evaluator.hpp**: 导航评估器类
  - 计算位置和姿态误差
  - 统计RMSE、最大值等指标

## 数据流

```
IMU数据 → IMU解析器 → 时间对齐 → INS解算 → 状态传播
                                                      ↓
GNSS数据 → GNSS解析器 → 时间对齐 → 滤波器更新 ← 误差状态
                                                      ↓
                                              姿态估计器 → 结果保存
```

## 坐标系定义

- **导航坐标系**: 北东地(NED)
- **载体/IMU 坐标系**: 前-右-下(FRD)
- **姿态表示**: 四元数（body → nav）
- **姿态误差**: 定义在导航下，采用左扰动模式

## 时间基准

- 以IMU时间作为系统时间基准
- 所有传感器数据对齐到IMU时间
- 滤波结果根据IMU时间触发

## 算法特点

1. **惯导解算**
   - 考虑地球自转效应
   - 使用WGS84椭球模型
   - 简单积分方法

2. **误差状态卡尔曼滤波**
   - 名义状态 + 误差状态
   - 误差状态定义在载体坐标系
   - 右扰动模式

3. **松耦合组合**
   - GNSS位置作为观测
   - 位置误差作为观测量
   - 不考虑GNSS速度观测

## 扩展性

项目设计时考虑了以下扩展点：

1. **初始对准**: 预留了初始对准接口
2. **安装误差**: 当前未考虑，但结构支持扩展
3. **更复杂的积分方法**: INS模块可以替换为更复杂的积分器
4. **紧耦合**: 可以扩展为紧耦合组合导航
