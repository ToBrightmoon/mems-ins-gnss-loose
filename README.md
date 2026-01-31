# MEMS INS/GNSS 组合导航系统

一个用于测试初始状态下的MEMS惯性导航系统(INS)与GPS组合导航的程序，用于获取位置和姿态结果。

## 项目简介

本项目是一个**低精度、非极区**的惯导卫星组合导航系统，参考牛小骥老师的《惯性导航原理与算法设计》一书实现的实验性代码。

### 主要特点

- 使用北东地(NED)作为导航坐标系
- 姿态误差定义在导航坐标系，采用左扰动模式
- IMU 坐标系为前-右-下(FRD)
- 以IMU时间作为时间基准，滤波结果根据IMU时间触发
- 惯导解算考虑了地球自转效应
- 不考虑安装误差等因素，仅考虑零偏
- 使用简单的积分方式进行惯导解算
- 基于Eigen库进行滤波计算

## 项目结构

```
mems-ins-gps/
├── CMakeLists.txt          # CMake 构建配置
├── config/                 # 配置文件目录
│   └── config.yaml        # 系统配置文件
├── data/                   # 数据文件目录
│   ├── imu.txt            # IMU 数据
│   ├── gnss.txt           # GNSS 数据
│   └── truth.nav          # 真值数据
├── estimator/              # 姿态估计器
│   ├── pose_estimator.h
│   └── pose_estimator.cpp
├── eval/                   # 评估工具
│   ├── ins-eval.cpp       # INS 评估
│   ├── estimator_eval.cpp # 估计器评估
│   ├── pose_evaluator.cpp # 姿态评估
│   └── nav_evaluator.hpp  # 导航评估器
├── filter/                 # 滤波器模块
│   ├── gi_loose_filter.h/cpp  # 松耦合滤波器
│   ├── gins_config.h/cpp      # GINS 配置
│   └── filter_model.h         # 滤波模型
├── ins/                    # 惯导解算模块
│   ├── pins.h
│   └── pins.cpp
├── io/                     # 输入输出模块
│   ├── imu_data_parser.h
│   ├── gnss_data_parser.h
│   ├── nav_result_parser.h
│   ├── nav_result_saver.h/cpp
│   ├── ref_data_parser.h
│   └── sensor_data_provider.hpp
├── measurement/            # 测量数据定义
│   ├── imu.h
│   ├── gnss.h
│   ├── nav_result.h
│   ├── ref.h
│   └── imu_time_aligner.h/cpp
├── state/                  # 状态定义
│   └── state.h
└── utils/                  # 工具与异常
    ├── constants.h
    ├── earth_model.h/cpp
    ├── rotation.h
    ├── logger.h/cpp        # 日志系统
    └── exceptions.h        # 异常类定义
```

## 依赖项

- **C++17** 或更高版本
- **CMake 3.16** 或更高版本
- **Eigen3** - 矩阵运算库
- **Sophus** - 李群/李代数库
- **yaml-cpp** - YAML配置文件解析库
- **spdlog** - 高性能日志库

### 安装依赖

#### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install libeigen3-dev libsophus-dev libyaml-cpp-dev libspdlog-dev
```

#### macOS (使用Homebrew)

```bash
brew install eigen sophus yaml-cpp spdlog
```

## 编译

详细编译说明请参考 [docs/BUILD.md](docs/BUILD.md)

快速开始：

```bash
# 创建构建目录
mkdir build && cd build

# 配置CMake
cmake ..

# 编译
make -j$(nproc)
```

## 使用方法

### 1. 配置系统参数

编辑 `config/config.yaml` 文件，设置以下参数：

- **IMU噪声参数**: ARW, VRW, 零偏稳定性等
- **GNSS噪声参数**: 位置精度
- **初始状态**: 位置、速度、姿态、零偏
- **初始协方差**: 各状态量的初始不确定性
- **天线杆臂**: IMU到GNSS天线的杆臂向量

### 2. 运行INS评估

```bash
./eval_ins <imu_file> <output_file> <config_file>
```

示例：
```bash
./eval_ins ../data/imu.txt ../res.txt ../config/config.yaml
```

### 3. 运行姿态估计器评估

```bash
./estimator_eval <imu_file> <gnss_file> <config_file> <output_file>
```

### 4. 运行姿态评估

```bash
./pose_eval <ref_file> <res_file> <output_file> 
```

## 配置说明

配置文件采用YAML格式，主要包含以下部分：

- `imu`: IMU噪声参数配置
- `gnss`: GNSS测量噪声配置
- `initial_state`: 系统初始状态
- `initial_covariance`: 初始协方差矩阵
- `antlever`: 天线杆臂向量

详细配置示例见 `config/config.yaml`。

## 数据格式

### IMU数据格式

IMU数据文件应包含时间戳和IMU测量值（角速度和加速度）。

### GNSS数据格式

GNSS数据文件应包含时间戳和位置信息。

### 输出格式

导航结果包含位置、速度、姿态等信息。

## 开发计划

计划之后加入一个-初始对准模块，不是根据配置文件，而是传感器的数据实现初始对准结果

## 建模说明

- **导航坐标系**: 北东地(NED)
- **姿态误差**: 定义在导航坐标系下，左扰动模式
- **IMU 坐标系**: 前-右-下(FRD)
- **时间基准**: IMU 时间
- **地球模型**: 考虑地球自转，使用 WGS84 椭球模型

## 注意事项

- 本项目为实验性代码，适用于小范围、低精度场景
- 考虑了地球自转效应
- 不考虑安装误差
- 使用简化的积分方法
- 适用于学习和研究目的，不建议直接用于实际工程

## 许可证

本项目为个人学习研究项目。

## 文档

- [构建说明](docs/BUILD.md) - 编译步骤与依赖安装
- [架构说明](docs/ARCHITECTURE.md) - 模块划分与数据流
- [代码风格](docs/CODING_STYLE.md) - 命名规范与错误处理（含异常类说明）
- [日志系统](docs/LOGGING.md) - 日志级别与使用方式

## 参考

- 牛小骥老师的《惯性导航原理与算法设计》
- 严恭敏老师的[pins](https://www.psins.org.cn/)实现
- 牛小骥老师的[kf-gins](https://github.com/i2Nav-WHU/KF-GINS)实现
