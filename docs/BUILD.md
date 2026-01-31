# 构建说明

## 系统要求

- **操作系统**: Linux / macOS / Windows (使用WSL或MinGW)
- **编译器**: GCC 7+ 或 Clang 5+ (支持C++17)
- **CMake**: 3.16 或更高版本

## 依赖安装

### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libsophus-dev \
    libyaml-cpp-dev \
    libspdlog-dev
```

### macOS (使用Homebrew)

```bash
brew install cmake eigen sophus yaml-cpp spdlog
```

### 从源码编译依赖

如果系统包管理器没有提供所需依赖，可以从源码编译：

#### Eigen3

```bash
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build && cd build
cmake ..
make install
```

#### Sophus

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build && cd build
cmake ..
make install
```

#### yaml-cpp

```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake -DYAML_BUILD_SHARED_LIBS=ON ..
make install
```

#### spdlog

```bash
git clone https://github.com/gabime/spdlog.git
cd spdlog
mkdir build && cd build
cmake ..
make install
```

## 编译步骤

### 标准编译

```bash
# 创建构建目录
mkdir build && cd build

# 配置CMake
cmake ..

# 编译
make -j$(nproc)  # Linux
# 或
make -j$(sysctl -n hw.ncpu)  # macOS
```

### 调试模式编译

```bash
mkdir build-debug && cd build-debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)
```

### 发布模式编译（默认）

```bash
mkdir build-release && cd build-release
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

## 输出文件

编译完成后，可执行文件位于 `build/bin/` 目录：

- `eval_ins`: INS评估工具
- `estimator_eval`: 估计器评估工具
- `pose_eval`: 姿态评估工具

库文件位于 `build/lib/` 目录：

- `libgins-lib.a`: 静态库

## 常见问题

### 找不到Eigen3

```bash
# 设置Eigen3路径
cmake -DEigen3_DIR=/path/to/eigen/share/eigen3/cmake ..
```

### 找不到Sophus

```bash
# 设置Sophus路径
cmake -DSophus_DIR=/path/to/sophus/share/sophus/cmake ..
```

### 找不到yaml-cpp

```bash
# 设置yaml-cpp路径
cmake -Dyaml-cpp_DIR=/path/to/yaml-cpp/lib/cmake/yaml-cpp ..
```

### 找不到spdlog

```bash
# 设置spdlog路径
cmake -Dspdlog_DIR=/path/to/spdlog/lib/cmake/spdlog ..
```

### 编译错误：C++17特性不支持

确保编译器版本足够新：
- GCC >= 7
- Clang >= 5
- MSVC >= 2017

## 清理构建

```bash
# 删除构建目录
rm -rf build

# 或只清理编译产物
cd build
make clean
```

## 安装（可选）

如果需要安装到系统：

```bash
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
make install
```

安装后可以在其他项目中使用：

```cmake
find_package(mems_ins_gps REQUIRED)
target_link_libraries(your_target PRIVATE mems_ins_gps::gins-lib)
```
