# 日志系统使用说明

## 概述

项目使用基于 **spdlog** 的统一日志系统来替代 `std::cout` 和 `std::cerr`，提供高性能的日志管理和控制。spdlog 是一个快速的 C++ 日志库，支持多线程、异步日志、日志轮转等功能。

## 日志级别

日志系统支持以下级别（从低到高）：

- **DEBUG**: 调试信息，用于开发调试
- **INFO**: 一般信息，程序运行状态
- **WARNING**: 警告信息，不影响程序运行但需要注意
- **ERROR**: 错误信息，程序可能无法正常工作
- **NONE**: 禁用所有日志输出

## 初始化

在使用日志系统之前，需要先初始化：

```cpp
#include "utils/logger.h"

// 初始化日志系统，只输出到控制台
Logger::Logger::getInstance().init(Logger::Level::INFO);

// 初始化日志系统，同时输出到文件
Logger::Logger::getInstance().init(Logger::Level::INFO, "log.txt");

// 设置日志级别
Logger::Logger::getInstance().setLevel(Logger::Level::DEBUG);
```

（注：`Logger` 为命名空间名，同时也是类名，故 API 为 `Logger::Logger::getInstance()`。）

## 使用方法

### 简单日志宏

对于简单的字符串消息，使用以下宏：

```cpp
LOG_DEBUG("这是一条调试信息");
LOG_INFO("程序开始运行");
LOG_WARNING("检测到异常情况");
LOG_ERROR("发生错误");
```

### 流式日志宏

对于需要格式化输出的复杂消息，使用流式宏：

```cpp
LOG_INFO_STREAM() << "IMU samples: " << count;
LOG_DEBUG_STREAM() << "Position: " << pos.transpose();
LOG_ERROR_STREAM() << "Failed to load file: " << filename;
```

### 示例

```cpp
#include "utils/logger.h"

int main() {
    // 初始化日志系统
    Logger::Logger::getInstance().init(Logger::Level::INFO);
    
    LOG_INFO("程序启动");
    LOG_INFO_STREAM() << "加载了 " << dataCount << " 条数据";
    
    if (errorOccurred) {
        LOG_ERROR("数据处理失败");
        return -1;
    }
    
    LOG_INFO("程序正常结束");
    return 0;
}
```

## 日志输出格式

日志输出格式如下：

```
[2026-01-28 10:30:45.123456] [INFO] [ins-eval.cpp:48] IMU samples: 1000
```

格式说明：
- `[时间戳]`: 精确到微秒的时间戳
- `[级别]`: 日志级别（控制台输出带颜色）
- `[文件名:行号]`: 代码位置（仅在使用宏时显示）
- `消息内容`: 实际的日志消息

**注意**: 控制台输出使用彩色格式，文件输出使用纯文本格式。

## 日志级别控制

可以通过设置日志级别来控制输出：

```cpp
// 只显示 WARNING 和 ERROR
Logger::Logger::getInstance().setLevel(Logger::Level::WARNING);

// 显示所有日志（包括 DEBUG）
Logger::Logger::getInstance().setLevel(Logger::Level::DEBUG);

// 禁用所有日志
Logger::Logger::getInstance().setLevel(Logger::Level::NONE);
```

## 文件输出

日志可以同时输出到控制台和文件：

```cpp
// 输出到控制台和文件
Logger::Logger::getInstance().init(Logger::Level::INFO, "app.log");
```

**日志轮转**: 文件日志支持自动轮转，每个文件最大 5MB，保留最近 3 个文件。当日志文件达到大小限制时，会自动创建新文件并重命名旧文件。

## 注意事项

1. **初始化**: 在使用日志宏之前，必须先初始化日志系统
2. **线程安全**: 日志系统基于 spdlog，完全线程安全，可以在多线程环境中使用
3. **性能**: 
   - spdlog 是高性能日志库，对性能影响很小
   - DEBUG 级别的日志在生产环境中应该禁用以提高性能
   - WARNING 及以上级别的日志会立即刷新到磁盘
4. **格式化输出**: `printStatistics()` 等专门用于用户可见报告的函数，可以继续使用 `std::cout` 进行格式化输出
5. **依赖**: 项目依赖 spdlog 库，确保已正确安装（见 [构建说明](BUILD.md)）

## 迁移指南

将现有代码中的 `std::cout` 替换为日志调用：

```cpp
// 旧代码
std::cout << "Error: " << message << std::endl;

// 新代码
LOG_ERROR_STREAM() << "Error: " << message;
```

```cpp
// 旧代码
std::cout << "[INFO] Processing..." << std::endl;

// 新代码
LOG_INFO("Processing...");
```
