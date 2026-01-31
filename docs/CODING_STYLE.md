# 代码风格指南

本文档定义了项目的代码风格规范，旨在保持代码的一致性和可读性。

## 命名规范

### 文件命名

- **头文件**: 使用小写字母和下划线，如 `pose_estimator.h`
- **源文件**: 与对应头文件同名，如 `pose_estimator.cpp`
- **模板头文件**: 使用 `.hpp` 扩展名，如 `sensor_data_provider.hpp`

### 类命名

- 使用 **PascalCase**（大驼峰），如 `PoseEstimator`, `GinsConfig`

### 函数和变量命名

- 使用 **camelCase**（小驼峰），如 `updatePose()`, `getPose()`
- 私有成员变量使用下划线后缀，如 `state_`, `isInit_`

### 常量命名

- 使用 **UPPER_SNAKE_CASE**（全大写下划线），如 `Deg2Rad`, `TIME_ALIGN_ERR`

### 命名空间

- 项目不使用命名空间，所有符号使用项目前缀 `MEMS_INS_GPS_` 在头文件保护宏中

## 头文件保护

所有头文件必须包含头文件保护宏：

```cpp
#ifndef MEMS_INS_GPS_MODULE_NAME_H
#define MEMS_INS_GPS_MODULE_NAME_H
// ... 文件内容 ...
#endif //MEMS_INS_GPS_MODULE_NAME_H
```

- `.h` 文件使用 `_H` 后缀
- `.hpp` 文件使用 `_HPP` 后缀
- 宏名称格式：`MEMS_INS_GPS_<MODULE>_<NAME>_<EXT>`

## 代码格式

### 缩进

- 使用 **4个空格** 进行缩进（不使用Tab）

### 大括号风格

- 使用 **Allman风格**（大括号独立成行）：

```cpp
if (condition)
{
    // code
}
else
{
    // code
}
```

### 空格

- 运算符前后加空格：`a + b`, `x = y`
- 函数调用括号前不加空格：`function()`
- 控制语句括号前后加空格：`if (condition)`
- 逗号后加空格：`a, b, c`

### 指针和引用

- 指针和引用符号靠近类型：`Type* ptr`, `Type& ref`

## 类设计

### 构造函数和析构函数

- 使用 `explicit` 标记单参数构造函数
- 使用 `= default` 或 `= delete` 明确表达意图
- 提供移动构造和移动赋值（如需要）

示例：

```cpp
class MyClass
{
public:
    explicit MyClass(const Config& config);
    ~MyClass() = default;
    
    MyClass(const MyClass&) = default;
    MyClass& operator=(const MyClass&) = delete;
    
    MyClass(MyClass&&) = default;
    MyClass& operator=(MyClass&&) = delete;
};
```

### 成员函数

- 使用 `[[nodiscard]]` 标记不应忽略返回值的函数
- 使用 `const` 标记不修改对象状态的成员函数

```cpp
[[nodiscard]] NominalState getPose() const;
```

## 注释

### 文件头注释（可选）

```cpp
/**
 * @file module_name.h
 * @brief Brief description of the module
 */
```

### 函数注释

对于公共接口，建议添加注释：

```cpp
/**
 * @brief Update the pose estimation
 * @param imuData IMU measurement data
 */
void updatePose(const ImuData& imuData);
```

### 行内注释

- 使用 `//` 进行单行注释
- 复杂逻辑必须添加注释说明

## 包含顺序

头文件包含顺序：

1. 对应的头文件（.cpp文件中）
2. C/C++标准库
3. 第三方库（Eigen, Sophus等）
4. 项目内部头文件

示例：

```cpp
#include "pose_estimator.h"  // 1. 对应头文件

#include <iostream>           // 2. 标准库
#include <vector>

#include <Eigen/Core>         // 3. 第三方库
#include <Sophus/so3.hpp>

#include "state/state.h"      // 4. 项目内部
#include "filter/gins_config.h"
```

## 错误处理

- 使用异常处理错误情况，优先使用项目自定义异常类（`utils/exceptions.h`）：
  - **ConfigException**: 配置解析或键值错误
  - **FileException**: 文件打开、读取、写入失败
  - **DataException**: 数据格式或内容错误
  - **ValidationException**: 参数范围或合法性校验失败
  - **TimeException**: 时间对齐或时间戳错误
- 在构造函数和入口函数中验证参数有效性，非法时抛出 `ValidationException`
- 仅在无法使用上述分类时使用 `std::runtime_error` 或 `BaseException`

## 现代C++特性

项目使用 **C++17**，鼓励使用：

- `auto` 关键字（适度使用）
- 范围for循环：`for (const auto& item : container)`
- `nullptr` 而非 `NULL`
- 智能指针（如需要）
- `constexpr` 和 `const` 尽可能使用

## 示例

完整的类定义示例：

```cpp
#ifndef MEMS_INS_GPS_EXAMPLE_H
#define MEMS_INS_GPS_EXAMPLE_H

#include <Eigen/Core>
#include "state/state.h"

class Example
{
public:
    explicit Example(const Config& config);
    ~Example() = default;

    Example(const Example&) = default;
    Example& operator=(const Example&) = delete;

    Example(Example&&) = default;
    Example& operator=(Example&&) = delete;

    void processData(const Data& data);
    [[nodiscard]] Result getResult() const;

private:
    void helperFunction();

    State state_;
    bool isInitialized_ = false;
};

#endif //MEMS_INS_GPS_EXAMPLE_H
```
