#ifndef MEMS_INS_GPS_EXCEPTIONS_H
#define MEMS_INS_GPS_EXCEPTIONS_H

#include <stdexcept>
#include <string>

namespace mems_ins_gps {

/**
 * @brief 基础异常类
 */
class BaseException : public std::runtime_error {
public:
    explicit BaseException(const std::string& message)
        : std::runtime_error(message) {}
};

/**
 * @brief 文件相关异常
 */
class FileException : public BaseException {
public:
    explicit FileException(const std::string& message)
        : BaseException("File error: " + message) {}
    
    explicit FileException(const std::string& operation, const std::string& path)
        : BaseException("File error [" + operation + "]: " + path) {}
};

/**
 * @brief 配置相关异常
 */
class ConfigException : public BaseException {
public:
    explicit ConfigException(const std::string& message)
        : BaseException("Config error: " + message) {}
    
    explicit ConfigException(const std::string& key, const std::string& reason)
        : BaseException("Config error [" + key + "]: " + reason) {}
};

/**
 * @brief 数据相关异常
 */
class DataException : public BaseException {
public:
    explicit DataException(const std::string& message)
        : BaseException("Data error: " + message) {}
};

/**
 * @brief 参数验证异常
 */
class ValidationException : public BaseException {
public:
    explicit ValidationException(const std::string& message)
        : BaseException("Validation error: " + message) {}
    
    explicit ValidationException(const std::string& parameter, const std::string& reason)
        : BaseException("Validation error [" + parameter + "]: " + reason) {}
};

/**
 * @brief 时间相关异常
 */
class TimeException : public BaseException {
public:
    explicit TimeException(const std::string& message)
        : BaseException("Time error: " + message) {}
};

} // namespace mems_ins_gps

#endif //MEMS_INS_GPS_EXCEPTIONS_H
