#ifndef MEMS_INS_GPS_LOGGER_H
#define MEMS_INS_GPS_LOGGER_H

#include <string>
#include <memory>
#include <sstream>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>

namespace Logger {

enum class Level {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
    NONE = 4  // 禁用所有日志
};

class Logger {
public:
    static Logger& getInstance();

    /**
     * @brief 初始化日志系统
     * @param level 最小日志级别
     * @param logFile 日志文件路径，空字符串表示只输出到控制台
     */
    void init(Level level = Level::INFO, const std::string& logFile = "");

    /**
     * @brief 设置日志级别
     */
    void setLevel(Level level);

    /**
     * @brief 获取当前日志级别
     */
    [[nodiscard]] Level getLevel() const;

    /**
     * @brief 记录日志
     */
    void log(Level level, const std::string& message, const std::string& file = "", int line = 0);

    /**
     * @brief 关闭日志系统
     */
    void shutdown();

private:
    Logger() = default;
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    spdlog::level::level_enum convertLevel(Level level) const;
    Level convertLevel(spdlog::level::level_enum level) const;
    std::shared_ptr<spdlog::logger> logger_;
    bool initialized_ = false;
};

// 便捷宏定义
#define LOG_DEBUG(msg) Logger::Logger::getInstance().log(Logger::Level::DEBUG, msg, __FILE__, __LINE__)
#define LOG_INFO(msg) Logger::Logger::getInstance().log(Logger::Level::INFO, msg, __FILE__, __LINE__)
#define LOG_WARNING(msg) Logger::Logger::getInstance().log(Logger::Level::WARNING, msg, __FILE__, __LINE__)
#define LOG_ERROR(msg) Logger::Logger::getInstance().log(Logger::Level::ERROR, msg, __FILE__, __LINE__)

} // namespace Logger

// 流式日志宏（用于复杂输出，定义在命名空间外以便全局使用）
class LogStream {
public:
    LogStream(Logger::Level level, const std::string& file, int line)
        : level_(level), file_(file), line_(line) {}

    template<typename T>
    LogStream& operator<<(const T& value) {
        stream_ << value;
        return *this;
    }

    ~LogStream() {
        if (!stream_.str().empty()) {
            Logger::Logger::getInstance().log(level_, stream_.str(), file_, line_);
        }
    }

private:
    Logger::Level level_;
    std::string file_;
    int line_;
    std::ostringstream stream_;
};

#define LOG_DEBUG_STREAM() LogStream(Logger::Level::DEBUG, __FILE__, __LINE__)
#define LOG_INFO_STREAM() LogStream(Logger::Level::INFO, __FILE__, __LINE__)
#define LOG_WARNING_STREAM() LogStream(Logger::Level::WARNING, __FILE__, __LINE__)
#define LOG_ERROR_STREAM() LogStream(Logger::Level::ERROR, __FILE__, __LINE__)

#endif //MEMS_INS_GPS_LOGGER_H
