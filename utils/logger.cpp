#include "logger.h"

#include <vector>
#include <filesystem>

namespace Logger {

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

void Logger::init(Level level, const std::string& logFile) {
    if (initialized_) {
        // 如果已经初始化，只更新级别
        setLevel(level);
        return;
    }

    std::vector<spdlog::sink_ptr> sinks;

    // 控制台输出（带颜色）
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::trace);
    console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%s:%#] %v");
    sinks.push_back(console_sink);

    // 文件输出（如果指定了文件）
    if (!logFile.empty()) {
        try {
            // 确保目录存在
            std::filesystem::path logPath(logFile);
            if (logPath.has_parent_path()) {
                std::filesystem::create_directories(logPath.parent_path());
            }

            // 使用 rotating file sink，每个文件最大 5MB，保留 3 个文件
            auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                logFile, 5 * 1024 * 1024, 3);
            file_sink->set_level(spdlog::level::trace);
            file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v");
            sinks.push_back(file_sink);
        } catch (const std::exception& e) {
            // 如果文件创建失败，只使用控制台输出
            spdlog::warn("Failed to create log file: {}, using console only", logFile);
        }
    }

    // 创建 logger
    logger_ = std::make_shared<spdlog::logger>("mems_ins_gps", sinks.begin(), sinks.end());
    logger_->set_level(convertLevel(level));
    logger_->flush_on(spdlog::level::warn);  // warn 及以上级别立即刷新

    // 注册为默认 logger（可选）
    spdlog::register_logger(logger_);
    spdlog::set_default_logger(logger_);

    initialized_ = true;
}

void Logger::setLevel(Level level) {
    if (logger_) {
        logger_->set_level(convertLevel(level));
    }
}

Level Logger::getLevel() const {
    if (logger_) {
        return convertLevel(logger_->level());
    }
    return Level::NONE;
}

void Logger::log(Level level, const std::string& message, const std::string& file, int line) {
    if (!initialized_ || !logger_) {
        return;
    }

    spdlog::level::level_enum spdlog_level = convertLevel(level);
    
    // 如果消息级别低于当前设置，不输出
    if (spdlog_level < logger_->level()) {
        return;
    }

    // 提取文件名（不含路径）
    std::string fileName = file;
    size_t pos = fileName.find_last_of("/\\");
    if (pos != std::string::npos) {
        fileName = fileName.substr(pos + 1);
    }

    // 使用 spdlog 记录日志
    // spdlog 会自动处理文件名和行号（通过宏），但这里我们手动指定
    logger_->log(spdlog::source_loc{file.c_str(), line, fileName.c_str()}, 
                 spdlog_level, message);
}

void Logger::shutdown() {
    if (logger_) {
        logger_->flush();
        spdlog::drop_all();
    }
    initialized_ = false;
}

spdlog::level::level_enum Logger::convertLevel(Level level) const {
    switch (level) {
        case Level::DEBUG:   return spdlog::level::debug;
        case Level::INFO:    return spdlog::level::info;
        case Level::WARNING: return spdlog::level::warn;
        case Level::ERROR:   return spdlog::level::err;
        case Level::NONE:    return spdlog::level::off;
        default:             return spdlog::level::info;
    }
}

Level Logger::convertLevel(spdlog::level::level_enum level) const {
    switch (level) {
        case spdlog::level::trace:
        case spdlog::level::debug:   return Level::DEBUG;
        case spdlog::level::info:    return Level::INFO;
        case spdlog::level::warn:    return Level::WARNING;
        case spdlog::level::err:
        case spdlog::level::critical: return Level::ERROR;
        case spdlog::level::off:     return Level::NONE;
        default:                      return Level::INFO;
    }
}

Logger::~Logger() {
    shutdown();
}

} // namespace Logger
