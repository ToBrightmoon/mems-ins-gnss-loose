#ifndef MEMS_INS_GPS_SENSOR_DATA_PROVIDER_HPP
#define MEMS_INS_GPS_SENSOR_DATA_PROVIDER_HPP

#include <fstream>
#include <queue>
#include <string>
#include <functional>
#include <stdexcept>
#include "utils/exceptions.h"

template<typename DataType>
class SensorDataProvider
{
public:
    using ParserFunc = std::function<bool(const std::string &, DataType &)>;

    SensorDataProvider(const std::string &path, ParserFunc parser)
        : file_(path), parser_(std::move(parser))
    {
        if (!file_.is_open())
        {
            throw mems_ins_gps::FileException("open", path);
        }
    }

    ~SensorDataProvider() = default;

    bool load()
    {
        if (loaded_) {
            return true;  // 已经加载过
        }

        std::string line;
        std::queue<DataType> tmp;
        size_t lineNumber = 0;

        while (std::getline(file_, line))
        {
            lineNumber++;
            if (line.empty())
                continue;

            DataType data{};
            if (!parser_(line, data))
            {
                throw mems_ins_gps::DataException(
                    "Failed to parse line " + std::to_string(lineNumber) + 
                    " in file: " + (file_.is_open() ? "unknown" : "closed"));
            }
            tmp.push(data);
        }

        if (tmp.empty()) {
            throw mems_ins_gps::DataException("No valid data found in file");
        }

        buffer_ = std::move(tmp);
        loaded_ = true;
        return true;
    }

    DataType next()
    {
        if (!loaded_)
            throw mems_ins_gps::DataException("Data not loaded. Call load() first.");

        if (buffer_.empty())
            throw mems_ins_gps::DataException("No more data available");

        DataType data = buffer_.front();
        buffer_.pop();
        return data;
    }

    size_t size() const
    {
        return buffer_.size();
    }

    bool empty() const
    {
        return buffer_.empty();
    }

private:
    std::ifstream file_;
    ParserFunc parser_;
    std::queue<DataType> buffer_;
    bool loaded_{false};
};

#endif //MEMS_INS_GPS_SENSOR_DATA_PROVIDER_HPP
