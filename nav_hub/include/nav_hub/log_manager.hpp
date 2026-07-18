#ifndef LOG_MANAGER_HPP
#define LOG_MANAGER_HPP

#include <string>

class LogManager {
public:
    enum LogLevel {
        Fatal,
        Error,
        Warn,
        Info,
        Debug
    };

    void gera_log(const std::string& msg, LogLevel level);

private:
    std::string get_current_datetime(const std::string& format);
    void write_to_file(const std::string& file_path, const std::string& log_message) const;
    std::string log_level_to_string(LogLevel level) const;
};

#endif 