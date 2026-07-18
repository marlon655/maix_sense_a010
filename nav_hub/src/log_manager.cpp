#include "nav_hub/log_manager.hpp"
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>

void LogManager::gera_log(const std::string& msg, LogLevel level) {
    std::string current_date = get_current_datetime("%d_%m_%Y");
    std::string log_file_path = "/home/ubuntu/Desktop/logs/info/Log_Info_" + current_date + ".log";
    std::string error_file_path = "/home/ubuntu/Desktop/logs/error/Log_Error_" + current_date + ".log";

    std::string timestamp = get_current_datetime("%d-%m-%Y %H:%M:%S");
    std::string log_level = log_level_to_string(level);
    std::string log_message = timestamp + ";" + log_level + ";" + msg;

    if (level == Info || level == Debug) {
        write_to_file(log_file_path, log_message);
    }

    if (level == Fatal || level == Error || level == Warn) {
        write_to_file(error_file_path, log_message);
    }
}

std::string LogManager::get_current_datetime(const std::string& format) {
    // get current time in seconds since Epoch (1970)
    auto current_time = std::time(nullptr);
    // convert the epoch time to a readable structure (year, day, hour, etc.)
    auto local_time = *std::localtime(&current_time);
    // format the time into a string based on the provided format (e.g., "%d %m %Y")
    std::ostringstream formatted_datetime;
    formatted_datetime << std::put_time(&local_time, format.c_str());

    //convert date to str to return
    return formatted_datetime.str();
}

void LogManager::write_to_file(const std::string& file_path, const std::string& log_message) const {
    try {
        // create the directories if they do not exist
        std::filesystem::path path(file_path);
        std::filesystem::create_directories(path.parent_path());

        std::ofstream log_file(file_path, std::ios::app);
        if(!log_file.is_open()) {
            throw std::ios_base::failure("Failed to open log file: " + file_path);
        }

        log_file << log_message << "\n";
        log_file.close();
    } catch (const std::exception& e) {
        std::cerr << "Error writing to log file: " << e.what() << std::endl;
    }
}

std::string LogManager::log_level_to_string(LogLevel level) const {
    switch(level) {
        case Fatal: return "FATAL";
        case Error: return "ERROR";
        case Warn: return "WARN";
        case Info: return "INFO";
        case Debug: return "DEBUG";
        default: return "UNKNOWN";
    }
}
