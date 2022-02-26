#include "Logger.hpp"

#include <iomanip>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>

CSMLogger::CSMLogger() {
    std::stringstream ss;
    time_t ti = time(NULL);
    ss << LOG_PATH << std::put_time(std::localtime(&ti), "%b_%d_%Y.txt");
    m_fd = open(ss.str().c_str(), O_RDWR);
    symlink(ss.str().c_str(), (LOG_PATH + "latest.txt").c_str());
}

CSMLogger::~CSMLogger() {
    std::lock_guard<std::mutex> lock(m_mutex);
    close(m_fd);
}

void CSMLogger::registerThread(const std::string& name) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_threadNames[std::this_thread::get_id()] = name;
    std::stringstream ss;
    ss << "Thread ID " << std::this_thread::get_id() << " registered as " << name << '\n';
    write(m_fd, ss.str().c_str(), ss.str().length());
}

void CSMLogger::log(const std::string& message) {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::stringstream ss;
    std::string threadName = "UNNAMED";
    try {
        threadName = m_threadNames.at(std::this_thread::get_id());
    } catch (std::out_of_range) {}
    ss << "[" << threadName << "/INFO] " << message << '\n';
    write(m_fd, message.c_str(), message.length());
}

void CSMLogger::error(const std::string& message) {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::stringstream ss;
    std::string threadName = "UNNAMED";
    try {
        threadName = m_threadNames.at(std::this_thread::get_id());
    } catch (std::out_of_range) {}
    ss << "[" << threadName << "/ERROR] " << message << '\n';
    write(m_fd, message.c_str(), message.length());
}
