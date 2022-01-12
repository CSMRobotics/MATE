#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <string>
#include <mutex>
#include <thread>
#include <unordered_map>

// threadsafe singleton logger
class CSMLogger {
public:
    CSMLogger();
    ~CSMLogger();
    void registerThread(const std::string& name);
    void log(const std::string& message);
    void error(const std::string& message);
private:
    static inline std::unordered_map<std::thread::id, std::string> m_threadNames = {};
    static inline int m_fd;
    static inline std::mutex m_mutex;
    static inline const std::string LOG_PATH = "./Logs/";
};

#endif // LOGGER_HPP