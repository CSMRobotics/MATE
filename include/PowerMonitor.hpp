#ifndef POWER_MONITOR_HPP
#define POWER_MONITOR_HPP

#include "Component.hpp"
#include "BMR4800116_005.hpp"

#include <chrono>

class PowerMonitor : public Component {
public:
    PowerMonitor();

    void Update();
    void AutoUpdate();
    void Stop();

    PowerStats getPowerStats() {return m_powerStats;};
private:
    BMR4800116_005 m_BMR480;
    PowerStats m_powerStats;
    std::chrono::time_point<std::chrono::system_clock> m_lastUpdated;
};

#endif // POWER_MONITOR_HPP