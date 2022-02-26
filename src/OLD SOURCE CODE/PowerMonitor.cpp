#include "PowerMonitor.hpp"

PowerMonitor::PowerMonitor() {
    m_lastUpdated = std::chrono::high_resolution_clock::now();
    m_BMR480.begin();
}

void PowerMonitor::Update() {
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> timeSinceLast = now - m_lastUpdated;
    if(timeSinceLast.count() >= 100) {
        m_BMR480.getPowerStats(&m_powerStats);
        std::cout << m_powerStats << std::endl;
        // TODO: send these stats to driver station
    }
}

void PowerMonitor::AutoUpdate() {

}

void PowerMonitor::Stop() {

}