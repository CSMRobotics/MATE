#ifndef BAR02_HPP
#define BAR02_HPP

#include <cstdint>
#include <chrono>

#include "Component.hpp"

namespace csmutil {
static const float Pa = 100.0f;
static const float bar = 0.001f;
static const float mbar = 1.0f;

// #define BAR02_ADDRESS 

class MS5837 : public Component {
public:
    MS5837();

    void Update();
    void AutoUpdate();
    void Stop();

    void setFluidDensity(float density) {m_density = density;};

    float pressure(float conversion=1.0f);
    float temperature();
    float depth();
    float altitude();
private:
    uint16_t C[8];
    uint32_t D1, D2;
    int32_t TEMP;
    int32_t P;
    uint8_t model;

    float m_density;

    void calculate();

    std::chrono::time_point<std::chrono::system_clock> m_lastUpdated;

    uint8_t crc4(uint16_t n_prom[]);
};

};

#endif //BAR02_HPP