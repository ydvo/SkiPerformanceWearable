#pragma once
#include <cstdint>
#include <array>

struct ImuSample {
    uint64_t timestamp_us; 
    std::array<float, 3> acc; 
    std::array<float, 3> gyro; 
    std::array<float, 3> mag; 
};

struct Attitude {
    uint64_t timestamp_us; 
    std::array<float, 4> quaternion; 
    std::array<float, 3> gyro_bias; 
};

class IFusion {
public: 
    virtual ~IFusion() = default; 
    virtual void reset(const Attitude *init = nullptr) = 0;
    virtual Attitude update(const ImuSample& sample) = 0; 
};