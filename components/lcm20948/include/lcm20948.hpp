#pragma once
#include <array>

struct AccelData {
    float x, y, z;
};

class LCM20948 {
public:
    LCM20948();
    void initialize();
    AccelData getAcceleration();
};
