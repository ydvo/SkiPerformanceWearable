#pragma once
#include <array>

struct AccelData {
    float x, y, z;
};

class ICM20948 {
public:
    ICM20948();
    void initialize();
    AccelData getAcceleration();
};
