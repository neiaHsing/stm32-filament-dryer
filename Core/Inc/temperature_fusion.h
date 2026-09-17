#ifndef TEMPERATURE_FUSION_H
#define TEMPERATURE_FUSION_H
#include <cstdint>

// Inputs have already passed sensor range/validity checks. Units: 0.01 C.
// Round nearest, with exact half steps away from zero.
constexpr int32_t FuseTemperature(int32_t aht, int32_t bmp)
{
    const int64_t weighted = 7LL * aht + 3LL * bmp;
    return static_cast<int32_t>((weighted + (weighted >= 0 ? 5 : -5)) / 10);
}
#endif
