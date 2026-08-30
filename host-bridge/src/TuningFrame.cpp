#include "TuningFrame.h"

#include <cmath>

namespace tuning {

uint8_t gammaToWire(float gamma) noexcept
{
    const float q{gamma * 16.0f};
    // `!(q > min)` rather than `q <= min` so a NaN routes to the minimum too -
    // the same trick WireParams.h::toWireSample uses to keep NaN out of a
    // float->int conversion.
    if (!(q > static_cast<float>(kMinGammaQ))) return kMinGammaQ;
    if (q >= static_cast<float>(kMaxGammaQ))   return kMaxGammaQ;
    return static_cast<uint8_t>(std::lround(q));
}

std::array<uint8_t, kFrameBytes> encode(uint16_t fullScaleCounts, uint8_t gammaQ) noexcept
{
    uint16_t fs{fullScaleCounts};
    if (fs < kMinFullScaleCounts) fs = kMinFullScaleCounts;
    if (fs > kMaxFullScaleCounts) fs = kMaxFullScaleCounts;

    uint8_t gq{gammaQ};
    if (gq < kMinGammaQ) gq = kMinGammaQ;
    if (gq > kMaxGammaQ) gq = kMaxGammaQ;

    // 2-count wire units, rounded: (n + 1) / 2. fs <= 20000 so half <= 10000,
    // well inside the 14-bit (0..16383) field.
    const uint16_t half{static_cast<uint16_t>((fs + 1) / 2)};
    const uint8_t fsLo{static_cast<uint8_t>(half & 0x7F)};
    const uint8_t fsHi{static_cast<uint8_t>((half >> 7) & 0x7F)};
    const uint8_t chk{static_cast<uint8_t>((kFrameType ^ fsLo ^ fsHi ^ gq) & 0x7F)};

    return {0xFF, 0xFF, 0xFF, 0xFF, kFrameType, fsLo, fsHi, gq, chk};
}

} // namespace tuning
