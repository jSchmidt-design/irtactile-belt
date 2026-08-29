#include "WireParams.h"

#include "SerialProtocol.h"

#include <algorithm>

bool rateCodeFor(uint32_t rateMilliHz, uint8_t& out) noexcept
{
    for (uint8_t code{0}; code <= kMaxRateCode; ++code) {
        if ((kLadderTopMilliHz >> code) == rateMilliHz) { out = code; return true; }
    }
    return false;
}

size_t markerIntervalFor(size_t explicitInterval, double rateHz) noexcept
{
    if (explicitInterval != 0) return explicitInterval;
    const double frames{rateHz / kMarkerCadenceHz};
    return frames < 1.0 ? size_t{1} : static_cast<size_t>(frames);
}

uint32_t backlogThresholdBytes(uint32_t samplesPerBlock, int baud,
                               uint32_t txQueueBytes) noexcept
{
    const uint32_t block{samplesPerBlock * static_cast<uint32_t>(SerialProtocol::kFrameBytes)};
    const uint32_t usbFrame{static_cast<uint32_t>(baud / 10 / 1000)};   // bytes in ~1 ms
    // max(1) keeps lo <= hi for the clamp below: txQueueBytes is a parameter
    // now, so a caller could hand in a queue too small to have a 3/4 point.
    const uint32_t ceiling{std::max(1u, txQueueBytes * 3 / 4)};
    return std::clamp(std::max(block, usbFrame), 1u, ceiling);
}
