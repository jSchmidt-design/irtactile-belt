#include "SerialProtocol.h"

SerialProtocol::SerialProtocol(size_t markerInterval, bool legacyMarker):
    m_markerInterval{markerInterval == 0 ? size_t{1} : markerInterval},
    m_legacyMarker{legacyMarker}
{
}

void SerialProtocol::setStreamConfig(uint8_t rateCode, uint32_t samplesPerBlock) noexcept
{
    m_rateCode = static_cast<uint8_t>(rateCode & 0x7F);
    m_blockSamples = static_cast<uint16_t>(
        (samplesPerBlock > kMaxBlockSamples ? kMaxBlockSamples : samplesPerBlock));
    m_headerPending = true;
}

void SerialProtocol::setMarkerInterval(size_t markerInterval) noexcept
{
    m_markerInterval = markerInterval == 0 ? size_t{1} : markerInterval;
}

void SerialProtocol::appendMarker()
{
    m_buffer.push_back(0xFF);
    m_buffer.push_back(0xFF);
    m_buffer.push_back(0xFF);
    m_buffer.push_back(0xFF);

    if (m_legacyMarker) return;

    const uint8_t blkLo{static_cast<uint8_t>(m_blockSamples & 0x7F)};
    const uint8_t blkHi{static_cast<uint8_t>((m_blockSamples >> 7) & 0x7F)};
    // Masked to keep every payload byte <= 0x7F.
    const uint8_t chk{static_cast<uint8_t>((kHeaderFrameType ^ m_rateCode ^ blkLo ^ blkHi) & 0x7F)};

    m_buffer.push_back(kHeaderFrameType);
    m_buffer.push_back(m_rateCode);
    m_buffer.push_back(blkLo);
    m_buffer.push_back(blkHi);
    m_buffer.push_back(chk);
}

const std::vector<uint8_t>& SerialProtocol::encode(const uint16_t* data, size_t numFrames)
{
    m_buffer.clear();

    // A scheduled header (startup, reconfiguration) pre-empts the counter and
    // restarts the cadence, so a reconfiguration landing near a rollover does
    // not emit two markers back to back. A due marker keeps the remainder, which
    // holds the average cadence exact. Modulo rather than subtraction: a block
    // carrying several intervals' worth of frames must not leave the counter
    // permanently past the interval.
    const bool due{m_messageCounter >= m_markerInterval};
    if (due || m_headerPending) {
        appendMarker();
        m_messageCounter = due ? m_messageCounter % m_markerInterval : 0;
        m_headerPending = false;
    }

    const size_t numSamples{numFrames * kChannels};
    for (size_t i{0}; i < numSamples; i += kChannels) {
        const uint8_t ch1_low{static_cast<uint8_t>(data[i] & 0xFF)};
        const uint8_t ch1_high{static_cast<uint8_t>((data[i] >> 8) & 0x7F)};
        const uint8_t ch2_low{static_cast<uint8_t>(data[i + 1] & 0xFF)};
        const uint8_t ch2_high{static_cast<uint8_t>((data[i + 1] >> 8) & 0x7F)};

        m_buffer.push_back(ch1_low);
        m_buffer.push_back(ch1_high);
        m_buffer.push_back(ch2_low);
        m_buffer.push_back(ch2_high);
        m_buffer.push_back(ch1_low ^ ch1_high ^ ch2_low ^ ch2_high);

        m_messageCounter++;
    }

    return m_buffer;
}
