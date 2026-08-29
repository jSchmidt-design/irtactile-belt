#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

// Encoder for the 2-channel serial wire format the tactile firmware speaks.
//
// Data frame, 5 bytes per 2-channel sample pair:
//
//   ch1_lo    sample & 0xFF
//   ch1_hi   (sample >> 8) & 0x7F
//   ch2_lo
//   ch2_hi
//   chk       ch1_lo ^ ch1_hi ^ ch2_lo ^ ch2_hi
//
// Every `markerInterval` frames the encoder inserts a sync marker carrying the
// stream description in-band, so the firmware follows a rate change without a
// handshake:
//
//   FF FF FF FF   sync
//   0x01          frame type / version           (<= 0x7F)
//   rateCode      ladder exponent, 0..10         (<= 0x7F)
//   blkLo          samplesPerBlock        & 0x7F
//   blkHi         (samplesPerBlock >> 7)  & 0x7F
//   chk           (type ^ rateCode ^ blkLo ^ blkHi) & 0x7F
//
// Both sample high bytes are masked to 0x7F and every header payload byte is
// <= 0x7F, so data can produce a run of at most 2 consecutive 0xFF. The header
// repeats, so a receiver that resets relocks within one marker interval.
//
// Receiver rule: consume the whole run of 0xFF and read the frame type from the
// first non-0xFF byte. The four bytes after the *first* 0xFF are not the
// payload - the previous frame's checksum is unmasked, so a marker may be
// preceded by one 0xFF and the run reaches 5.
//
// `legacyMarker` emits the bare 4x0xFF marker with no payload, for firmware
// that does not know the header.
class SerialProtocol {
public:
    static constexpr size_t kChannels{2};
    static constexpr size_t kFrameBytes{5};
    static constexpr uint8_t kHeaderFrameType{0x01};
    // Bytes a marker occupies, with and without the header payload.
    static constexpr size_t kHeaderBytes{9};
    static constexpr size_t kLegacyMarkerBytes{4};
    // Largest samplesPerBlock the two 7-bit header fields hold.
    static constexpr uint32_t kMaxBlockSamples{(1u << 14) - 1};

    // `markerInterval` is in frames; 0 is clamped to 1.
    explicit SerialProtocol(size_t markerInterval, bool legacyMarker = false);

    // Sets the stream description the header carries and schedules a header for
    // the next encode(), bypassing the marker counter. Call at startup and on
    // every reconfiguration. Values wider than the wire fields are truncated.
    void setStreamConfig(uint8_t rateCode, uint32_t samplesPerBlock) noexcept;

    // In frames; takes effect at the next encode().
    void setMarkerInterval(size_t markerInterval) noexcept;

    [[nodiscard]] size_t markerInterval() const noexcept { return m_markerInterval; }

    // Bytes one marker costs. Static, so a bandwidth budget can be computed
    // before an encoder exists.
    [[nodiscard]] static constexpr size_t markerBytes(bool legacyMarker) noexcept {
        return legacyMarker ? kLegacyMarkerBytes : kHeaderBytes;
    }
    [[nodiscard]] size_t markerBytes() const noexcept { return markerBytes(m_legacyMarker); }

    // Encodes `numFrames` interleaved frames (kChannels samples per frame) and
    // returns the internal buffer, valid until the next encode(). Emits at most
    // one marker per call, so an interval below the caller's block size caps the
    // cadence at one marker per block.
    [[nodiscard]] const std::vector<uint8_t>& encode(const uint16_t* data, size_t numFrames);

private:
    // Appends a sync marker, with the stream header unless in legacy mode.
    void appendMarker();

    size_t m_markerInterval{1};
    size_t m_messageCounter{0};
    bool m_legacyMarker{false};
    bool m_headerPending{false};
    uint8_t m_rateCode{0};
    uint16_t m_blockSamples{0};
    std::vector<uint8_t> m_buffer{};
};
