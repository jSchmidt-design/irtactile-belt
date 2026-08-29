#include <doctest/doctest.h>

#include "SerialProtocol.h"

#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

// Pins the 2-channel serial wire format of SerialProtocol byte-for-byte:
//   per frame: ch1_lo, ch1_hi & 0x7F, ch2_lo, ch2_hi & 0x7F, XOR checksum
//   every markerInterval frames: 4 x 0xFF sync, followed by the 5-byte rate
//   header (see controller-firmware/protocol.md) unless --legacy-marker selected the bare
//   marker.

namespace {

// Longest run of consecutive 0xFF anywhere in `bytes`.
size_t longestFfRun(const std::vector<uint8_t>& bytes)
{
    size_t best{0};
    size_t run{0};
    for (const uint8_t b : bytes) {
        run = (b == 0xFF) ? run + 1 : 0;
        if (run > best) best = run;
    }
    return best;
}

} // namespace

TEST_CASE("SerialProtocol encodes payload bytes and XOR checksum")
{
    SerialProtocol protocol{500};

    const std::vector<uint16_t> data{0x1234, 0x5678};
    const auto& out = protocol.encode(data.data(), 1);

    REQUIRE(out.size() == 5);
    CHECK(out[0] == 0x34); // ch1 low
    CHECK(out[1] == 0x12); // ch1 high
    CHECK(out[2] == 0x78); // ch2 low
    CHECK(out[3] == 0x56); // ch2 high
    CHECK(out[4] == static_cast<uint8_t>(0x34 ^ 0x12 ^ 0x78 ^ 0x56)); // 0x08
}

TEST_CASE("SerialProtocol masks sample high bytes to 15 bits")
{
    SerialProtocol protocol{500};

    // High bits set beyond bit 14 must not leak into the wire format.
    const std::vector<uint16_t> data{0xFFFF, 0x8000};
    const auto& out = protocol.encode(data.data(), 1);

    REQUIRE(out.size() == 5);
    CHECK(out[0] == 0xFF);
    CHECK(out[1] == 0x7F); // (0xFFFF >> 8) & 0x7F
    CHECK(out[2] == 0x00);
    CHECK(out[3] == 0x00); // (0x8000 >> 8) & 0x7F
    CHECK(out[4] == static_cast<uint8_t>(0xFF ^ 0x7F ^ 0x00 ^ 0x00)); // 0x80
}

TEST_CASE("SerialProtocol inserts the rate header at the configured interval")
{
    SerialProtocol protocol{2};

    const std::vector<uint16_t> frame{0x0001, 0x0002};

    // Message 1: counter 0 -> 1, no marker.
    CHECK(protocol.encode(frame.data(), 1).size() == 5);
    // Message 2: counter 1 -> 2, no marker yet (marker checked at encode start).
    CHECK(protocol.encode(frame.data(), 1).size() == 5);

    // Message 3: counter 2 >= interval -> 9-byte header first, then payload.
    const auto& out = protocol.encode(frame.data(), 1);
    REQUIRE(out.size() == 14);
    CHECK(out[0] == 0xFF);
    CHECK(out[1] == 0xFF);
    CHECK(out[2] == 0xFF);
    CHECK(out[3] == 0xFF);
    CHECK(out[4] == SerialProtocol::kHeaderFrameType);
    CHECK(out[9] == 0x01); // payload follows the header
}

TEST_CASE("SerialProtocol header carries rate code and block size with a checksum")
{
    SerialProtocol protocol{500};
    // 3000 Hz is ladder exponent 4; 300 samples straddles the 7-bit split
    // (300 = 0b10_0101100 -> lo 0x2C, hi 0x02).
    protocol.setStreamConfig(4, 300);

    const auto& out = protocol.encode(nullptr, 0);
    REQUIRE(out.size() == SerialProtocol::kHeaderBytes);
    CHECK(out[4] == SerialProtocol::kHeaderFrameType);
    CHECK(out[5] == 4);
    CHECK(out[6] == 0x2C);
    CHECK(out[7] == 0x02);
    CHECK(out[8] == static_cast<uint8_t>((SerialProtocol::kHeaderFrameType ^ 4 ^ 0x2C ^ 0x02) & 0x7F));

    // Every payload byte stays <= 0x7F, so none can extend or re-trigger a
    // marker run.
    for (size_t i{4}; i < out.size(); ++i)
        CHECK(out[i] <= 0x7F);
}

TEST_CASE("SerialProtocol header payload stays 7-bit across the whole ladder")
{
    for (uint8_t code{0}; code <= 10; ++code) {
        SerialProtocol protocol{500};
        protocol.setStreamConfig(code, SerialProtocol::kMaxBlockSamples);

        const auto& out = protocol.encode(nullptr, 0);
        REQUIRE(out.size() == SerialProtocol::kHeaderBytes);
        for (size_t i{4}; i < out.size(); ++i)
            CHECK(out[i] <= 0x7F);
        CHECK(out[5] == code);
        CHECK(out[6] == 0x7F);
        CHECK(out[7] == 0x7F);
    }
}

TEST_CASE("SerialProtocol clamps an oversized block count to the wire field")
{
    SerialProtocol protocol{500};
    protocol.setStreamConfig(7, SerialProtocol::kMaxBlockSamples + 1000);

    const auto& out = protocol.encode(nullptr, 0);
    REQUIRE(out.size() == SerialProtocol::kHeaderBytes);
    CHECK(out[6] == 0x7F);
    CHECK(out[7] == 0x7F);
}

TEST_CASE("SerialProtocol schedules a header immediately on reconfiguration")
{
    SerialProtocol protocol{500};
    const std::vector<uint16_t> frame{0x0001, 0x0002};

    protocol.setStreamConfig(7, 1);
    CHECK(protocol.encode(frame.data(), 1).size() == 14); // header + payload
    CHECK(protocol.encode(frame.data(), 1).size() == 5);  // counter nowhere near 500

    // A reconfiguration must not wait for the rollover: the receiver has to
    // learn the new rate now, not up to a marker interval later.
    protocol.setStreamConfig(3, 16);
    const auto& out = protocol.encode(frame.data(), 1);
    REQUIRE(out.size() == 14);
    CHECK(out[5] == 3);
    CHECK(out[6] == 16);
    CHECK(out[7] == 0);
}

TEST_CASE("SerialProtocol legacy mode emits the bare 4x0xFF marker")
{
    SerialProtocol protocol{2, /*legacyMarker=*/true};
    CHECK(protocol.markerBytes() == SerialProtocol::kLegacyMarkerBytes);
    // The bandwidth budget asks the mode, not the object, before one exists.
    CHECK(SerialProtocol::markerBytes(true) == SerialProtocol::kLegacyMarkerBytes);
    CHECK(SerialProtocol::markerBytes(false) == SerialProtocol::kHeaderBytes);

    // setStreamConfig still schedules a marker; it just carries no payload.
    protocol.setStreamConfig(7, 1);

    const std::vector<uint16_t> frame{0x0001, 0x0002};
    const auto& out = protocol.encode(frame.data(), 1);
    REQUIRE(out.size() == 9);
    CHECK(out[0] == 0xFF);
    CHECK(out[1] == 0xFF);
    CHECK(out[2] == 0xFF);
    CHECK(out[3] == 0xFF);
    CHECK(out[4] == 0x01); // payload follows the marker directly
}

TEST_CASE("SerialProtocol marker counter wraps and survives multi-frame encodes")
{
    SerialProtocol protocol{2};

    // Three frames in one call: counter 0 -> 3, still no marker.
    const std::vector<uint16_t> threeFrames{0, 0, 0, 0, 0, 0};
    CHECK(protocol.encode(threeFrames.data(), 3).size() == 15);

    // Next call: counter 3 >= 2 -> marker, counter wraps to 1 (3 - 2).
    const std::vector<uint16_t> frame{0x0001, 0x0002};
    CHECK(protocol.encode(frame.data(), 1).size() == 14);

    // Counter now 2 -> next call starts with a marker again.
    CHECK(protocol.encode(frame.data(), 1).size() == 14);
    // Counter now 1 -> plain payload.
    CHECK(protocol.encode(frame.data(), 1).size() == 5);
}

TEST_CASE("SerialProtocol counter wraps modulo the interval, not by subtraction")
{
    // A block carrying more frames than the interval overshoots by more than
    // one interval. Subtracting a single interval per call would leave the
    // counter permanently above the threshold.
    SerialProtocol protocol{8};

    const std::vector<uint16_t> tenFrames(20, 0);
    CHECK(protocol.encode(tenFrames.data(), 10).size() == 50);   // counter 0 -> 10
    for (int call{0}; call < 4; ++call)                          // due every call
        CHECK(protocol.encode(tenFrames.data(), 10).size() == 59);
    // Counter is 16 % 8 == 0 -> 10 here; subtraction would have left it at 18.
    const std::vector<uint16_t> frame{0x0001, 0x0002};
    CHECK(protocol.encode(frame.data(), 1).size() == 14);         // 10 % 8 == 2 -> 3
    CHECK(protocol.encode(frame.data(), 1).size() == 5);          // 3 < 8, no marker
}

TEST_CASE("SerialProtocol restarts the cadence from a scheduled header")
{
    SerialProtocol protocol{4};
    const std::vector<uint16_t> frame{0x0001, 0x0002};

    // Counter 0 -> 3.
    CHECK(protocol.encode(frame.data(), 1).size() == 5);
    CHECK(protocol.encode(frame.data(), 1).size() == 5);
    CHECK(protocol.encode(frame.data(), 1).size() == 5);

    // Scheduled header pre-empts at counter 3 and resets it to 0, so the next
    // header is a full interval away rather than one frame later.
    protocol.setStreamConfig(3, 16);
    CHECK(protocol.encode(frame.data(), 1).size() == 14);
    CHECK(protocol.encode(frame.data(), 1).size() == 5);
    CHECK(protocol.encode(frame.data(), 1).size() == 5);
    CHECK(protocol.encode(frame.data(), 1).size() == 5);
    CHECK(protocol.encode(frame.data(), 1).size() == 14);
}

TEST_CASE("SerialProtocol encodes zero frames to an empty buffer")
{
    SerialProtocol protocol{1};

    // Nothing pending: empty input yields empty output, no marker.
    CHECK(protocol.encode(nullptr, 0).empty());

    // Queue one message so the marker becomes due...
    const std::vector<uint16_t> frame{0x0001, 0x0002};
    CHECK(protocol.encode(frame.data(), 1).size() == 5);

    // ...then a zero-frame encode emits only the pending header.
    const auto& out = protocol.encode(nullptr, 0);
    REQUIRE(out.size() == SerialProtocol::kHeaderBytes);
    CHECK(out[0] == 0xFF);
    CHECK(out[1] == 0xFF);
    CHECK(out[2] == 0xFF);
    CHECK(out[3] == 0xFF);

    // Marker consumed; empty again.
    CHECK(protocol.encode(nullptr, 0).empty());
}

// The header is only decodable because a 4 x 0xFF run cannot occur in data.
// Both high bytes are masked to 0x7F, so the longest possible run is 2 - a
// checksum byte followed by the next frame's low byte. If that ever stops
// holding, the receiver silently mistakes samples for a header.
TEST_CASE("SerialProtocol never emits four consecutive 0xFF in a data region")
{
    std::mt19937 rng{0xC0FFEEu};
    std::uniform_int_distribution<uint32_t> sample{0, 0xFFFF};
    std::uniform_int_distribution<int> frames{1, 16};

    SUBCASE("randomised samples, markers suppressed")
    {
        // A marker interval far beyond the frames encoded here, and no
        // setStreamConfig call, means every byte below is data.
        SerialProtocol protocol{1'000'000};

        std::vector<uint8_t> wire{};
        std::vector<uint16_t> block{};
        for (int call{0}; call < 2000; ++call) {
            const int n{frames(rng)};
            block.resize(static_cast<size_t>(n) * 2);
            for (uint16_t& v : block) v = static_cast<uint16_t>(sample(rng));

            const auto& out = protocol.encode(block.data(), n);
            wire.insert(wire.end(), out.begin(), out.end());
        }

        // Concatenated across calls, so the call boundary is covered too.
        CHECK(longestFfRun(wire) <= 2);
    }

    SUBCASE("randomised samples, markers interleaved")
    {
        SerialProtocol protocol{3};
        protocol.setStreamConfig(7, 1);

        std::vector<uint8_t> wire{};
        std::vector<size_t>  markerOffsets{};
        std::vector<uint16_t> block{};
        for (int call{0}; call < 2000; ++call) {
            const int n{frames(rng)};
            block.resize(static_cast<size_t>(n) * 2);
            for (uint16_t& v : block) v = static_cast<uint16_t>(sample(rng));

            const auto& out = protocol.encode(block.data(), n);
            // Data is 5 bytes per frame; a 9-byte header shifts the total out
            // of that multiple.
            if (out.size() % 5 == 4) markerOffsets.push_back(wire.size());
            wire.insert(wire.end(), out.begin(), out.end());
        }
        REQUIRE(!markerOffsets.empty());

        // Every maximal 0xFF run of 4 or more *ends* where an emitted marker
        // ends. It need not start there: a data checksum of 0xFF immediately
        // before a marker extends the run to 5. Hence the receiver rule -
        // consume the whole run, then read the frame type.
        size_t matched{0};
        size_t i{0};
        while (i < wire.size()) {
            if (wire[i] != 0xFF) { ++i; continue; }
            const size_t start{i};
            while (i < wire.size() && wire[i] == 0xFF) ++i;
            const size_t len{i - start};
            if (len < 4) {
                // A data-only run can never reach the marker length.
                CHECK(len <= 2);
                continue;
            }
            // At most one data byte (a checksum) can precede the marker.
            CHECK(len <= 5);
            CHECK(std::find(markerOffsets.begin(), markerOffsets.end(), i - 4)
                  != markerOffsets.end());
            ++matched;
        }
        CHECK(matched == markerOffsets.size());
    }
}
