// Native test for SerialDecoder.h (WP4 header decode).
#include <cstdio>
#include <cstdint>
#include <vector>
#include <cstdlib>

#include "SerialDecoder.h"

static std::vector<std::pair<uint16_t, uint16_t>> g_data;
static std::vector<std::pair<uint32_t, uint16_t>> g_cfg;
static std::vector<std::pair<uint16_t, uint8_t>> g_tune;

static void onData(uint16_t d[2]) { g_data.push_back({ d[0], d[1] }); }
static void onCfg(uint32_t rate, uint16_t blk) { g_cfg.push_back({ rate, blk }); }
static void onTune(uint16_t fs, uint8_t gq) { g_tune.push_back({ fs, gq }); }

static int g_fail = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while (0)

static void header(std::vector<uint8_t>& s, uint8_t rateCode, uint16_t blk, bool corruptChk = false) {
    for (int i = 0; i < 4; i++) s.push_back(0xFF);
    uint8_t type = 0x01;
    uint8_t lo = blk & 0x7F, hi = (blk >> 7) & 0x7F;
    uint8_t chk = (type ^ rateCode ^ lo ^ hi) & 0x7F;
    if (corruptChk) chk ^= 0x01;
    s.push_back(type); s.push_back(rateCode); s.push_back(lo); s.push_back(hi); s.push_back(chk);
}

// Tuning header, matching host-bridge/src/TuningFrame.cpp: fullScaleCounts is
// halved on the wire, rounded (n + 1) / 2.
static void tuningHeader(std::vector<uint8_t>& s, uint16_t fullScaleCounts,
                         uint8_t gammaQ, bool corruptChk = false) {
    for (int i = 0; i < 4; i++) s.push_back(0xFF);
    const uint8_t type = 0x02;
    const uint16_t half = (uint16_t)((fullScaleCounts + 1) / 2);
    const uint8_t lo = half & 0x7F, hi = (half >> 7) & 0x7F;
    uint8_t chk = (type ^ lo ^ hi ^ gammaQ) & 0x7F;
    if (corruptChk) chk ^= 0x01;
    s.push_back(type); s.push_back(lo); s.push_back(hi); s.push_back(gammaQ); s.push_back(chk);
}

static void message(std::vector<uint8_t>& s, uint16_t ch0, uint16_t ch1) {
    // Matches the bridge encoder: high bytes masked to 0x7F.
    uint8_t b[5];
    b[0] = ch0 & 0xFF; b[1] = (ch0 >> 8) & 0x7F;
    b[2] = ch1 & 0xFF; b[3] = (ch1 >> 8) & 0x7F;
    b[4] = b[0] ^ b[1] ^ b[2] ^ b[3];
    for (int i = 0; i < 5; i++) s.push_back(b[i]);
}

// Feed the stream in chunks of the given size to exercise buffer boundaries.
static void feed(SerialDecoder& d, const std::vector<uint8_t>& s, size_t chunk) {
    for (size_t i = 0; i < s.size(); i += chunk) {
        size_t n = (i + chunk <= s.size()) ? chunk : s.size() - i;
        d.process(s.data() + i, (int)n);
    }
}

int main() {
    // 1: header then data, at every chunk size.
    for (size_t chunk = 1; chunk <= 64; chunk++) {
        g_data.clear(); g_cfg.clear();
        SerialDecoder d(onData, onCfg);
        std::vector<uint8_t> s;
        header(s, 7, 1);                       // 375 Hz, 1 sample per block
        for (uint16_t i = 0; i < 20; i++) message(s, i * 137, 32767 - i * 91);
        feed(d, s, chunk);
        CHECK(g_cfg.size() == 1);
        CHECK(g_cfg[0].first == 375000u && g_cfg[0].second == 1);
        CHECK(g_data.size() == 20);
        CHECK(d.synced());
        for (uint16_t i = 0; i < g_data.size(); i++) {
            CHECK(g_data[i].first == (uint16_t)(i * 137));
            CHECK(g_data[i].second == (uint16_t)(32767 - i * 91));
        }
    }

    // 2: the accepted ladder, rateCode -> rateMilliHz. The ladder runs 0..10;
    //    this firmware takes 3..10, because a stream above the 6 kHz
    //    DAC_TICK_HZ would be decimated with no anti-alias filter.
    {
        const uint32_t expect[11] = { 48000000,24000000,12000000,6000000,3000000,1500000,
                                      750000,375000,187500,93750,46875 };
        for (uint8_t rc = MIN_RATE_CODE; rc <= MAX_RATE_CODE; rc++) {
            g_cfg.clear();
            SerialDecoder d(onData, onCfg);
            std::vector<uint8_t> s; header(s, rc, 16);
            feed(d, s, 3);
            CHECK(g_cfg.size() == 1 && g_cfg[0].first == expect[rc]);
        }
    }

    // 3: rate codes above the tick rate are rejected, not silently decimated.
    //    12 kHz is reachable in practice: 60 kB/s fits the 1.2 Mbaud link.
    for (uint8_t rc = 0; rc < MIN_RATE_CODE; rc++) {
        g_cfg.clear();
        SerialDecoder d(onData, onCfg);
        std::vector<uint8_t> s; header(s, rc, 16);
        for (uint16_t i = 0; i < 10; i++) message(s, i, i);
        feed(d, s, 3);
        CHECK(g_cfg.empty());
        CHECK(!d.synced());
        CHECK(d.headersRejected() == 1);
    }

    // 4: blockSamples across the 7-bit split.
    for (uint16_t blk : { (uint16_t)1, (uint16_t)16, (uint16_t)127, (uint16_t)128, (uint16_t)300, (uint16_t)16383 }) {
        g_cfg.clear();
        SerialDecoder d(onData, onCfg);
        std::vector<uint8_t> s; header(s, 3, blk);
        feed(d, s, 5);
        CHECK(g_cfg.size() == 1 && g_cfg[0].second == blk);
    }

    // 5: blockSamples 0 is reported as 1. A zero would otherwise divide into
    //    the playout's own sizing terms.
    {
        g_data.clear(); g_cfg.clear();
        SerialDecoder d(onData, onCfg);
        std::vector<uint8_t> s; header(s, 3, 0);
        for (uint16_t i = 0; i < 4; i++) message(s, i, i);
        feed(d, s, 5);
        CHECK(g_cfg.size() == 1 && g_cfg[0].second == 1);
        CHECK(g_data.size() == 4);
    }

    // 6: bad header checksum -> no config, no data, stays unsynced;
    //    the next good header recovers.
    {
        g_data.clear(); g_cfg.clear();
        SerialDecoder d(onData, onCfg);
        std::vector<uint8_t> s;
        header(s, 7, 1, /*corruptChk=*/true);
        for (uint16_t i = 0; i < 10; i++) message(s, i, i);
        feed(d, s, 7);
        CHECK(g_cfg.empty());
        CHECK(g_data.empty());
        CHECK(!d.synced());
        CHECK(d.headersRejected() == 1);

        std::vector<uint8_t> s2;
        header(s2, 3, 16);
        for (uint16_t i = 0; i < 10; i++) message(s2, 100 + i, 200 + i);
        feed(d, s2, 7);
        CHECK(g_cfg.size() == 1 && g_cfg[0].first == 6000000u);
        CHECK(g_data.size() == 10);
    }

    // 7: an unknown frame type and an out-of-range rate code are both rejected,
    //    with valid checksums, so it is the type/range check that catches them.
    for (int variant = 0; variant < 2; variant++) {
        g_cfg.clear(); g_tune.clear();
        SerialDecoder d(onData, onCfg, onTune);
        std::vector<uint8_t> s;
        for (int i = 0; i < 4; i++) s.push_back(0xFF);
        uint8_t type = variant == 0 ? 0x03 : 0x01;   // 0x03: not a defined type
        uint8_t rc = variant == 0 ? 3 : 11;
        uint8_t lo = 16, hi = 0;
        s.push_back(type); s.push_back(rc); s.push_back(lo); s.push_back(hi);
        s.push_back((type ^ rc ^ lo ^ hi) & 0x7F);
        feed(d, s, 9);
        CHECK(g_cfg.empty() && g_tune.empty() && !d.synced());
    }

    // 8: a new header while synced reconfigures, and message phase restarts
    //    after it.
    {
        g_data.clear(); g_cfg.clear();
        SerialDecoder d(onData, onCfg);
        std::vector<uint8_t> s;
        header(s, 7, 1);
        for (uint16_t i = 0; i < 5; i++) message(s, i, i);
        header(s, 3, 16);
        for (uint16_t i = 0; i < 5; i++) message(s, 1000 + i, 1000 + i);
        for (size_t chunk = 1; chunk <= 32; chunk++) {
            g_data.clear(); g_cfg.clear();
            SerialDecoder d2(onData, onCfg);
            feed(d2, s, chunk);
            CHECK(g_cfg.size() == 2);
            CHECK(g_cfg[0].first == 375000u && g_cfg[1].first == 6000000u);
            CHECK(g_data.size() == 10);
            CHECK(g_data[9].first == 1004);
        }
    }

    // 9: data containing the longest legal 0xFF run (checksum 0xFF followed by
    //    a low byte 0xFF) must not be mistaken for a marker.
    {
        g_data.clear(); g_cfg.clear();
        SerialDecoder d(onData, onCfg);
        std::vector<uint8_t> s;
        header(s, 3, 16);
        // ch0 = 0x7FFF -> b0=0xFF b1=0x7F ; ch1 = 0x0080 -> b2=0x80 b3=0x00
        // chk = FF^7F^80^00 = 0x00. Then follow with a frame starting 0xFF.
        message(s, 0x7FFF, 0x0080);
        message(s, 0x00FF, 0x0000);
        message(s, 0x7FFF, 0x0000);   // b0=FF b1=7F b2=00 b3=00 chk=0x80
        feed(d, s, 4);
        CHECK(g_data.size() == 3);
        CHECK(d.synced());
        CHECK(d.headersAccepted() == 1);
    }

    // 10: a marker preceded by an unmasked 0xFF checksum makes a run of five.
    //     The frame before it is still delivered and the header still accepted.
    {
        // ch0=0x00FF, ch1=0x0000 -> b0=FF b1=00 b2=00 b3=00, chk = 0xFF.
        std::vector<uint8_t> probe;
        message(probe, 0x00FF, 0x0000);
        CHECK(probe[4] == 0xFF);

        for (size_t chunk = 1; chunk <= 32; chunk++) {
            g_data.clear(); g_cfg.clear();
            SerialDecoder d(onData, onCfg);
            std::vector<uint8_t> s;
            header(s, 3, 16);
            message(s, 1234, 5678);
            message(s, 0x00FF, 0x0000);   // checksum 0xFF, immediately before...
            header(s, 7, 1);              // ...the next header -> 0xFF x5
            message(s, 4321, 8765);

            // The run is five, ending at the marker's 4th byte.
            int run = 0, maxrun = 0;
            for (size_t i = 0; i < s.size(); i++) {
                run = (s[i] == 0xFF) ? run + 1 : 0;
                if (run > maxrun) maxrun = run;
            }
            CHECK(maxrun == 5);

            feed(d, s, chunk);
            CHECK(g_cfg.size() == 2);
            CHECK(g_cfg[0].first == 6000000u && g_cfg[1].first == 375000u);
            CHECK(g_data.size() == 3);
            CHECK(g_data[0].first == 1234 && g_data[0].second == 5678);
            CHECK(g_data[1].first == 0x00FF && g_data[1].second == 0x0000);
            CHECK(g_data[2].first == 4321 && g_data[2].second == 8765);
            CHECK(d.headersRejected() == 0);
            CHECK(d.checksumErrors() == 0);
        }
        printf("marker preceded by 0xFF checksum: run of 5 decoded\n");
    }

    // 11: sync runs of length 4..10 decode identically - the type field is the
    //     first non-0xFF byte.
    for (int extra = 0; extra <= 6; extra++) {
        g_data.clear(); g_cfg.clear();
        SerialDecoder d(onData, onCfg);
        std::vector<uint8_t> s;
        for (int i = 0; i < extra; i++) s.push_back(0xFF);
        header(s, 3, 16);
        for (uint16_t i = 0; i < 4; i++) message(s, i, i);
        feed(d, s, 3);
        CHECK(g_cfg.size() == 1 && g_cfg[0].first == 6000000u);
        CHECK(g_data.size() == 4);
    }

    // 12: one corrupted data byte - a UART bit error, the fault the frame
    //     checksum exists for. The frame carrying it fails and the decoder
    //     slides one byte at a time to re-frame. Message phase is never
    //     abandoned: no marker hunt, no reconfiguration, and the phase is
    //     exact again a few frames later.
    //
    //     Re-framing is not immediate and not free. An 8-bit XOR checksum
    //     matches a misaligned window once in 256, and two of them do here, so
    //     one bit error costs a lost frame plus two garbage samples. At 6 kHz
    //     that is 0.5 ms, and the decoder's 0x7FFF mask bounds what a garbage
    //     sample can command.
    {
        const int frames = 20;
        const int badFrame = 5;
        const int alignedFrom = 7;    // exact again from this sample on
        for (size_t chunk = 1; chunk <= 32; chunk++) {
            g_data.clear(); g_cfg.clear();
            SerialDecoder d(onData, onCfg);
            std::vector<uint8_t> s;
            header(s, 3, 16);
            for (uint16_t i = 0; i < frames; i++) message(s, 100 + i, 200 + i);
            s[9 + MSG_LEN * badFrame] ^= 0x01;      // flip a bit in the low byte

            feed(d, s, chunk);
            CHECK(d.synced());
            CHECK(d.headersAccepted() == 1);
            CHECK(d.headersRejected() == 0);
            CHECK(d.checksumErrors() == 5);         // the frame, then 4 windows
            CHECK(g_data.size() == frames - 1);     // one frame's worth lost
            for (size_t i = 0; i < g_data.size(); i++) {
                CHECK(g_data[i].first <= 0x7FFF && g_data[i].second <= 0x7FFF);
                if ((int)i < badFrame) {
                    CHECK(g_data[i].first == (uint16_t)(100 + i));
                    CHECK(g_data[i].second == (uint16_t)(200 + i));
                } else if ((int)i >= alignedFrom) {
                    CHECK(g_data[i].first == (uint16_t)(101 + i));
                    CHECK(g_data[i].second == (uint16_t)(201 + i));
                }
            }
        }
        printf("one corrupt byte: 1 frame lost, 2 garbage, phase exact after 3\n");
    }

    // 13: the live stream shape - randomised samples with a header at the
    //     bridge's ~10 Hz cadence, so the preceding checksum lands on 0xFF by
    //     chance now and then.
    {
        srand(7);
        g_data.clear(); g_cfg.clear();
        SerialDecoder d(onData, onCfg);
        std::vector<uint8_t> s;
        std::vector<std::pair<uint16_t, uint16_t>> want;
        int fiveRuns = 0;
        // P(checksum == 0xFF) is ~1/256, so the case needs a few thousand
        // headers to show up on its own.
        const int headers = 5000;
        for (int blk = 0; blk < headers; blk++) {
            size_t before = s.size();
            header(s, 3, 16);
            // Did the frame before this header end in a 0xFF checksum?
            if (before >= 1 && s[before - 1] == 0xFF) fiveRuns++;
            for (int i = 0; i < 20; i++) {
                uint16_t a = rand() & 0x7FFF, b = rand() & 0x7FFF;
                want.push_back({ a, b });
                message(s, a, b);
            }
        }
        int run = 0, maxrun = 0;
        for (size_t i = 0; i < s.size(); i++) {
            run = (s[i] == 0xFF) ? run + 1 : 0;
            if (run > maxrun) maxrun = run;
        }
        printf("live-shape stream: %d markers preceded by 0xFF, max run %d\n", fiveRuns, maxrun);
        CHECK(fiveRuns > 0);          // the case did occur
        CHECK(maxrun == 5);
        feed(d, s, 64);
        CHECK(d.headersAccepted() == (uint32_t)headers);
        CHECK(d.headersRejected() == 0);
        CHECK(d.checksumErrors() == 0);
        CHECK(g_data.size() == want.size());
        for (size_t i = 0; i < g_data.size() && i < want.size(); i++) {
            if (g_data[i] != want[i]) { printf("FAIL sample %zu\n", i); g_fail++; break; }
        }
    }

    // 14: a valid tuning header dispatches the tuning callback with the fields
    //     reassembled and the 2-count wire scaling undone. Even counts round
    //     trip exactly; an odd count comes back one high (host rounds up).
    for (size_t chunk = 1; chunk <= 16; chunk++) {
        struct { uint16_t in; uint16_t out; uint8_t gq; } cases[] = {
            { 4000, 4000, 10 },      // even, exact
            { 6667, 6668, 16 },      // odd default: lands one count high
            { 20000, 20000, 127 },   // the ceiling round-trips intact
            { 100, 100, 4 },         // the floor
        };
        for (auto &c : cases) {
            g_data.clear(); g_cfg.clear(); g_tune.clear();
            SerialDecoder d(onData, onCfg, onTune);
            std::vector<uint8_t> s;
            tuningHeader(s, c.in, c.gq);
            feed(d, s, chunk);
            CHECK(g_tune.size() == 1);
            CHECK(g_tune[0].first == c.out);
            CHECK(g_tune[0].second == c.gq);
            CHECK(g_cfg.empty());
            CHECK(d.headersAccepted() == 1);
        }
    }

    // 15: a tuning header with a bad checksum is discarded - no callback, and
    //     the decoder stays hunting for a marker.
    {
        g_tune.clear();
        SerialDecoder d(onData, onCfg, onTune);
        std::vector<uint8_t> s;
        tuningHeader(s, 4000, 16, /*corruptChk=*/true);
        feed(d, s, 3);
        CHECK(g_tune.empty());
        CHECK(!d.synced());
        CHECK(d.headersRejected() == 1);

        // ...and the next good tuning header recovers.
        std::vector<uint8_t> s2;
        tuningHeader(s2, 8000, 32);
        feed(d, s2, 3);
        CHECK(g_tune.size() == 1 && g_tune[0].first == 8000 && g_tune[0].second == 32);
    }

    // 16: rate and tuning headers interleave on the one stream - each goes to
    //     its own callback, and message phase still runs after a rate header.
    {
        g_data.clear(); g_cfg.clear(); g_tune.clear();
        SerialDecoder d(onData, onCfg, onTune);
        std::vector<uint8_t> s;
        header(s, 3, 16);
        for (uint16_t i = 0; i < 5; i++) message(s, i, i);
        tuningHeader(s, 5000, 24);
        header(s, 7, 1);
        for (uint16_t i = 0; i < 5; i++) message(s, 100 + i, 100 + i);
        tuningHeader(s, 12000, 8);
        feed(d, s, 4);
        CHECK(g_cfg.size() == 2);
        CHECK(g_cfg[0].first == 6000000u && g_cfg[1].first == 375000u);
        CHECK(g_data.size() == 10);
        CHECK(g_tune.size() == 2);
        CHECK(g_tune[0].first == 5000 && g_tune[0].second == 24);
        CHECK(g_tune[1].first == 12000 && g_tune[1].second == 8);
    }

    printf(g_fail ? "\n%d CHECK(s) FAILED\n" : "\nall checks passed\n", g_fail);
    return g_fail ? 1 : 0;
}
