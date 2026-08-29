// Off-target simulation of Playout.h (WP5): ZOH resampling + drift lock.
//
// The consumer's DAC_TICK_HZ tick defines simulated time. The producer delivers
// whole blocks at the stream rate, running fast or slow by a given ppm error
// against it - the host-audio-clock vs ESP-crystal mismatch the drift lock
// absorbs.
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <cstdlib>

#include "Playout.h"

static int g_fail = 0;
#define CHECK(cond) do { if (!(cond)) { printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while (0)

struct Result {
    uint32_t underrunTicks = 0;
    uint32_t resynced = 0;
    uint32_t dropped = 0;
    uint32_t minFillLast = 0;
    uint32_t maxFillLast = 0;
    int32_t corrPpm = 0;
    uint32_t underrunAfterSettle = 0;
    uint32_t minFillWorstAfterSettle = 0xFFFFFFFF;
    uint32_t maxFillWorstAfterSettle = 0;
};

// seconds of simulated time, producer clock error in ppm (+ = producer fast)
static Result run(uint32_t rateMilliHz, uint16_t blockSamples, double ppm,
                  int seconds, double jitterTicks = 0.0, int settleSeconds = 15) {
    SampleRing ring;
    Playout p;
    p.configure(rateMilliHz, blockSamples);

    const double rateHz = rateMilliHz / 1000.0;
    // Ticks of the consumer clock between block deliveries.
    const double blockPeriodTicks = (blockSamples / rateHz) * DAC_TICK_HZ / (1.0 + ppm * 1e-6);
    // Jitter perturbs each arrival around a fixed nominal schedule rather than
    // accumulating, which would random-walk the producer's rate. Arrivals stay
    // ordered - the UART is a stream, blocks cannot overtake.
    uint64_t blockIndex = 1;
    double nextBlockAt = blockPeriodTicks;
    auto scheduleNext = [&](double now) {
        blockIndex++;
        double nominal = blockIndex * blockPeriodTicks;
        if (jitterTicks > 0.0) {
            nominal += ((double)rand() / RAND_MAX - 0.5) * 2.0 * jitterTicks;
            if (nominal < now) nominal = now;
        }
        nextBlockAt = nominal;
    };

    Result r;
    playout_stats_t st;
    const uint64_t total = (uint64_t)seconds * DAC_TICK_HZ;
    uint32_t second = 0;

    for (uint64_t t = 0; t < total; t++) {
        if ((double)t >= nextBlockAt) {
            for (uint16_t i = 0; i < blockSamples; i++) {
                dac_sample_t s; s.ch0 = 1000; s.ch1 = 2000;
                ring.push(s);
            }
            scheduleNext((double)t);
        }
        p.tick(ring);
        if (p.takeStats(st)) {
            second++;
            r.underrunTicks += st.underrunTicks;
            r.resynced += st.resynced;
            r.dropped = st.dropped;
            r.minFillLast = st.minFill;
            r.maxFillLast = st.maxFill;
            r.corrPpm = st.corrPpm;
            if ((int)second > settleSeconds) {
                r.underrunAfterSettle += st.underrunTicks;
                if (st.minFill < r.minFillWorstAfterSettle) r.minFillWorstAfterSettle = st.minFill;
                if (st.maxFill > r.maxFillWorstAfterSettle) r.maxFillWorstAfterSettle = st.maxFill;
            }
        }
    }
    return r;
}

int main() {
    struct { const char *name; uint32_t rate; uint16_t blk; } ladder[] = {
        { "375 Hz",  375000, 1 },
        { "750 Hz",  750000, 2 },
        { "1500 Hz", 1500000, 4 },
        { "3000 Hz", 3000000, 8 },
        { "6000 Hz", 6000000, 16 },
    };

    // 1: configure() derives an exact phase increment on every accepted rate
    //    (codes 10..3), and - the tick being itself a ladder rate - a power of
    //    two, so the hold length is a whole number of ticks and does not
    //    alternate. A tick rate off the ladder breaks the second property for
    //    every rate at once, which is what this guards.
    {
        const uint32_t rates[]  = { 46875, 93750, 187500, 375000,
                                    750000, 1500000, 3000000, 6000000 };
        const uint32_t expect[] = { 512,   1024,  2048,   4096,
                                    8192,  16384,   32768,   65536 };
        for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
            Playout p;
            p.configure(rates[i], 16);
            const uint32_t inc = p.phaseInc();
            CHECK(inc == expect[i]);
            CHECK((inc & (inc - 1)) == 0);   // power of two -> uniform holds
        }
        printf("phase increments exact and power-of-two for the ladder\n");
    }

    // 2: 10 minutes at each rate, +/-100 ppm clock error. Pass = zero
    //    underruns after settling and min fill held at targetMin +/- 1.
    for (auto &L : ladder) {
        for (double ppm : { -100.0, 0.0, +100.0 }) {
            Result r = run(L.rate, L.blk, ppm, 600);
            Playout probe; probe.configure(L.rate, L.blk);
            const int target = probe.targetMin();
            printf("%-8s blk=%-3u %+7.1f ppm : under=%lu(settled %lu) minFill=%lu maxFill=%lu target=%d corr=%ld ppm resync=%lu drop=%lu\n",
                   L.name, L.blk, ppm,
                   (unsigned long)r.underrunTicks, (unsigned long)r.underrunAfterSettle,
                   (unsigned long)r.minFillWorstAfterSettle, (unsigned long)r.maxFillWorstAfterSettle,
                   target, (long)r.corrPpm, (unsigned long)r.resynced, (unsigned long)r.dropped);
            CHECK(r.underrunAfterSettle == 0);
            CHECK((int)r.minFillWorstAfterSettle >= target - 1);
            CHECK((int)r.minFillWorstAfterSettle <= target + 1);
            CHECK(r.dropped == 0);
            CHECK(r.resynced == 0);
        }
    }

    // 3: same, with +-8 ticks of block arrival jitter on top of the clock
    //    error - 1.33 ms at the 6 kHz tick, more than the ~1 ms of USB frame
    //    scheduling PLAYOUT_MARGIN_US is sized against.
    srand(99);
    for (auto &L : ladder) {
        Result r = run(L.rate, L.blk, +100.0, 300, /*jitterTicks=*/8.0, /*settle=*/20);
        Playout probe; probe.configure(L.rate, L.blk);
        printf("%-8s jitter +-1.33 ms   : under=%lu(settled %lu) minFill=%lu maxFill=%lu target=%d corr=%ld ppm\n",
               L.name, (unsigned long)r.underrunTicks, (unsigned long)r.underrunAfterSettle,
               (unsigned long)r.minFillWorstAfterSettle, (unsigned long)r.maxFillWorstAfterSettle,
               probe.targetMin(), (long)r.corrPpm);
        CHECK(r.dropped == 0);
        CHECK(r.underrunAfterSettle == 0);
        // +-1.33 ms of jitter against a 2.67 ms block period can coalesce two
        // blocks; more fill than that would be acquired latency, not jitter.
        CHECK(r.maxFillWorstAfterSettle <= (uint32_t)(2 * L.blk + probe.targetMin() + 8));
    }

    // 4: ZOH output frequency. Feed a known sine at the stream rate and count
    //    zero crossings of the DAC output over 10 s.
    for (auto &L : ladder) {
        SampleRing ring;
        Playout p;
        p.configure(L.rate, L.blk);
        const double rateHz = L.rate / 1000.0;
        const double sigHz = 40.0;
        const double blockPeriodTicks = (L.blk / rateHz) * DAC_TICK_HZ;
        double nextBlockAt = blockPeriodTicks;
        uint32_t srcIdx = 0;
        int crossings = 0;
        bool above = false;
        const uint64_t total = 10ull * DAC_TICK_HZ;
        for (uint64_t t = 0; t < total; t++) {
            if ((double)t >= nextBlockAt) {
                for (uint16_t i = 0; i < L.blk; i++) {
                    double v = sin(2.0 * M_PI * sigHz * (double)srcIdx / rateHz);
                    srcIdx++;
                    dac_sample_t s;
                    s.ch0 = (uint16_t)((v + 1.0) * 16383.0);
                    s.ch1 = s.ch0;
                    ring.push(s);
                }
                nextBlockAt += blockPeriodTicks;
            }
            dac_sample_t o = p.tick(ring);
            if (t > 2ull * DAC_TICK_HZ) {      // ignore the fill-up transient
                bool nowAbove = o.ch0 > 16383;
                if (nowAbove && !above) crossings++;
                above = nowAbove;
            }
        }
        double measured = crossings / 8.0;      // 8 s of measurement window
        printf("%-8s 40 Hz sine -> %.2f Hz at the DAC\n", L.name, measured);
        CHECK(fabs(measured - sigHz) < 0.5);
    }

    // 5: producer stalls -> output ramps to zero within ~300 ms and stays
    //    there; it comes back up when the stream resumes.
    {
        SampleRing ring;
        Playout p;
        p.configure(6000000, 16);
        const double blockPeriodTicks = (16 / 6000.0) * DAC_TICK_HZ;
        double nextBlockAt = blockPeriodTicks;

        auto step = [&](uint64_t ticks, bool feed) {
            dac_sample_t last;
            for (uint64_t t = 0; t < ticks; t++) {
                if (feed && (double)t >= nextBlockAt) {
                    for (int i = 0; i < 16; i++) { dac_sample_t s; s.ch0 = 30000; s.ch1 = 30000; ring.push(s); }
                    nextBlockAt += blockPeriodTicks;
                }
                last = p.tick(ring);
            }
            nextBlockAt = blockPeriodTicks;
            return last;
        };

        dac_sample_t v = step(DAC_TICK_HZ, true);
        CHECK(v.ch0 > 29000);                             // streaming: full level

        v = step(DAC_TICK_HZ / 10, false);                // 100 ms of stall
        CHECK(v.ch0 > 29000);                             // still holding, not ramping yet

        v = step(DAC_TICK_HZ / 4, false);                 // total 350 ms
        printf("after 350 ms stall: out=%u\n", v.ch0);
        CHECK(v.ch0 == 0);                                // ramped to silence

        v = step(DAC_TICK_HZ * 2, false);                 // stays at zero
        CHECK(v.ch0 == 0);

        v = step(DAC_TICK_HZ, true);                      // stream resumes
        printf("1 s after resume:   out=%u\n", v.ch0);
        CHECK(v.ch0 > 29000);
    }

    // 5b: the fault mute (the DAC-write-failure path) takes the same ramp as
    //     the underrun silence with a healthy stream running throughout - it
    //     must not need a starved ring to reach zero. processData() applies
    //     the same gain to the encoder floor, so the belts release with it.
    {
        SampleRing ring;
        Playout p;
        p.configure(6000000, 16);
        const double blockPeriodTicks = (16 / 6000.0) * DAC_TICK_HZ;
        double nextBlockAt = 0.0;
        uint64_t clock = 0;

        auto step = [&](uint64_t ticks) {
            dac_sample_t last{};
            for (uint64_t t = 0; t < ticks; t++, clock++) {
                if ((double)clock >= nextBlockAt) {
                    for (int i = 0; i < 16; i++) { dac_sample_t s; s.ch0 = 30000; s.ch1 = 30000; ring.push(s); }
                    nextBlockAt += blockPeriodTicks;
                }
                last = p.tick(ring);
            }
            return last;
        };

        dac_sample_t v = step(DAC_TICK_HZ);
        CHECK(v.ch0 > 29000);
        CHECK(p.gainQ16() == GAIN_ONE);

        p.setMuted(true);
        v = step(DAC_TICK_HZ / 20 + 2);                   // 50 ms ramp + slack
        printf("muted after 50 ms:  out=%u gain=%lu\n", v.ch0, (unsigned long)p.gainQ16());
        CHECK(v.ch0 == 0);
        CHECK(p.gainQ16() == 0);                          // the encoder floor goes with it

        v = step(DAC_TICK_HZ);                            // stays down while muted
        CHECK(v.ch0 == 0 && p.gainQ16() == 0);

        p.setMuted(false);
        v = step(DAC_TICK_HZ / 20 + 2);                   // recovers over the same 50 ms
        printf("unmuted after 50 ms: out=%u gain=%lu\n", v.ch0, (unsigned long)p.gainQ16());
        CHECK(v.ch0 > 29000);
        CHECK(p.gainQ16() == GAIN_ONE);
    }

    // 6: a host backlog dump must not become permanent latency - the stale
    //    samples are discarded and the fill returns to the target.
    {
        SampleRing ring;
        Playout p;
        p.configure(6000000, 16);
        for (int i = 0; i < 400; i++) { dac_sample_t s; s.ch0 = 100; s.ch1 = 100; ring.push(s); }
        p.tick(ring);
        printf("backlog 400 -> fill %lu after one tick (limit %d)\n",
               (unsigned long)ring.fill(), p.targetMin() + 4 * 16 + 8);
        CHECK(ring.fill() <= (uint32_t)(p.targetMin() + 16));
    }

    // 6b: block lengths the 512-entry ring cannot be sized around. The wire
    //     permits blockSamples up to 16383 (two 7-bit header fields) and the
    //     bridge sends it, but every sizing term in configure() is a fraction
    //     or a multiple of the block:
    //
    //       blk >= ~121  m_resyncLimit exceeds the ring and clamps to 510, so
    //                    the resync fires on ordinary block arrival
    //       blk >= ~454  targetMin + blk exceeds any reachable fill; the
    //                    unsigned resync subtraction wrapped to ~4e9 and
    //                    discard() clamped it to the whole ring, so a backlog
    //                    spike emptied the buffer - an underrun and a 50 ms
    //                    silence ramp on every fill
    //       blk >= 512   targetMin exceeds the ring the PLL drives minFill
    //                    into, so the error never reaches zero and the
    //                    correction pins at -PLL_MAX_PPM
    {
        // The sizing bound holds; the reported block length is not clamped.
        for (uint16_t blk : { (uint16_t)1, (uint16_t)16, (uint16_t)64,
                              (uint16_t)128, (uint16_t)512, (uint16_t)16383 }) {
            Playout p;
            p.configure(6000000, blk);
            CHECK(p.blockSizing() <= PLAYOUT_MAX_BLOCK);
            CHECK(p.blockSizing() == (blk < PLAYOUT_MAX_BLOCK ? blk : (uint16_t)PLAYOUT_MAX_BLOCK));
            CHECK(p.targetMin() < RING_SIZE);
            CHECK(p.resyncLimit() < RING_SIZE);
            // targetMin + block < resyncLimit makes the resync subtraction
            // safe: the discard only runs when fill > resyncLimit.
            CHECK((uint32_t)p.targetMin() + p.blockSizing() < p.resyncLimit());

            playout_stats_t st;
            SampleRing ring;
            for (uint64_t t = 0; t < DAC_TICK_HZ; t++) {
                dac_sample_t s; s.ch0 = 1000; s.ch1 = 2000;
                ring.push(s);
                p.tick(ring);
            }
            CHECK(p.takeStats(st));
            CHECK(st.blockSamples == blk);   // reported, not clamped
        }

        // A full ring at an oversized block must trim to the target, not
        // flush. Before the guard this discarded all 511.
        for (uint16_t blk : { (uint16_t)600, (uint16_t)4096, (uint16_t)16383 }) {
            SampleRing ring;
            Playout p;
            p.configure(6000000, blk);
            for (int i = 0; i < RING_SIZE + 50; i++) {
                dac_sample_t s; s.ch0 = 100; s.ch1 = 100; ring.push(s);
            }
            const uint32_t before = ring.fill();
            p.tick(ring);
            const uint32_t after = ring.fill();
            printf("blk=%-5u backlog %lu -> %lu (sizing %u, target %u, limit %lu)\n",
                   blk, (unsigned long)before, (unsigned long)after,
                   p.blockSizing(), p.targetMin(), (unsigned long)p.resyncLimit());
            CHECK(after > 0);                                    // not flushed
            CHECK(after >= (uint32_t)p.targetMin());             // margin kept
            CHECK(after <= (uint32_t)p.targetMin() + p.blockSizing());
        }

        // Streamed for 60 s. An oversized block is lossy regardless: 4096
        // samples do not fit a 512-entry ring, so ~87 % of every block is
        // dropped at the producer and the ring is empty for most of the block
        // period. The assertions check that the loss is reported as `drop` and
        // the ring stays coherent instead of being flushed on every fill.
        {
            Result r = run(6000000, 4096, 0.0, 60, 0.0, /*settle=*/20);
            printf("blk=4096  60 s      : under=%lu(settled %lu) minFill=%lu drop=%lu resync=%lu\n",
                   (unsigned long)r.underrunTicks, (unsigned long)r.underrunAfterSettle,
                   (unsigned long)r.minFillWorstAfterSettle,
                   (unsigned long)r.dropped, (unsigned long)r.resynced);
            CHECK(r.dropped > 0);            // loss reported at the producer
            CHECK(r.maxFillWorstAfterSettle > 0);
        }

        // The boundary the clamp lands on must stay clean. 64 ==
        // PLAYOUT_MAX_BLOCK at 6 kHz is 10.7 ms of arrival granularity:
        // large, legitimate, and exactly on the limit.
        {
            Result r = run(6000000, PLAYOUT_MAX_BLOCK, 0.0, 60, 0.0, /*settle=*/20);
            Playout probe; probe.configure(6000000, PLAYOUT_MAX_BLOCK);
            printf("blk=%-5d 60 s      : under=%lu(settled %lu) minFill=%lu target=%u drop=%lu resync=%lu\n",
                   PLAYOUT_MAX_BLOCK,
                   (unsigned long)r.underrunTicks, (unsigned long)r.underrunAfterSettle,
                   (unsigned long)r.minFillWorstAfterSettle, probe.targetMin(),
                   (unsigned long)r.dropped, (unsigned long)r.resynced);
            CHECK(r.underrunAfterSettle == 0);
            CHECK(r.dropped == 0);
            CHECK(r.resynced == 0);
            CHECK((int)r.minFillWorstAfterSettle >= (int)probe.targetMin() - 1);
        }
    }

    // 7: ring full is a drop, never a block, and the ring stays coherent.
    {
        SampleRing ring;
        for (int i = 0; i < RING_SIZE + 100; i++) {
            dac_sample_t s; s.ch0 = (uint16_t)i; s.ch1 = 0;
            ring.push(s);
        }
        CHECK(ring.fill() == RING_SIZE - 1);
        CHECK(ring.dropped() == 101);
        dac_sample_t s;
        CHECK(ring.pop(s) && s.ch0 == 0);
        CHECK(ring.discard(1000) == RING_SIZE - 2);
        CHECK(ring.fill() == 0);
        CHECK(!ring.pop(s));
    }

    printf(g_fail ? "\n%d CHECK(s) FAILED\n" : "\nall checks passed\n", g_fail);
    return g_fail ? 1 : 0;
}
