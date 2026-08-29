// irTactileSerialBridge - standalone shm -> serial bridge.
//
// Attaches to an irTactile shared-memory stream (irtactile-shm C++ bindings),
// encodes two chosen channels with the SerialProtocol wire format (5 bytes per
// 2-channel frame + XOR checksum, sync marker every --marker-interval frames)
// and writes the byte stream to a serial port.
//
// Any rate on the stream ladder up to 6 kHz is forwarded as-is, and the marker
// carries the rate in-band, so the firmware reconfigures its playout without a
// recompile or a handshake. The marker cadence defaults to ~10 Hz at every
// rate, putting re-acquisition after a receiver reset at ~100 ms.
//
// The stream is checked at attach and on every header rewrite - a rate change
// alone arrives as ProducerRestarted, not Reconfigured. Missing channels,
// an off-ladder rate and a rate above 6 kHz are fatal. A publish cadence slower
// than the rate allows and a wire budget above 80% of the baud rate only warn.
//
// Every runtime diagnostic goes through shmlog (LOG_*). The compiled-in backend
// decides where the lines go - see HOST_BRIDGE_LOG_BACKEND in CMakeLists.txt.
// Argument errors and --help happen before the logger exists and go to the
// console.
//
// Usage:
//   irTactileSerialBridge --port COM6 --shm-name <resolved>_256k
//                         [--event-name <resolved>_256k] [--baud 1200000]
//                         [--channels 0,1] [--marker-interval <n>]
//                         [--legacy-marker] [--log-name <name>]
//                         [--log-source <id>]
//
// Names are the RESOLVED kernel names (base name plus the producer's
// size-class suffix "_256k"/"_512k"/"_1m"). Omitting --event-name selects
// polling mode.

#include "BridgeConfig.h"
#include "SerialPort.h"
#include "SerialProtocol.h"
#include "Throttle.h"
#include "WireParams.h"

#include <irtactile/shm/stream/win32.h>
#include <shmlog/Logger.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

using namespace irtactile::shm::stream;

namespace {

/// How often a repeating condition - skips, a stalled producer, a dead port -
/// may report. Never per occurrence: each recurs at the wake rate.
constexpr auto kReportInterval{std::chrono::seconds{5}};
/// Consecutive failed writes before the port is treated as down. Each failure
/// costs the drain one write timeout, so this cannot be large.
constexpr uint32_t kWriteFailuresBeforeBackoff{4};
/// How long the port stays down before a reopen is attempted.
constexpr auto kPortRetryInterval{std::chrono::seconds{2}};

/// Checks the current layout snapshot against what the wire format can carry,
/// and on success writes the encoder parameters `rateCode` and `markerInterval`.
/// Called at attach and on every header rewrite (both Reconfigured and
/// ProducerRestarted); a failure is fatal. The Client re-snapshots the header
/// before reporting either, so every Reader accessor is current here.
[[nodiscard]] bool checkStream(const Reader& rd, const Options& opt,
                               uint8_t& rateCode, size_t& markerInterval)
{
    const uint32_t channels{rd.channelCount()};
    const uint32_t rateMilliHz{rd.sampleRateMilliHz()};
    const double rateHz{static_cast<double>(rateMilliHz) / 1000.0};

    if (opt.channelA >= channels || opt.channelB >= channels) {
        LOG_ERROR("selected channels {},{} not present in stream ({} channels). "
                  "Remedy: pass --channels within the stream's channel count.",
                  opt.channelA, opt.channelB, channels);
        return false;
    }
    if (!rateCodeFor(rateMilliHz, rateCode)) {
        LOG_ERROR("stream rate {:.3f} Hz is not one of the 11 stream ladder rates. "
                  "Remedy: pick a listed rate in the irTactile stream profile.",
                  rateHz);
        return false;
    }
    if (rateMilliHz > kMaxSampleRateMilliHz) {
        LOG_ERROR("stream rate {:.3f} Hz exceeds the 6 kHz firmware maximum. "
                  "Remedy: lower the profile rate in the irTactile configuration.",
                  rateHz);
        return false;
    }

    // The firmware decodes any block size - samplesPerBlock is in the header -
    // so a slow publish cadence costs latency rather than compatibility. The
    // floor is one sample per block: 375 Hz at and above 375 Hz, the rate
    // itself below it.
    const double wantPublishHz{std::min(kMaxPublishHz, rateHz)};
    if (rd.publishHz() < wantPublishHz - 0.5) {
        LOG_WARN("stream publishes at {:.2f} Hz ({} samples/block), not the {:.2f} Hz "
                 "this rate allows - every block carries {:.2f} ms of latency. "
                 "Remedy: lower \"shm.framesPerPublish\" to the smallest value the rate divides.",
                 rd.publishHz(), rd.samplesPerBlock(), wantPublishHz, rd.periodMs());
    }

    // The header carries samplesPerBlock in two 7-bit fields, and
    // setStreamConfig() truncates anything wider. Truncation is silent on the
    // wire - the firmware would size its playout from a block count that is
    // not the one being sent - so it is called out here with the other
    // silent-misconfiguration checks.
    if (rd.samplesPerBlock() > SerialProtocol::kMaxBlockSamples) {
        LOG_WARN("stream publishes {} samples/block, more than the {} the marker header "
                 "can carry - the advertised block size will be wrong. "
                 "Remedy: lower \"shm.framesPerPublish\".",
                 rd.samplesPerBlock(), SerialProtocol::kMaxBlockSamples);
    }

    markerInterval = markerIntervalFor(opt.markerInterval, rateHz);

    // Wire budget: 8N1 => 10 bits per byte, so baud/10 bytes/s. The format
    // emits 5 bytes per frame plus a marker every markerInterval frames.
    const double markerBytes{static_cast<double>(SerialProtocol::markerBytes(opt.legacyMarker))};
    const double bytesPerSec{rateHz * (static_cast<double>(SerialProtocol::kFrameBytes)
                                       + markerBytes / static_cast<double>(markerInterval))};
    const double capacity{static_cast<double>(opt.baud) / 10.0};
    const double utilization{bytesPerSec / capacity};
    if (utilization > kBandwidthWarnFraction) {
        LOG_WARN("wire format needs {:.0f} bytes/s of {:.0f} bytes/s ({:.0f}%) "
                 "at {:.3f} Hz / {} baud. Remedy: raise --baud or lower the profile rate.",
                 bytesPerSec, capacity, utilization * 100.0, rateHz, opt.baud);
    }
    return true;
}

/// Shuts the logger down on every path out of wmain(), so a later early return
/// cannot be the one that forgets and loses the entries still in the ring.
struct LoggerShutdown {
    ~LoggerShutdown() { shmlog::Logger::Shutdown(); }
};

} // namespace

int wmain(int argc, wchar_t** argv)
{
    Options opt{};
    std::string parseError{};
    if (!parseArgs(argc, argv, opt, parseError)) {
        std::fprintf(stderr, "error: %s\n\n", parseError.c_str());
        printUsage();
        return 2;
    }
    if (opt.help) {
        printUsage();
        return 0;
    }

    // A failed Initialize leaves every LOG_* macro a silent no-op, so this one
    // failure has to report to the console.
    if (!shmlog::Logger::Initialize(opt.logName.c_str(), opt.logSourceId)) {
        std::fprintf(stderr,
                     "NOTE: shmlog partition \"%s\" unavailable; running without logging\n",
                     opt.logName.c_str());
    }
    const LoggerShutdown loggerShutdown{};

    // NotFound is the normal case: the bridge may start before irTactile.
    // shouldRetry() retries exactly the results a later attach can change.
    //
    // No StreamRequest: it asserts exact equality on a single field, while this
    // bridge needs "at least the two selected channels", "on the ladder" and
    // "at or below 6 kHz". checkStream() below covers all three and re-runs on
    // every reconfiguration.
    Client c{};
    OpenResult r{};
    const wchar_t* eventName{opt.eventName.empty() ? nullptr : opt.eventName.c_str()};
    // Throttled: the retry runs 4x a second and the wait is open-ended, so an
    // unthrottled line would scroll a whole session out of the log ring.
    Throttle waitReport{kReportInterval};
    while (shouldRetry(r = c.open(opt.shmName.c_str(), eventName))) {
        if (waitReport.due(std::chrono::steady_clock::now()))
            LOG_INFO("waiting for irTactile ({})...", toCString(r));
        std::this_thread::sleep_for(std::chrono::milliseconds{250});
    }
    if (r != OpenResult::Ok) {
        LOG_ERROR("open failed: {}", toCString(r));
        return 1;
    }

    const Reader& rd{c.reader()};
    LOG_INFO("attached to \"{}\": {} ch @ {:.3f} Hz, {} samples/block, period {:.2f} ms, {} mode",
             rd.deviceName(), rd.channelCount(), rd.sampleRateMilliHz() / 1000.0,
             rd.samplesPerBlock(), rd.periodMs(), c.nativeEvent() ? "event" : "polling");
    if (c.eventFallback())
        LOG_WARN("event \"{}\" not found -- falling back to polling. "
                 "Check the resolved event name.", narrow(opt.eventName.c_str()));

    uint8_t rateCode{0};
    size_t  markerInterval{0};
    if (!checkStream(rd, opt, rateCode, markerInterval)) {
        return 3;
    }

    // An absent port is the normal case, not a fatal one: the USB adapter may
    // be unplugged, the receiver may boot after the PC, or another process may
    // still hold the handle. All three are what the mid-stream retry below
    // recovers from, so the bridge starts in the port-down state rather than
    // exiting.
    SerialPort serial{};
    const bool portOpenAtStart{serial.open(opt.port, opt.baud)};
    if (!portOpenAtStart) {
        LOG_WARN("{} not available at start ({}, GetLastError={}); "
                 "streaming anyway and retrying the port every {}s",
                 opt.port, serial.lastFailure(), serial.lastError(),
                 kPortRetryInterval.count());
    }

    SerialProtocol protocol{markerInterval, opt.legacyMarker};
    // Schedule the first header, so the receiver knows the rate from the first
    // bytes rather than at the first rollover.
    protocol.setStreamConfig(rateCode, rd.samplesPerBlock());
    LOG_INFO("wire: {} marker every {} frames ({:.1f} Hz), rateCode {}, {} samples/block",
             opt.legacyMarker ? "legacy 4x0xFF" : "9-byte header",
             markerInterval,
             (rd.sampleRateMilliHz() / 1000.0) / static_cast<double>(markerInterval),
             rateCode, rd.samplesPerBlock());

    uint32_t backlogThreshold{backlogThresholdBytes(rd.samplesPerBlock(), opt.baud,
                                                    kTxQueueBytes)};
    std::vector<uint16_t> u16buf{};  // interleaved chA,chB, reused across blocks

    // Skipped-sample accounting. droppedBlocks() covers ring overruns; this
    // covers those *and* the blocks the latency policy jumps over, which
    // latest() advances past silently.
    uint64_t nextExpectedSample{0};
    bool     haveExpected{false};
    uint64_t skippedSamples{0};
    uint64_t reportedSkips{0};

    // Blocks that do not carry both selected channels. checkStream() rules this
    // out at attach and on every Reconfigured, so a non-zero count means a
    // block disagreed with its own layout snapshot.
    uint64_t unusableBlocks{0};

    // Write-path health. Each failed write costs the drain one write timeout,
    // so a port that has stopped accepting bytes is dropped rather than retried
    // per block, then reopened on a slow cadence. "Down" is one state, whether
    // the port was never up or went away mid-stream.
    uint32_t writeFailures{0};
    bool     portDown{!portOpenAtStart};
    uint64_t discardedSamples{0};   // lifetime
    uint64_t outageDiscarded{0};    // since the port went down
    auto     nextPortRetry{std::chrono::steady_clock::now() + kPortRetryInterval};

    Throttle skipReport{kReportInterval};
    Throttle stallReport{kReportInterval};
    Throttle portReport{kReportInterval};
    Throttle blockReport{kReportInterval};

    BlockView b{};
    for (;;) {
        const WaitResult w{c.wait()};

        // Both results mean the header was rewritten and the snapshot is
        // already refreshed; they differ only in which fields moved. Both
        // rebuild, because the split is not the one it reads as: Reconfigured
        // is raised for a *geometry* change alone, and sampleRateMilliHz is
        // not a geometry field - so a producer that changes only the rate,
        // holding samplesPerBlock, arrives here as ProducerRestarted.
        // Treating that as "nothing to re-derive" would leave the marker
        // advertising the previous rateCode for the rest of the session - the
        // firmware would play every later block at the wrong rate - and would
        // skip the ladder and 6 kHz checks entirely.
        //
        // Only the check result, the wire parameters and the threshold came
        // from the old header (the conversion buffer resizes lazily in the
        // drain loop), so re-deriving those is the whole rebuild.
        if (w == WaitResult::Reconfigured || w == WaitResult::ProducerRestarted) {
            LOG_INFO("{}: now {} ch @ {:.3f} Hz, {} samples/block; re-checking",
                     w == WaitResult::Reconfigured
                         ? "stream reconfigured"
                         : "producer restarted, geometry unchanged",
                     rd.channelCount(), rd.sampleRateMilliHz() / 1000.0,
                     rd.samplesPerBlock());
            if (!checkStream(rd, opt, rateCode, markerInterval)) {
                return 3;
            }
            protocol.setMarkerInterval(markerInterval);
            protocol.setStreamConfig(rateCode, rd.samplesPerBlock());
            backlogThreshold = backlogThresholdBytes(rd.samplesPerBlock(), opt.baud,
                                                     kTxQueueBytes);
            // firstSampleIndex restarts at 0 on a producer restart, so the
            // running expectation is meaningless until the next block.
            haveExpected = false;
            // The header changed under it, so any block anomaly is a new one.
            blockReport.reset();
            continue;
        }
        // Closed means the producer shut down cleanly (or the reader detached
        // after a failed re-validation). The contract is to drain what is left
        // and then stop, so the loop falls through to the drain once more and
        // breaks at the bottom; next() simply returns false on a detached
        // reader, which collapses that case to an immediate exit.
        const bool closing{w == WaitResult::Closed};

        // Drain every block the wake made available: the event is auto-reset,
        // so wakes coalesce once the bridge falls behind.
        //
        // The TX queue is the trigger - it measures accumulated wire latency
        // directly. Below the threshold every block goes out in order; above it
        // the drain jumps to the newest and lets the stale ones go.
        auto pull{[&]() -> bool {
            return serial.pendingTxBytes() > backlogThreshold ? c.latest(b) : c.next(b);
        }};

        while (pull()) {
            if (b.sampleCount == 0) continue;

            // Account before classifying, so skippedSamples means "samples the
            // reader never offered" and nothing else - blocks dropped below
            // have their own counters.
            if (haveExpected && b.firstSampleIndex > nextExpectedSample)
                skippedSamples += b.firstSampleIndex - nextExpectedSample;
            nextExpectedSample = b.firstSampleIndex + b.sampleCount;
            haveExpected = true;

            const float* chA{b.channel(opt.channelA)};
            const float* chB{b.channel(opt.channelB)};
            // checkStream() validated both channel indices against the layout
            // snapshot, so a null pointer means this block does not match the
            // layout it was published under.
            if (!chA || !chB) {
                unusableBlocks++;
                if (blockReport.due(std::chrono::steady_clock::now()))
                    LOG_WARN("block at sample {} carries no data for channels {},{} "
                             "({} channels in the block, {} in the current layout); "
                             "{} blocks unusable so far",
                             b.firstSampleIndex, opt.channelA, opt.channelB,
                             b.channelCount(), rd.channelCount(), unusableBlocks);
                continue;
            }

            // Consume the ring at the same rate whether or not the port is
            // usable: the producer must never back up behind this consumer.
            if (portDown) {
                discardedSamples += b.sampleCount;
                outageDiscarded  += b.sampleCount;
                continue;
            }

            // Interleave the two selected channels and scale each to the 15-bit
            // unsigned range the wire format carries.
            u16buf.resize(SerialProtocol::kChannels * b.sampleCount);
            for (uint32_t i{0}; i < b.sampleCount; ++i) {
                u16buf[2 * i]     = toWireSample(chA[i]);
                u16buf[2 * i + 1] = toWireSample(chB[i]);
            }

            const std::vector<uint8_t>& bytes{protocol.encode(u16buf.data(), b.sampleCount)};
            if (bytes.empty()) continue;

            const SerialPort::WriteStatus st{serial.write(bytes.data(), bytes.size())};
            if (st == SerialPort::WriteStatus::Ok) {
                writeFailures = 0;
                continue;
            }

            // A TimedOut write has already put part of the buffer on the wire,
            // so the byte stream is cut mid-frame and the receiver's 5-byte
            // framing is off until the next marker - up to a full marker
            // interval, ~100 ms at the default cadence. Re-arming the header
            // costs 9 bytes and relocks it on the very next block instead, the
            // same reasoning as the reopen path below. Harmless on Failed,
            // where nothing more goes out until the port is back anyway.
            protocol.setStreamConfig(rateCode, rd.samplesPerBlock());

            // A failed port is non-fatal mid-stream: report the first failure of
            // a run, then let the count decide whether the port is merely slow
            // or gone.
            if (++writeFailures == 1)
                LOG_WARN("serial write failed ({}, GetLastError={}); continuing",
                         serial.lastFailure(), serial.lastError());
            if (writeFailures >= kWriteFailuresBeforeBackoff) {
                LOG_WARN("{} unusable after {} consecutive failed writes; "
                         "dropping output and retrying every {}s",
                         opt.port, writeFailures, kPortRetryInterval.count());
                serial.close();
                portDown      = true;
                nextPortRetry = std::chrono::steady_clock::now() + kPortRetryInterval;
                portReport.reset();
            }
        }

        const auto now{std::chrono::steady_clock::now()};

        if (portDown && now >= nextPortRetry) {
            if (serial.open(opt.port, opt.baud)) {
                LOG_INFO("{} opened after {} discarded samples", opt.port, outageDiscarded);
                portDown        = false;
                writeFailures   = 0;
                outageDiscarded = 0;
                // The other end may have restarted with the port: re-arm the
                // header so it relocks on the next block.
                protocol.setStreamConfig(rateCode, rd.samplesPerBlock());
            } else {
                nextPortRetry = now + kPortRetryInterval;
                if (portReport.due(now))
                    LOG_WARN("{} still unavailable ({}, GetLastError={}); "
                             "{} samples discarded so far",
                             opt.port, serial.lastFailure(), serial.lastError(),
                             outageDiscarded);
            }
        }

        // Quiet (engine up, device not driven) is legitimate; Stalled is not.
        if (c.state() == StreamState::Stalled && stallReport.due(now))
            LOG_WARN("irTactile appears wedged (no publish in {:.0f} ms)", rd.staleAfterMs());

        if (skippedSamples != reportedSkips && skipReport.due(now)) {
            LOG_WARN("skipped {} samples to bound wire latency ({} total, {} ring drops)",
                     skippedSamples - reportedSkips, skippedSamples, rd.droppedBlocks());
            reportedSkips = skippedSamples;
        }

        if (closing) break;
    }

    LOG_INFO("producer closed. {} blocks dropped, {} samples skipped, "
             "{} samples discarded, {} blocks unusable.",
             rd.droppedBlocks(), skippedSamples, discardedSamples, unusableBlocks);
    return 0;
}
