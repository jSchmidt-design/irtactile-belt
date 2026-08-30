// belt-tune - interactively set the belt's runtime-tunable encoder force curve.
//
// Two parameters travel over the serial link, without a reflash:
//   fullScaleCounts  how far the belt is pulled before force saturates
//   gamma            the shape of the ramp (fast-then-flat vs loose-then-wall)
//
// It shares only the serial port and the frame encoder with host-bridge, none
// of the shm-streaming code.
//
// It is open-loop. The data link is host -> device only (the firmware never
// replies), so there is no read-back: the effect is felt on the belt.
//
// A valid tuning header also holds the firmware's encoder floor gain up, so the
// belt stays live without a sample stream - but only for ~250 ms per header.
// The tool therefore resends at --rate-hz (default 20) from the moment the port
// opens; below ~8 Hz the hold lapses between headers and the belt stutters. On
// exit (Ctrl+C or EOF) the port closes, the hold lapses, and the belt releases
// in ~300 ms.
//
// The main thread sends; the operator's input loop runs on a detached thread.
// std::getline on stdin cannot be interrupted, so the thread owning it cannot
// be woken to shut down. Keeping the send loop on main puts both shutdown paths
// (Ctrl+C, a failed write) on the thread that owns the port; the input thread is
// abandoned in getline when the process exits.
//
// Usage:
//   belt-tune --port COM6 [--baud 1200000] [--rate-hz 20]
// then type "<fullScaleCounts> <gamma>" per line, e.g. "4000 0.6".

#include "SerialPort.h"
#include "TuningFrame.h"

#include <windows.h>   // SetConsoleCtrlHandler; SerialPort.h sets the guards

#include <atomic>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace {

constexpr int kDefaultBaud{1200000};
constexpr int kDefaultRateHz{20};
// Below this the firmware's 250 ms hold window sees fewer than two headers, so
// a single dropped frame lets the hold lapse mid-pull and the belt stutters.
constexpr int kMinRateHz{8};
constexpr int kMaxRateHz{1000};

struct TuneOptions {
    std::string port{};
    int         baud{kDefaultBaud};
    int         rateHz{kDefaultRateHz};
    bool        help{false};
};

struct Curve {
    uint16_t fullScaleCounts{tuning::kDefaultFullScaleCounts};
    uint8_t  gammaQ{tuning::kDefaultGammaQ};
};

// Shared with the detached input thread, so it has static storage duration: the
// reader can still be parked inside getline when main() returns, and must never
// be left pointing at a destroyed local.
std::mutex        g_curveMutex{};
Curve             g_curve{};
std::atomic<bool> g_stop{false};

void printUsage()
{
    std::printf(
        "belt-tune - interactively set the belt's encoder force curve\n"
        "\n"
        "Usage:\n"
        "  belt-tune --port <COMx> [options]\n"
        "\n"
        "Required:\n"
        "  --port <COMx>        serial port to write to (opened \\\\.\\COMx)\n"
        "Options:\n"
        "  --baud <n>           baud rate (default %d)\n"
        "  --rate-hz <n>        header resend rate (default %d, min %d)\n"
        "                       This is load-bearing: each header holds the belt\n"
        "                       live for ~250 ms, so a lower rate makes the belt\n"
        "                       stutter and slows the release on exit.\n"
        "  --help, -h, /?       this text\n"
        "\n"
        "Then type one \"<fullScaleCounts> <gamma>\" line per change, e.g.\n"
        "  4000 0.6\n"
        "fullScaleCounts is clamped to %u..%u, gamma to %.2f..%.4f.\n"
        "\n"
        "The tool is open-loop: the firmware never replies, so the effect is\n"
        "felt on the belt, not reported here. Ctrl+C or EOF releases the belt.\n",
        kDefaultBaud, kDefaultRateHz, kMinRateHz,
        tuning::kMinFullScaleCounts, tuning::kMaxFullScaleCounts,
        tuning::kMinGammaQ / 16.0, tuning::kMaxGammaQ / 16.0);
}

bool parseInt(const char* s, int& out)
{
    if (!s || !*s) return false;
    errno = 0;
    char* end{nullptr};
    const long v{std::strtol(s, &end, 10)};
    if (!end || *end != '\0' || errno == ERANGE || v < INT_MIN || v > INT_MAX)
        return false;
    out = static_cast<int>(v);
    return true;
}

bool parseArgs(int argc, char** argv, TuneOptions& opt, std::string& error)
{
    auto reject{[&error](std::string msg) {
        error = std::move(msg);
        return false;
    }};

    for (int i{1}; i < argc; ++i) {
        const std::string arg{argv[i]};
        auto next{[&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; }};

        if (arg == "--help" || arg == "-h" || arg == "/?") {
            opt.help = true;
        } else if (arg == "--port") {
            const char* v{next()};
            if (!v || !*v) return reject("--port needs a value");
            opt.port = v;
        } else if (arg == "--baud") {
            int n{0};
            if (!parseInt(next(), n) || n <= 0)
                return reject("--baud needs a positive integer");
            opt.baud = n;
        } else if (arg == "--rate-hz") {
            int n{0};
            if (!parseInt(next(), n))
                return reject("--rate-hz needs an integer");
            if (n < kMinRateHz)
                return reject("--rate-hz below " + std::to_string(kMinRateHz)
                    + " leaves fewer than two headers in the firmware's 250 ms "
                      "hold window; the belt would stutter");
            if (n > kMaxRateHz)
                return reject("--rate-hz above " + std::to_string(kMaxRateHz)
                    + " just floods the link");
            opt.rateHz = n;
        } else {
            return reject("unknown argument: " + arg);
        }
    }

    if (opt.help) return true;
    if (opt.port.empty()) return reject("--port is required");
    return true;
}

// Ctrl+C, Ctrl+Break and the console-close box. Returning TRUE ("handled, do
// not terminate") lets main()'s exit path run; the default handler would kill
// the process mid-write. The belt releases either way once the headers stop.
BOOL WINAPI onConsoleSignal(DWORD type)
{
    switch (type) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        g_stop.store(true, std::memory_order_relaxed);
        return TRUE;
    default:
        return FALSE;
    }
}

// The operator's input loop, on the detached thread. Parses one
// "<fullScaleCounts> <gamma>" line per change and publishes it; a malformed
// line is reported without disturbing the last-good value.
void inputLoop()
{
    std::string line{};
    while (!g_stop.load(std::memory_order_relaxed) && std::getline(std::cin, line)) {
        std::istringstream in{line};
        long   fsIn{0};
        double gammaIn{0.0};
        std::string trailing{};
        if (!(in >> fsIn >> gammaIn) || (in >> trailing)) {
            if (line.find_first_not_of(" \t\r") == std::string::npos)
                continue;   // blank line, no complaint
            std::printf("  ? expected \"<fullScaleCounts> <gamma>\", e.g. 4000 0.6\n");
            continue;
        }
        if (fsIn < 0) fsIn = 0;
        if (fsIn > UINT16_MAX) fsIn = UINT16_MAX;

        const uint16_t fsReq{static_cast<uint16_t>(fsIn)};
        uint16_t fsClamped{fsReq};
        if (fsClamped < tuning::kMinFullScaleCounts) fsClamped = tuning::kMinFullScaleCounts;
        if (fsClamped > tuning::kMaxFullScaleCounts) fsClamped = tuning::kMaxFullScaleCounts;

        const uint8_t gammaQ{tuning::gammaToWire(static_cast<float>(gammaIn))};

        {
            const std::lock_guard<std::mutex> lock{g_curveMutex};
            g_curve.fullScaleCounts = fsClamped;
            g_curve.gammaQ          = gammaQ;
        }

        const bool gammaClamped{gammaIn < tuning::kMinGammaQ / 16.0
                                || gammaIn > tuning::kMaxGammaQ / 16.0};

        std::printf("  -> full-scale = %u", fsClamped);
        if (fsClamped != fsReq) std::printf(" (clamped from %ld)", fsIn);
        std::printf(", gamma = %.3f", gammaQ / 16.0);
        if (gammaClamped) std::printf(" (clamped from %.3f)", gammaIn);
        std::printf("\n");
    }

    // EOF, or a stdin that went away: same as Ctrl+C - stop sending and let the
    // belt release.
    g_stop.store(true, std::memory_order_relaxed);
}

} // namespace

int main(int argc, char** argv)
{
    TuneOptions opt{};
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

    SetConsoleCtrlHandler(onConsoleSignal, TRUE);

    SerialPort serial{};
    if (!serial.open(opt.port, opt.baud)) {
        std::fprintf(stderr, "error: cannot open %s (%s, GetLastError=%lu)\n",
                     opt.port.c_str(), serial.lastFailure(),
                     static_cast<unsigned long>(serial.lastError()));
        return 1;
    }

    std::printf(
        "belt-tune: holding %s live, resending at %d Hz.\n"
        "  full-scale = %u counts, gamma = %.3f  (defaults; nothing tuned yet)\n"
        "Type \"<fullScaleCounts> <gamma>\" per line. Ctrl+C or EOF to release.\n",
        opt.port.c_str(), opt.rateHz,
        tuning::kDefaultFullScaleCounts, tuning::kDefaultGammaQ / 16.0);
    std::fflush(stdout);

    std::thread{inputLoop}.detach();

    // Send from the moment the port opens, using the defaults until the
    // operator types something: the cadence is what holds the belt live, so
    // waiting for first input would leave it limp until then.
    const auto period{std::chrono::milliseconds{1000 / opt.rateHz}};
    int exitCode{0};
    while (!g_stop.load(std::memory_order_relaxed)) {
        Curve snapshot{};
        {
            const std::lock_guard<std::mutex> lock{g_curveMutex};
            snapshot = g_curve;
        }
        const auto frame{tuning::encode(snapshot.fullScaleCounts, snapshot.gammaQ)};
        if (serial.write(frame.data(), frame.size()) == SerialPort::WriteStatus::Failed) {
            std::fprintf(stderr, "\nserial write failed (%s); exiting\n",
                         serial.lastFailure());
            exitCode = 1;
            break;
        }
        std::this_thread::sleep_for(period);
    }

    serial.close();
    std::printf("belt-tune: port closed, belt releasing (~300 ms).\n");
    return exitCode;
}
