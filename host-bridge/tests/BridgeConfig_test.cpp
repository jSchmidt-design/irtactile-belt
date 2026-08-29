#include <doctest/doctest.h>

#include "BridgeConfig.h"

#include <string>
#include <vector>

// The command line. Every option here selects a stream, a port or a wire
// parameter, so a value accepted loosely - a truncated number, a channel index
// that silently stayed at its default - misconfigures the bridge in a way that
// only shows up as a wrong or silent belt.

namespace {

/// Runs parseArgs over a literal argument list, prepending the argv[0] the
/// parser skips.
struct Parse {
    Options     opt{};
    std::string error{};
    bool        ok{false};

    explicit Parse(std::initializer_list<const wchar_t*> args)
    {
        std::vector<wchar_t*> argv{};
        argv.push_back(const_cast<wchar_t*>(L"irTactileSerialBridge.exe"));
        for (const wchar_t* a : args) argv.push_back(const_cast<wchar_t*>(a));
        ok = parseArgs(static_cast<int>(argv.size()), argv.data(), opt, error);
    }
};

/// The two required options, for cases testing something else.
constexpr const wchar_t* kPort{L"--port"};
constexpr const wchar_t* kCom6{L"COM6"};
constexpr const wchar_t* kShm{L"--shm-name"};
constexpr const wchar_t* kName{L"irTactile_Stream_Primary_256k"};

} // namespace

TEST_CASE("parseUInt accepts a complete decimal number")
{
    uint32_t v{0};
    REQUIRE(parseUInt(L"0", v));
    CHECK(v == 0);
    REQUIRE(parseUInt(L"1200000", v));
    CHECK(v == 1200000);
    REQUIRE(parseUInt(L"4294967295", v));
    CHECK(v == 4294967295u);
}

TEST_CASE("parseUInt rejects anything that is not a whole number")
{
    uint32_t v{12345};
    CHECK_FALSE(parseUInt(nullptr, v));
    CHECK_FALSE(parseUInt(L"", v));
    CHECK_FALSE(parseUInt(L"abc", v));
    CHECK_FALSE(parseUInt(L"12abc", v));      // trailing character
    CHECK_FALSE(parseUInt(L"12 ", v));        // trailing space
    CHECK_FALSE(parseUInt(L"1.5", v));
    CHECK_FALSE(parseUInt(L"0x10", v));       // base 10 only
    // wcstoul wraps a negative into the range; the explicit sign check is what
    // stops --baud -1 becoming 4294967295.
    CHECK_FALSE(parseUInt(L"-1", v));
    CHECK_FALSE(parseUInt(L"-4294967295", v));
    // Above 32 bits. Rejected by the explicit UINT32_MAX bound, not by
    // ERANGE - unsigned long is only 32 bits on this host.
    CHECK_FALSE(parseUInt(L"4294967296", v));
    CHECK_FALSE(parseUInt(L"99999999999999999999", v));
    // A rejected parse leaves the caller's value alone.
    CHECK(v == 12345);
}

TEST_CASE("parseChannels splits a distinct pair")
{
    uint32_t a{99}, b{99};
    REQUIRE(parseChannels(L"0,1", a, b));
    CHECK(a == 0);
    CHECK(b == 1);
    REQUIRE(parseChannels(L"7,2", a, b));
    CHECK(a == 7);
    CHECK(b == 2);
}

TEST_CASE("parseChannels rejects a malformed or degenerate pair")
{
    uint32_t a{99}, b{99};
    CHECK_FALSE(parseChannels(nullptr, a, b));
    CHECK_FALSE(parseChannels(L"", a, b));
    CHECK_FALSE(parseChannels(L"0", a, b));      // no comma
    CHECK_FALSE(parseChannels(L"0,", a, b));     // empty second
    CHECK_FALSE(parseChannels(L",1", a, b));     // empty first
    CHECK_FALSE(parseChannels(L"0,1,2", a, b));  // trailing junk in the second
    CHECK_FALSE(parseChannels(L"a,b", a, b));
    // The two protocol slots must carry different channels; one channel in
    // both would be a silent duplicate rather than a stereo pair.
    CHECK_FALSE(parseChannels(L"1,1", a, b));
    CHECK_FALSE(parseChannels(L"0,0", a, b));
}

TEST_CASE("parseArgs fills the required options")
{
    const Parse p{kPort, kCom6, kShm, kName};
    REQUIRE(p.ok);
    CHECK_FALSE(p.opt.help);
    CHECK(p.opt.port == "COM6");
    CHECK(p.opt.shmName == L"irTactile_Stream_Primary_256k");
    CHECK(p.opt.eventName.empty());        // polling mode
}

TEST_CASE("parseArgs leaves untouched options at their documented defaults")
{
    const Parse p{kPort, kCom6, kShm, kName};
    REQUIRE(p.ok);
    CHECK(p.opt.baud == 1200000);
    CHECK(p.opt.channelA == 0);
    CHECK(p.opt.channelB == 1);
    CHECK(p.opt.markerInterval == 0);      // 0 => derive from the stream rate
    CHECK_FALSE(p.opt.legacyMarker);
    CHECK(p.opt.logName == kDefaultLogName);
    CHECK(p.opt.logSourceId == 0);
}

TEST_CASE("parseArgs requires both --port and --shm-name")
{
    const Parse noPort{kShm, kName};
    CHECK_FALSE(noPort.ok);
    CHECK(noPort.error.find("--port") != std::string::npos);

    const Parse noShm{kPort, kCom6};
    CHECK_FALSE(noShm.ok);
    CHECK(noShm.error.find("--shm-name") != std::string::npos);

    const Parse neither{};
    CHECK_FALSE(neither.ok);
}

TEST_CASE("parseArgs reads every optional value")
{
    const Parse p{kPort, kCom6, kShm, kName,
                  L"--event-name", L"irTactile_Event_Primary_256k",
                  L"--baud", L"921600",
                  L"--channels", L"3,5",
                  L"--marker-interval", L"250",
                  L"--legacy-marker",
                  L"--log-name", L"Bridge_B",
                  L"--log-source", L"7"};
    REQUIRE(p.ok);
    CHECK(p.opt.eventName == L"irTactile_Event_Primary_256k");
    CHECK(p.opt.baud == 921600);
    CHECK(p.opt.channelA == 3);
    CHECK(p.opt.channelB == 5);
    CHECK(p.opt.markerInterval == 250);
    CHECK(p.opt.legacyMarker);
    CHECK(p.opt.logName == "Bridge_B");
    CHECK(p.opt.logSourceId == 7);
}

TEST_CASE("parseArgs short-circuits the required check for --help")
{
    for (const wchar_t* flag : {L"--help", L"-h", L"/?"}) {
        std::vector<wchar_t*> argv{const_cast<wchar_t*>(L"bridge.exe"),
                                   const_cast<wchar_t*>(flag)};
        Options     opt{};
        std::string error{};
        REQUIRE(parseArgs(2, argv.data(), opt, error));
        CHECK(opt.help);
    }
}

TEST_CASE("parseArgs rejects an unknown argument by name")
{
    const Parse p{kPort, kCom6, kShm, kName, L"--marker-inteval", L"10"};
    CHECK_FALSE(p.ok);
    // The typo itself has to reach the message, or the operator is left
    // guessing which of a dozen options was misspelled.
    CHECK(p.error.find("--marker-inteval") != std::string::npos);
}

TEST_CASE("parseArgs rejects an option with no value")
{
    // Every value-taking option, each as the final argument.
    for (const wchar_t* flag : {L"--port", L"--shm-name", L"--event-name",
                                L"--baud", L"--channels", L"--marker-interval",
                                L"--log-name", L"--log-source"}) {
        const Parse p{kPort, kCom6, kShm, kName, flag};
        CAPTURE(narrow(flag));
        CHECK_FALSE(p.ok);
        CHECK_FALSE(p.error.empty());
    }
}

TEST_CASE("parseArgs rejects out-of-range numeric values")
{
    // A zero baud would divide by zero in the wire budget.
    CHECK_FALSE(Parse{kPort, kCom6, kShm, kName, L"--baud", L"0"}.ok);
    CHECK_FALSE(Parse{kPort, kCom6, kShm, kName, L"--baud", L"-9600"}.ok);
    // Above INT32_MAX, since Options::baud is an int.
    CHECK_FALSE(Parse{kPort, kCom6, kShm, kName, L"--baud", L"3000000000"}.ok);
    // A zero marker interval would mean a marker before every frame.
    CHECK_FALSE(Parse{kPort, kCom6, kShm, kName, L"--marker-interval", L"0"}.ok);
    // shmlog source ids are a byte.
    CHECK_FALSE(Parse{kPort, kCom6, kShm, kName, L"--log-source", L"256"}.ok);
    CHECK(Parse{kPort, kCom6, kShm, kName, L"--log-source", L"255"}.ok);
}

TEST_CASE("parseArgs takes the last value when an option repeats")
{
    // Not an error, but it must be the last one that wins - a wrapper script
    // appending an override after a base command line depends on it.
    const Parse p{kPort, kCom6, kShm, kName, L"--baud", L"9600", L"--baud", L"115200"};
    REQUIRE(p.ok);
    CHECK(p.opt.baud == 115200);
}

TEST_CASE("narrow substitutes for non-ASCII and tolerates a null pointer")
{
    CHECK(narrow(L"COM6") == "COM6");
    CHECK(narrow(L"") == "");
    // The bridge logs the event name on the polling fallback, where the raw
    // pointer is null; narrowing that must not dereference it.
    CHECK(narrow(nullptr) == "");
    // Outside ASCII, one '?' per wide character.
    CHECK(narrow(L"COMÜ") == "COM?");
    CHECK(narrow(L"äöü") == "???");
}
