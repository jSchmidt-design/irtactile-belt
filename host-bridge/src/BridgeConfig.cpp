#include "BridgeConfig.h"

#include <cerrno>
#include <cstdio>
#include <cstdint>   // UINT32_MAX, INT32_MAX
#include <cwchar>
#include <string_view>

void printUsage()
{
    std::printf(
        "irTactileSerialBridge - forward an irTactile shm stream to a serial port\n"
        "\n"
        "Usage:\n"
        "  irTactileSerialBridge --port <COMx> --shm-name <name> [options]\n"
        "\n"
        "Required:\n"
        "  --port <COMx>            serial port to write to\n"
        "  --shm-name <name>        resolved shm section name (incl. size-class\n"
        "                           suffix, e.g. irTactile_Stream_Primary_256k)\n"
        "Options:\n"
        "  --event-name <name>      resolved event name; omit for polling mode\n"
        "  --baud <n>               baud rate (default 1200000)\n"
        "  --channels <a,b>         stream channels feeding protocol slots 1/2\n"
        "                           (default 0,1)\n"
        "  --marker-interval <n>    frames between sync markers; default is\n"
        "                           derived from the stream rate for a ~10 Hz\n"
        "                           marker cadence\n"
        "  --legacy-marker          emit the bare 4x0xFF marker with no rate\n"
        "                           header, for firmware that cannot read it\n"
        "  --log-name <name>        shmlog partition name (default %s);\n"
        "                           give each concurrent bridge its own\n"
        "  --log-source <id>        shmlog source id 0..255 (default 0)\n"
        "  --help                   this text\n",
        kDefaultLogName);
}

std::string narrow(const wchar_t* wide)
{
    std::string out{};
    if (!wide) return out;
    for (const wchar_t* p{wide}; *p; ++p)
        out.push_back(*p <= 0x7F ? static_cast<char>(*p) : '?');
    return out;
}

bool parseUInt(const wchar_t* s, uint32_t& out)
{
    if (!s || !*s) return false;
    if (*s == L'-') return false;   // wcstoul wraps a negative into the range
    errno = 0;
    wchar_t* end{nullptr};
    const unsigned long v{std::wcstoul(s, &end, 10)};
    if (!end || *end != L'\0' || errno == ERANGE) return false;
    // ERANGE alone is not the range check: unsigned long is 32 bits under
    // MSVC but 64 on an LP64 host, where 4294967296 parses cleanly and would
    // truncate to 0. This parser is meant to be testable off Windows, so the
    // bound is asserted rather than inherited from the type.
    if (v > UINT32_MAX) return false;
    out = static_cast<uint32_t>(v);
    return true;
}

bool parseChannels(const wchar_t* s, uint32_t& a, uint32_t& b)
{
    if (!s) return false;
    const std::wstring_view sv{s};
    const size_t comma{sv.find(L',')};
    if (comma == std::wstring_view::npos) return false;
    const std::wstring sa{sv.substr(0, comma)};
    const std::wstring sb{sv.substr(comma + 1)};
    return parseUInt(sa.c_str(), a) && parseUInt(sb.c_str(), b) && a != b;
}

bool parseArgs(int argc, wchar_t** argv, Options& opt, std::string& error)
{
    // Names the option that failed, so the message points at the argument the
    // user typed rather than at the value the parser choked on.
    auto reject{[&error](const char* what, std::string_view detail = {}) {
        error = what;
        if (!detail.empty()) {
            error += ": ";
            error += detail;
        }
        return false;
    }};

    for (int i{1}; i < argc; ++i) {
        const std::wstring_view arg{argv[i]};
        auto next{[&]() -> const wchar_t* { return i + 1 < argc ? argv[++i] : nullptr; }};

        if (arg == L"--help" || arg == L"-h" || arg == L"/?") {
            opt.help = true;
        } else if (arg == L"--port") {
            const wchar_t* v{next()};
            if (!v || !*v) return reject("--port needs a value");
            opt.port = narrow(v);
        } else if (arg == L"--shm-name") {
            const wchar_t* v{next()};
            if (!v || !*v) return reject("--shm-name needs a value");
            opt.shmName = v;
        } else if (arg == L"--event-name") {
            // The only value option that accepts an empty string: a profile
            // with "eventName": "" means polling, so a wrapper handing the
            // profile through verbatim selects polling rather than failing.
            const wchar_t* v{next()};
            if (!v) return reject("--event-name needs a value");
            opt.eventName = v;
        } else if (arg == L"--baud") {
            uint32_t v{0};
            if (!parseUInt(next(), v) || v == 0 || v > INT32_MAX)
                return reject("--baud needs a positive integer");
            opt.baud = static_cast<int>(v);
        } else if (arg == L"--channels") {
            if (!parseChannels(next(), opt.channelA, opt.channelB))
                return reject("--channels needs two distinct indices, e.g. 0,1");
        } else if (arg == L"--marker-interval") {
            uint32_t v{0};
            if (!parseUInt(next(), v) || v == 0)
                return reject("--marker-interval needs a positive integer");
            opt.markerInterval = v;
        } else if (arg == L"--legacy-marker") {
            opt.legacyMarker = true;
        } else if (arg == L"--log-name") {
            const wchar_t* v{next()};
            if (!v || !*v) return reject("--log-name needs a value");
            opt.logName = narrow(v);
        } else if (arg == L"--log-source") {
            uint32_t v{0};
            if (!parseUInt(next(), v) || v > 255)
                return reject("--log-source needs an id in 0..255");
            opt.logSourceId = static_cast<uint8_t>(v);
        } else {
            return reject("unknown argument", narrow(argv[i]));
        }
    }

    if (opt.help) return true;
    if (opt.port.empty())    return reject("--port is required");
    if (opt.shmName.empty()) return reject("--shm-name is required");
    return true;
}
