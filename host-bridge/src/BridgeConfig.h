#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

// The command line: the Options the bridge runs from, and the parsing that
// fills them.
//
// Free of irtactile-shm, shmlog and windows.h, so the parser is testable
// natively. parseArgs() reports a failure through `error` rather than printing
// it - argument errors happen before the logger exists, and where they go is
// the caller's decision, not the parser's.

/// Default shmlog partition name; a collector is configured against it. Two
/// bridges sharing one partition interleave under a single source id.
inline constexpr const char* kDefaultLogName{"irTactile_Belt_Bridge_Log"};

struct Options {
    std::wstring shmName{};          // required; resolved section name
    std::wstring eventName{};        // optional; empty => polling mode
    std::string  port{};             // required; e.g. "COM6"
    int          baud{1200000};
    uint32_t     channelA{0};
    uint32_t     channelB{1};
    size_t       markerInterval{0};  // 0 => derive from the stream rate
    bool         legacyMarker{false};
    std::string  logName{kDefaultLogName};
    uint8_t      logSourceId{0};
    bool         help{false};
};

/// Writes the usage text to stdout.
void printUsage();

/// Narrows a wide string, substituting '?' for anything outside ASCII. The
/// names passed through here (a port, a log partition) are ASCII. A null
/// pointer narrows to an empty string.
[[nodiscard]] std::string narrow(const wchar_t* wide);

/// Parses a complete decimal uint32. Rejects an empty string, a negative, a
/// trailing character and an out-of-range value.
[[nodiscard]] bool parseUInt(const wchar_t* s, uint32_t& out);

/// Parses "a,b" into two distinct channel indices.
[[nodiscard]] bool parseChannels(const wchar_t* s, uint32_t& a, uint32_t& b);

/// Fills `opt` from argv. Returns false on an unknown argument, a missing or
/// malformed value, or a missing required option, leaving a one-line
/// description in `error`. --help short-circuits the required-option check.
[[nodiscard]] bool parseArgs(int argc, wchar_t** argv, Options& opt, std::string& error);
