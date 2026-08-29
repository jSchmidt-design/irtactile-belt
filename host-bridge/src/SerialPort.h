#pragma once
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>

#include "WireParams.h"

#include <cstddef>
#include <cstdint>
#include <string>

// Blocking Win32 serial writer, tuned for latency:
//  - a TX queue of kTxQueueBytes (WireParams.h), capping driver-side backlog
//    at ~2 ms of wire time at 1.2 Mbaud;
//  - a 50 ms budget per write() call, bounding how long a wedged or crawling
//    port blocks the caller;
//  - DTR/RTS held deasserted. On a CH340 devkit those two lines drive the
//    auto-reset transistors into EN/IO0, so asserting them reboots the ESP32.
//
// Never logs. Failures are reported through the return value plus
// lastFailure()/lastError().
class SerialPort {
public:
    // Per-call write budget, in ms. Bounds both the driver's timeout for one
    // WriteFile and the total time write() may spend on a buffer - the two are
    // not the same bound, because write() retries partial writes and each retry
    // restarts the driver's timeout.
    static constexpr DWORD kWriteTimeoutMs{50};

    // TimedOut is the driver accepting no bytes, or too few to finish the
    // buffer within kWriteTimeoutMs. The underlying call succeeds, so
    // lastError() reads 0. Part of the buffer may already be on the wire.
    enum class WriteStatus { Ok, Failed, TimedOut };

    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Opens and configures the port (8N1, given baud), closing any port already
    // open. On failure lastFailure() names the call that failed.
    [[nodiscard]] bool open(const std::string& port, int baudrate);
    void close() noexcept;

    [[nodiscard]] bool isOpen() const noexcept { return m_hSerial != INVALID_HANDLE_VALUE; }

    // Writes all `length` bytes, looping on partial writes.
    [[nodiscard]] WriteStatus write(const uint8_t* data, size_t length);

    // Bytes still sitting in the driver's TX queue - a direct measure of
    // accumulated wire latency. 0 when closed or on query failure.
    [[nodiscard]] uint32_t pendingTxBytes() const noexcept;

    // The Win32 call behind the last failure, and its GetLastError(). Both hold
    // until the next failure; "none" / 0 when nothing has failed.
    [[nodiscard]] const char* lastFailure() const noexcept { return m_lastFailure; }
    [[nodiscard]] DWORD lastError() const noexcept { return m_lastError; }

private:
    // Records `what` and GetLastError() as the last failure, and returns false.
    bool fail(const char* what) noexcept;

    HANDLE m_hSerial{INVALID_HANDLE_VALUE};
    const char* m_lastFailure{"none"};
    DWORD m_lastError{0};
};
