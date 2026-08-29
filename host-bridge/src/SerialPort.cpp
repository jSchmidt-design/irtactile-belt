#include "SerialPort.h"

#include <algorithm>
#include <chrono>
#include <limits>

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::fail(const char* what) noexcept
{
    m_lastError = GetLastError();
    m_lastFailure = what;
    return false;
}

bool SerialPort::open(const std::string& port, int baudrate)
{
    close();

    // The \\.\ prefix makes COM10 and above addressable.
    const std::string portName{"\\\\.\\" + port};
    m_hSerial = CreateFileA(portName.c_str(),
        GENERIC_WRITE,
        0,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);

    if (m_hSerial == INVALID_HANDLE_VALUE) {
        return fail("CreateFile");
    }

    // Sized for latency, not throughput - see kTxQueueBytes (WireParams.h).
    SetupComm(m_hSerial, kTxQueueBytes, kTxQueueBytes);

    DCB dcbSerialParams{};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(m_hSerial, &dcbSerialParams)) {
        const bool r = fail("GetCommState");
        close();
        return r;
    }

    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    // On an ESP32 devkit DTR and RTS drive the auto-reset circuit; asserting
    // them on open reboots the target.
    dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(m_hSerial, &dcbSerialParams)) {
        const bool r = fail("SetCommState");
        close();
        return r;
    }

    // Bound one WriteFile, so a wedged port cannot block the shm consumer
    // indefinitely. write() applies the same budget across its retry loop.
    COMMTIMEOUTS timeouts{};
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant   = kWriteTimeoutMs;
    if (!SetCommTimeouts(m_hSerial, &timeouts)) {
        const bool r = fail("SetCommTimeouts");
        close();
        return r;
    }

    m_lastFailure = "none";
    m_lastError = 0;
    return true;
}

void SerialPort::close() noexcept
{
    if (m_hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(m_hSerial);
        m_hSerial = INVALID_HANDLE_VALUE;
    }
}

SerialPort::WriteStatus SerialPort::write(const uint8_t* data, size_t length)
{
    if (m_hSerial == INVALID_HANDLE_VALUE) {
        m_lastFailure = "port closed";
        m_lastError = ERROR_INVALID_HANDLE;
        return WriteStatus::Failed;
    }

    size_t offset{0};
    const auto deadline{std::chrono::steady_clock::now()
        + std::chrono::milliseconds{kWriteTimeoutMs}};

    while (offset < length) {
        const DWORD chunk{static_cast<DWORD>(std::min<size_t>(
            length - offset, (std::numeric_limits<DWORD>::max)()))};
        DWORD bytesWritten{0};
        if (!WriteFile(m_hSerial, data + offset, chunk, &bytesWritten, nullptr)) {
            fail("WriteFile");
            return WriteStatus::Failed;
        }
        offset += bytesWritten;

        // A synchronous WriteFile that hits WriteTotalTimeoutConstant returns
        // TRUE having written less than asked, so a short write is the driver
        // signalling backpressure, not an error. Retrying is right - the queue
        // drains at line rate - but each retry restarts the driver's timeout,
        // so the loop carries its own deadline: without one, a port accepting a
        // byte per timeout would hold the drain for seconds while never
        // reporting a failure. Zero progress folds into the same branch, since
        // retrying that would spin unbounded on a dead port. The error code is
        // cleared because the call itself succeeded.
        if (offset < length
            && (bytesWritten == 0 || std::chrono::steady_clock::now() >= deadline)) {
            m_lastFailure = "write timed out";
            m_lastError = 0;
            return WriteStatus::TimedOut;
        }
    }

    return WriteStatus::Ok;
}

uint32_t SerialPort::pendingTxBytes() const noexcept
{
    if (m_hSerial == INVALID_HANDLE_VALUE) return 0;

    DWORD   errors{0};
    COMSTAT status{};
    if (!ClearCommError(m_hSerial, &errors, &status)) return 0;
    return static_cast<uint32_t>(status.cbOutQue);
}
