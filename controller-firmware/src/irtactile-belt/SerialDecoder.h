#ifndef SERIAL_DECODER_H
#define SERIAL_DECODER_H

#include <stdint.h>
#include <string.h>

#define MSG_LEN 5
#define MARKER_LEN 4
#define MARKER_BYTE 0xFF

// Header frame: the 4x0xFF sync is followed by 5 payload bytes.
//
//   FF FF FF FF   sync
//   0x01          frame type / version
//   rateCode      3..10, exponent on the ladder: rateMilliHz = 48000000 >> rateCode
//                 (the ladder itself is 0..10; see MIN_RATE_CODE for the floor)
//   blkLo          samplesPerBlock        & 0x7F
//   blkHi         (samplesPerBlock >> 7)  & 0x7F
//   chk           (type ^ rateCode ^ blkLo ^ blkHi) & 0x7F
//
// Every payload byte is <= 0x7F, so it can neither extend nor re-trigger a
// marker run. Sample data is masked to 0x7F in the high bytes upstream, so a
// run of four 0xFF cannot occur in data either - the marker is unambiguous.
//
// The run can however be *five*: the frame checksum is not masked, so a frame
// ending in a 0xFF checksum right before a marker makes it 0xFF x5, and a
// sliding four-byte window would match one byte early. So the leading run is
// consumed in full and the type is read from the first non-0xFF byte. The run
// always ends at the marker's fourth byte, so nothing after the header start
// needs skipping.
#define HEADER_LEN 5
#define HEADER_TYPE 0x01

// Second header type: the encoder force-curve tuning parameters, emitted by the
// belt-tune tool and never by the bridge. Same fixed 5-byte payload, same
// checksum shape (protocol.md).
//
//   FF FF FF FF   sync
//   0x02          frame type: tuning
//   fsLo           (fullScaleCounts / 2)        & 0x7F
//   fsHi          ((fullScaleCounts / 2) >> 7)  & 0x7F   -- 14 bits, 2-count units
//   gammaQ        gamma * 16, Q3.4
//   chk           (type ^ fsLo ^ fsHi ^ gammaQ) & 0x7F
//
// Payload bytes only have to be <= 0x7F here; forceCurveClamp() owns the ranges.
#define HEADER_TYPE_TUNING 0x02

// The ladder runs 0..10 (48 kHz down to 46.875 Hz); this firmware accepts only
// 3..10, i.e. 6 kHz and below.
//
// DAC_TICK_HZ is 6 kHz, so a stream above it makes the resampler a *decimator*:
// it would pop two or four samples per tick and throw all but the last away,
// with no anti-alias filter in the path and nothing in the PLAY line saying so.
// The bridge rejects codes 0-2 for the same reason and the README advertises
// 375-6000 Hz.
//
// Raising DAC_TICK_HZ to a higher rung is what lets this floor drop - keep the
// two in step.
#define MIN_RATE_CODE 3
#define MAX_RATE_CODE 10
#define RATE_BASE_MILLIHZ 48000000u

enum ReceiverState {
  WAIT_MARKER,
  READ_HEADER,
  PROCESS_MESSAGES
};

typedef void (*DataCallback)(uint16_t[2]);
typedef void (*ConfigCallback)(uint32_t rateMilliHz, uint16_t blockSamples);
typedef void (*TuningCallback)(uint16_t fullScaleCounts, uint8_t gammaQ);

class SerialDecoder {
public:

  SerialDecoder(DataCallback cb, ConfigCallback ccb = nullptr, TuningCallback tcb = nullptr)
    : m_callback(cb), m_configCallback(ccb), m_tuningCallback(tcb) {
    idx = 0;
    markerIdx = 0;
    hdrIdx = 0;
  }

  void setConfigCallback(ConfigCallback cb) { m_configCallback = cb; }
  void setTuningCallback(TuningCallback cb) { m_tuningCallback = cb; }

  void process(const uint8_t *rxBuf, int len) {
    uint16_t data[2];
    for (int k = 0; k < len; k++) {

      uint8_t byte = rxBuf[k];

      // Runs on every byte in every state, so a header arriving mid-message
      // still re-syncs us.
      markerBuffer[markerIdx++] = byte;
      if (markerIdx == MARKER_LEN) {
        bool isMarker = true;
        for (uint8_t i = 0; i < MARKER_LEN; i++) {
          if (markerBuffer[i] != MARKER_BYTE) {
            isMarker = false;
            break;
          }
        }

        if (isMarker) {
          // The next 5 bytes are the config header.
          state = READ_HEADER;
          hdrIdx = 0;
          idx = 0;
          markerIdx = 0;
          continue;  // consume marker
        } else {
          memmove(markerBuffer, markerBuffer + 1, MARKER_LEN - 1);
          markerIdx = MARKER_LEN - 1;
        }
      }

      if (state == READ_HEADER) {
        // Still inside the sync run - the type field can never be 0xFF.
        if (hdrIdx == 0 && byte == MARKER_BYTE) continue;

        hdrBuffer[hdrIdx++] = byte;
        if (hdrIdx == HEADER_LEN) {
          hdrIdx = 0;
          if (validateHeader()) {
            applyHeader();
            state = PROCESS_MESSAGES;
            idx = 0;
            m_headerOk++;
          } else {
            // Bad header: hunt for a marker again rather than corrupt message
            // phase. The marker buffer still holds the last 3 header bytes, so
            // the next sync is picked up normally.
            state = WAIT_MARKER;
            m_headerBad++;
          }
        }
        continue;
      }

      if (state != PROCESS_MESSAGES) continue;

      buffer[idx++] = byte;
      if (idx == MSG_LEN) {
        uint8_t checksum = buffer[0] ^ buffer[1] ^ buffer[2] ^ buffer[3];
        if (checksum == buffer[4]) {
          data[0] = ((uint16_t)buffer[1] << 8) | buffer[0];
          data[1] = ((uint16_t)buffer[3] << 8) | buffer[2];

          data[0] &= 0x7FFF;
          data[1] &= 0x7FFF;

          m_callback(data);

          idx = 0;
        } else {
          // Checksum failed: slide one byte and retry.
          memmove(buffer, buffer + 1, MSG_LEN - 1);
          idx = MSG_LEN - 1;
          m_msgBad++;
        }
      }
    }
  }

  bool synced() const { return state == PROCESS_MESSAGES; }
  uint32_t headersAccepted() const { return m_headerOk; }
  uint32_t headersRejected() const { return m_headerBad; }
  uint32_t checksumErrors() const { return m_msgBad; }

private:

  bool validateHeader() {
    // Checksum first - its shape is the same for every header type.
    uint8_t chk = (hdrBuffer[0] ^ hdrBuffer[1] ^ hdrBuffer[2] ^ hdrBuffer[3]) & 0x7F;
    if (chk != hdrBuffer[4]) return false;

    if (hdrBuffer[0] == HEADER_TYPE) {
      if (hdrBuffer[1] < MIN_RATE_CODE || hdrBuffer[1] > MAX_RATE_CODE) return false;
      if (hdrBuffer[2] > 0x7F || hdrBuffer[3] > 0x7F) return false;
      return true;
    }
    if (hdrBuffer[0] == HEADER_TYPE_TUNING) {
      // Just the payload-byte invariant; forceCurveClamp() owns the ranges.
      if (hdrBuffer[1] > 0x7F || hdrBuffer[2] > 0x7F || hdrBuffer[3] > 0x7F) return false;
      return true;
    }
    return false;
  }

  void applyHeader() {
    if (hdrBuffer[0] == HEADER_TYPE_TUNING) {
      // 14-bit field, reassembled the same way blockSamples is below, then
      // << 1 to undo the 2-count wire scaling. Max input is 0x3FFF, so
      // (0x3FFF << 1) == 32766 - no uint16_t overflow for any 14-bit value.
      uint16_t fullScaleCounts =
        ((uint16_t)hdrBuffer[1] | ((uint16_t)hdrBuffer[2] << 7)) << 1;
      uint8_t gammaQ = hdrBuffer[3];
      if (m_tuningCallback) m_tuningCallback(fullScaleCounts, gammaQ);
      return;
    }

    // Exact for all 11 ladder rates: no table, no float.
    uint32_t rateMilliHz = RATE_BASE_MILLIHZ >> hdrBuffer[1];
    uint16_t blockSamples = (uint16_t)hdrBuffer[2] | ((uint16_t)hdrBuffer[3] << 7);
    if (blockSamples == 0) blockSamples = 1;
    if (m_configCallback) m_configCallback(rateMilliHz, blockSamples);
  }

  ReceiverState state = WAIT_MARKER;
  uint8_t buffer[MSG_LEN];
  uint8_t idx;

  uint8_t markerBuffer[MARKER_LEN];
  uint8_t markerIdx;

  uint8_t hdrBuffer[HEADER_LEN];
  uint8_t hdrIdx;

  uint32_t m_headerOk = 0;
  uint32_t m_headerBad = 0;
  uint32_t m_msgBad = 0;

  DataCallback m_callback;
  ConfigCallback m_configCallback;
  TuningCallback m_tuningCallback;
};

#endif
