#ifndef SAMPLE_RING_H
#define SAMPLE_RING_H

#include <stdint.h>

// 512 entries x 4 B = 2 KB. Power of two so the wrap is a mask.
#define RING_SIZE 512
#define RING_MASK (RING_SIZE - 1)

#if defined(__GNUC__)
#define RING_LOAD_ACQ(v) __atomic_load_n(&(v), __ATOMIC_ACQUIRE)
#define RING_STORE_REL(v, x) __atomic_store_n(&(v), (x), __ATOMIC_RELEASE)
#define RING_INC_RELAXED(v) __atomic_fetch_add(&(v), 1, __ATOMIC_RELAXED)
#define RING_LOAD_RELAXED(v) __atomic_load_n(&(v), __ATOMIC_RELAXED)
#else
// Host test builds only - single-threaded there.
#define RING_LOAD_ACQ(v) (v)
#define RING_STORE_REL(v, x) ((v) = (x))
#define RING_INC_RELAXED(v) ((v)++)
#define RING_LOAD_RELAXED(v) (v)
#endif

typedef struct {
  uint16_t ch0 = 0;
  uint16_t ch1 = 0;
} dac_sample_t;

// Lock-free single-producer / single-consumer ring.
// Producer: serialTask (via the decoder callback). Consumer: dacTask.
// A full ring drops the incoming sample and never blocks the producer.
class SampleRing {
public:
  SampleRing() : m_head(0), m_tail(0), m_dropped(0) {}

  // Producer side.
  bool push(const dac_sample_t &s) {
    const uint32_t head = m_head;  // only the producer writes head
    const uint32_t next = (head + 1) & RING_MASK;
    if (next == RING_LOAD_ACQ(m_tail)) {
      RING_INC_RELAXED(m_dropped);
      return false;
    }
    m_buf[head] = s;
    RING_STORE_REL(m_head, next);
    return true;
  }

  // Consumer side.
  bool pop(dac_sample_t &s) {
    const uint32_t tail = m_tail;  // only the consumer writes tail
    if (tail == RING_LOAD_ACQ(m_head)) return false;
    s = m_buf[tail];
    RING_STORE_REL(m_tail, (tail + 1) & RING_MASK);
    return true;
  }

  // Consumer side: throw away the n oldest entries (stale samples).
  uint32_t discard(uint32_t n) {
    const uint32_t avail = fill();
    if (n > avail) n = avail;
    RING_STORE_REL(m_tail, (m_tail + n) & RING_MASK);
    return n;
  }

  uint32_t fill() const {
    const uint32_t head = RING_LOAD_ACQ(m_head);
    const uint32_t tail = RING_LOAD_ACQ(m_tail);
    return (head - tail) & RING_MASK;
  }

  uint32_t dropped() const { return RING_LOAD_RELAXED(m_dropped); }

private:
  dac_sample_t m_buf[RING_SIZE];
  uint32_t m_head;
  uint32_t m_tail;
  uint32_t m_dropped;
};

#endif
