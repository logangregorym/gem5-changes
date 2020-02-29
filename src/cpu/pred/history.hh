/**
 * Will have to ask what I'm supposed to put here lol
 */

#ifndef __CPU_PRED_HISTORY_HH__
#define __CPU_PRED_HISTORY_HH__

// #include "base/misc.hh"
#include "base/types.hh"

/**
 * Similar to sat_counter.hh, implements history update logic
 * History length and bits per entry set at construction time
 * Calls to this object look the same regardless of internal size
 */
class History
{
  public:
    /**
     * Default constructor.
     */
    History()
        : entrySize(8), historyLength(4), history(0),
          totalSize(32), entryMask((1 << 8) - 1), historyMask((uint64_t(1) << 32) - 1)
    {}
    /**
     * Constructor with dimensions.
     * @param bits Number of bits per entry.
     * @param length Number of entries.
     */
    History(unsigned bits, unsigned length)
        : entrySize(bits), historyLength(length), history(0),
          totalSize(bits*length), entryMask((1 << bits) - 1), historyMask((uint64_t(1) << (bits*length)) - 1)
    {}
    /**
     * Constructor with dimensions and start value.
     * @param bits Number of bits per entry.
     * @param length Number of entries.
     * @param val Initial value.
     */
    History(unsigned bits, unsigned length, unsigned val)
        : entrySize(bits), historyLength(length), history(val),
          totalSize(bits*length), entryMask((1 << bits) - 1), historyMask((uint64_t(1) << (bits*length)) - 1)
    {}

    /**
     * Read the current history.
     */
    uint64_t read() { return history; }

    /**
     * Update the history with new entry.
     */
    void update(unsigned newVal) {
        history = ((history << entrySize) & historyMask) | (newVal & entryMask);
    }

    /**
     * Reset history to 0
     */
    void reset() {
        history = 0;
    }

  private:
    uint8_t entrySize;
    uint8_t historyLength;
    uint64_t history; // Consider re-sizing
    uint8_t totalSize;
    unsigned entryMask;
    unsigned historyMask; // Consider re-sizing
};

#endif //__CPU_PRED_HISTORY_HH__
