/*
 * Will have to ask what I'm supposed to put here
 */

#ifndef __CPU_PRED_BIG_SAT_COUNTER_HH__
#define __CPU_PRED_BIG_SAT_COUNTER_HH__

/**
 * Version of SatCounter with a larger base type
 */
class BigSatCounter
{
  public:
    /**
     * Default constructor.
     */
    BigSatCounter() : initialVal(0), counter(0) {}

    /**
     * Constructor with size.
     * @param bits How many bits the counter will have.
     */
    BigSatCounter(unsigned bits) : initialVal(0), maxVal((1 << bits) - 1), counter(0) {}

    /**
     * Constructor with size and start value.
     * @param bits How many bits the counter will have.
     * @param initial_val Starting value for each counter.
     */
    BigSatCounter(unsigned bits, uint64_t initial_val)
        : initialVal(initial_val), maxVal((1 << bits) - 1), counter(initial_val)
    {
        if (initial_val > maxVal) {
            fatal("BP: Initial counter value exceeds max size.");
        }
    }

    /**
     * Sets the number of bits.
     */
    void setBits(unsigned bits) { maxVal = (1 << bits) - 1; }

    void reset() { counter = initialVal; }

    /**
     * Increments the counter's current value.
     */
    void increment()
    {
        if (counter < maxVal) {
            ++counter;
        }
    }

    /**
     * Decrements the counter's current value.
     */
    void decrement()
    {
        if (counter > 0) {
            --counter;
        }
    }

    /**
     * Read the counter's value.
     */
    uint64_t read() const { return counter; }

  private:
    uint64_t initialVal;
    uint64_t maxVal;
    uint64_t counter;
};

#endif // __CPU_PRED_SAT_COUNTER_HH__
