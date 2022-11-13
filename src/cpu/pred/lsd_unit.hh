#ifndef __CPU_PRED_LSD_UNIT_HH
#define __CPU_PRED_LSD_UNIT_HH

#include "base/types.hh"
#include "params/LoopStreamDetector.hh"
#include "sim/sim_object.hh"

#include <deque>
#include <iostream>
#include <vector>

struct fullAddr {
    Addr pc;
    Addr upc;

    fullAddr() : pc(0), upc(0) {}

    fullAddr(const struct fullAddr &addr) {
        this->pc = addr.pc;
        this->upc = addr.upc;
    }

    fullAddr(Addr pc, Addr upc) : pc(pc), upc(upc) {}

    bool operator==(const struct fullAddr &addr) {
        return (pc == addr.pc && upc == addr.upc);
    }

    bool operator!=(const struct fullAddr &addr) {
        return !(*this == addr);
    }

    bool operator<(const struct fullAddr &addr) {
        return ((pc < addr.pc) || (pc == addr.pc && upc < addr.upc));
    }

    bool operator<=(const struct fullAddr &addr) {
        return ((*this < addr) || (*this == addr));
    }
    
    bool operator>(const struct fullAddr &addr) {
        return ((pc > addr.pc) || (pc == addr.pc && upc > addr.upc));
    }

    bool operator>=(const struct fullAddr &addr) {
        return ((*this > addr) || (*this == addr));
    }

    std::string str() {
        std::stringstream str;
        str << "pc: 0x" << std::hex << pc << ", ";
        str << "upc: 0x" << std::hex << upc;
        return str.str();
    }
};

struct lsdElem {
    struct fullAddr addr;
    uint32_t repeat_count;
    uint8_t repeat_dist;
    bool is_loop_head;

    lsdElem(struct fullAddr _addr) : addr(_addr), repeat_count(0),
             repeat_dist(0), is_loop_head(false) {}
};

struct loopInfo {
    fullAddr start_addr;
    fullAddr end_addr;
    uint32_t iteration;

    void clear() {
        start_addr = fullAddr();
        end_addr = fullAddr();
        iteration = 0;
    }

    bool containsAddr(struct fullAddr addr) {
        return (addr >= start_addr && addr <= end_addr);
    }
};

class LSDUnit : public SimObject
{
    public:
        typedef LoopStreamDetectorParams Params;

        LSDUnit(const Params *params);

        std::string generateLoopElemsStr();
        struct loopInfo getLoopInfo();
        uint32_t getLoopIteration();
        bool inLoop();
        bool isFirstOfficialIteration();
        bool isLoopHead();
        void update(struct fullAddr addr);
    
    private:
        // This detector officially detects loops at
        // the start of the third loop iteration.
        const uint32_t FIRST_OFFICIAL_ITERATION = 3;
        const uint8_t LSD_LENGTH;
        std::deque<struct lsdElem> lsd;
        struct loopInfo cur_loop;
        bool loop_active;
};

#endif