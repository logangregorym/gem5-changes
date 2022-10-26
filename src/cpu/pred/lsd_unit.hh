#ifndef __CPU_PRED_LSD_UNIT_HH
#define __CPU_PRED_LSD_UNIT_HH

#include "base/types.hh"
#include "params/LoopStreamDetector.hh"
#include "sim/sim_object.hh"

#include <deque>

class LSDUnit : public SimObject
{
    public:
        typedef LoopStreamDetectorParams Params;

        LSDUnit(const Params *params);

        void update(Addr pc, Addr upc);

        uint8_t lsdLength;

    private:
        struct lsd_elem {
            Addr pc;
            Addr upc;
            uint8_t repeat_count;
            uint8_t repeat_dist;
        };
        std::deque<struct lsd_elem> lsd;

};

#endif
