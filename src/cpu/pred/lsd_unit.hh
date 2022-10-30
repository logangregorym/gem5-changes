#ifndef __CPU_PRED_LSD_UNIT_HH
#define __CPU_PRED_LSD_UNIT_HH

#include "base/types.hh"
#include "params/LoopStreamDetector.hh"
#include "sim/sim_object.hh"

#include <deque>
#include <vector>

class LSDUnit : public SimObject
{
    public:
        typedef LoopStreamDetectorParams Params;

        LSDUnit(const Params *params);
    
    public:
        struct lsd_elem {
            Addr pc;
            Addr upc;
            uint32_t repeat_count;
            uint8_t repeat_dist;
            bool is_head;
        };
        struct loop_info {
            Addr start_pc;
            Addr start_upc;
            Addr end_pc;
            Addr end_upc;
            uint32_t cur_iter;
        };

    private:
        const uint8_t lsdLength;
        std::deque<struct lsd_elem> lsd;
        struct loop_info cur_loop;
    
    public:
        struct loop_info getLoopInfo();
        bool inLoop();
        bool isHead();
        std::string printLoopElems();
        void update(Addr pc, Addr upc);

    private:
        void clearLoopInfo();
        std::vector<struct lsd_elem> getLoopElems();
};

#endif
