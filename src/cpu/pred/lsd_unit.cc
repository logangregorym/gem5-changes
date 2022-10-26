#include "cpu/pred/lsd_unit.hh"

#include <algorithm>
#include <iterator>

#include "debug/LSD.hh"

LSDUnit::LSDUnit(const Params *params)
    : SimObject(params),
      lsdLength(params->lsdLength)
{
    lsd = std::deque<struct lsd_elem>();
}

void LSDUnit::update(Addr pc, Addr upc) {
    struct lsd_elem cur;
    cur.pc = pc;
    cur.upc = upc;

    // try to find pc/upc pair in instruction history
    auto lsd_iter = lsd.begin();
    while (lsd_iter != lsd.end()) {
        if (lsd_iter->pc == cur.pc && lsd_iter->upc == cur.upc) break;
        lsd_iter++;
    }

    if (lsd_iter != lsd.end()) {
        // pc/upc pair found in instruction history
        // determine if the history distance is repeating
        auto cur_dist = std::distance(lsd.begin(), lsd_iter);
        if (lsd_iter->repeat_dist == cur_dist) {
            cur.repeat_count = lsd_iter->repeat_count + 1;
        } else {
            cur.repeat_count = 0;
        }
        cur.repeat_dist = cur_dist;
    } else {
        // pc/upc pair not found in instruction history
        cur.repeat_count = 0;
        cur.repeat_dist = 0;
    }

    if (cur.repeat_count > 1) {
        DPRINTF(LSD, "LOOP INSTRUCTION DETECTED: "
            "pc: 0x%lx, upc: 0x%lx, dist: %u, count: %u\n",
            cur.pc, cur.upc, cur.repeat_dist, cur.repeat_count);
    }

    // update the lsd deque with the new element
    if (lsd.size() == lsdLength) {
        lsd.pop_back();
    }
    lsd.push_front(cur);
}

LSDUnit*
LoopStreamDetectorParams::create()
{
    return new LSDUnit(this);
}