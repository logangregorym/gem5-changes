#include "cpu/pred/lsd_unit.hh"

#include <algorithm>
#include <iterator>
#include <string>
#include <ios>

#include "debug/LSD.hh"

LSDUnit::LSDUnit(const Params *params)
    : SimObject(params),
      lsdLength(params->lsdLength)
{
    lsd = std::deque<struct lsd_elem>();
    clearLoopInfo();
}

void
LSDUnit::clearLoopInfo()
{
    cur_loop = {0, 0, 0, 0, 0};
}

std::vector<struct LSDUnit::lsd_elem>
LSDUnit::getLoopElems()
{
    std::vector<struct LSDUnit::lsd_elem> loop_elems;

    if (lsd.empty()) {
        return loop_elems;
    }

    auto front = lsd.begin();
    if (!front->is_head) {
        return loop_elems;
    }

    auto cur = std::next(front, front->repeat_dist);
    if (cur->pc != front->pc || cur->upc != front->upc) {
        return loop_elems;
    }

    while (cur != front && cur->repeat_dist == front->repeat_dist) {
        loop_elems.push_back(*cur);
        std::advance(cur, -1);
    }

    if (cur != front) {
        loop_elems.clear();
    }

    return loop_elems;
}

struct LSDUnit::loop_info
LSDUnit::getLoopInfo()
{
    return cur_loop;
}

bool
LSDUnit::inLoop()
{
    if (lsd.empty()) {
        return false;
    }

    auto cur = lsd.front();
    if (cur.pc >= cur_loop.start_pc && cur.pc <= cur_loop.end_pc &&
        cur.upc >= cur_loop.start_upc && cur.upc <= cur_loop.end_upc) {
        
        //increment cur_iter if at the head of a loop
        if (cur.is_head) {
            cur_loop.cur_iter++;
        }
        return true;
    } else {
        clearLoopInfo();
    }

    std::vector<struct LSDUnit::lsd_elem> loop_elems = getLoopElems();

    //not a loop
    if (loop_elems.empty()) return false;

    //update cur_loop with current loop info
    auto front = loop_elems.front();
    auto back = loop_elems.back();
    cur_loop.start_pc = front.pc;
    cur_loop.start_upc = front.upc;
    cur_loop.end_pc = back.pc;
    cur_loop.end_upc = back.upc;
    /*
        Since loop_elems will be from the second iteration
        of the loop and we are just now starting the third
        iteration, cur_iter will always be 3 here.
    */
    cur_loop.cur_iter = 3;

    return true;
}

bool
LSDUnit::isHead()
{
    return (!lsd.empty() && lsd.front().is_head);
}

std::string
LSDUnit::printLoopElems()
{
    auto loop_elems = getLoopElems();
    std::stringstream str;
    if (!loop_elems.empty()) {
        str << "LOOP DETECTED - BEGIN" << std::endl;
        auto loop_iter = loop_elems.begin();
        do {
            str << "pc: 0x" << std::hex << loop_iter->pc << ", ";
            str << "upc: 0x" << std::hex << loop_iter->upc << std::endl;
            std::advance(loop_iter, 1);
        } while(loop_iter != loop_elems.end());

        str << "LOOP DETECTED - END" << std::endl;
    }
    return str.str();
}

void
LSDUnit::update(Addr pc, Addr upc)
{
    struct lsd_elem cur = {pc, upc, 0, 0, false};

    // try to find pc/upc pair in instruction history
    auto lsd_iter = lsd.begin();
    while (lsd_iter != lsd.end() && (lsd_iter->pc != cur.pc || lsd_iter->upc != cur.upc)) {
        std::advance(lsd_iter, 1);
    }

    if (lsd_iter != lsd.end()) {
        /*
            pc/upc pair found in instruction history.
            Determine if the history distance is repeating.
            +1 since we will be adding cur to front of lsd
            and that will be the true distance.
        */
        auto cur_dist = std::distance(lsd.begin(), lsd_iter) + 1;
        if (lsd_iter->repeat_dist == cur_dist) {
            cur.repeat_count = lsd_iter->repeat_count + 1;

            // if first instruction to repeat, mark as head of repetition
            if (lsd.front().repeat_count == (cur.repeat_count - 1)) {
                cur.is_head = true;
            }
        }
        cur.repeat_dist = cur_dist;
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