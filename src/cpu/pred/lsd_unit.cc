#include "cpu/pred/lsd_unit.hh"

#include <algorithm>
#include <iterator>
#include <string>

#include "debug/LSD.hh"

LSDUnit::LSDUnit(const Params *params)
    : SimObject(params),
      LSD_LENGTH(params->lsdLength)
{
    lsd = std::deque<struct lsdElem>();
    loop_active = false;
}

std::string
LSDUnit::generateLoopElemsStr()
{
    assert(!lsd.empty() && inLoop() && isLoopHead());
    auto head = lsd.begin();
    auto loop_iter = std::next(head, head->repeat_dist);
    std::stringstream str;
    while (loop_iter != head) {
        str << loop_iter->addr.str() << std::endl;
        std::advance(loop_iter, -1);
    }
    return str.str();
}

struct loopInfo
LSDUnit::getLoopInfo()
{
    assert(!lsd.empty() && inLoop() && isLoopHead());
    return cur_loop;
}

uint32_t
LSDUnit::getLoopIteration() {
    assert(!lsd.empty() && inLoop() &&
           cur_loop.iteration >= FIRST_OFFICIAL_ITERATION);
    return cur_loop.iteration;
}

bool
LSDUnit::inLoop()
{
    assert(!lsd.empty());
    return (loop_active && cur_loop.containsAddr(lsd.front().addr));
}

bool
LSDUnit::isFirstOfficialIteration()
{
    return (getLoopIteration() == FIRST_OFFICIAL_ITERATION);
}

bool
LSDUnit::isLoopHead()
{
    assert(!lsd.empty());
    return lsd.front().is_loop_head;
}

void
LSDUnit::update(struct fullAddr cur_addr)
{
    struct lsdElem cur (cur_addr);
    auto lsd_size = lsd.size();

    if (lsd_size > 0) {
        // try to find addr in instruction history
        auto lsd_iter = lsd.begin();
        const uint8_t start_distance = lsd_iter->repeat_dist;
        const uint32_t start_count = lsd_iter->repeat_count;
        bool potential_loop_head = (start_distance > 0);

        uint8_t cur_distance = 1;

        do {
            // if all repeat_dist and repeat_count values are consistent until
            // repeated addr, cur_addr is a loop head
            if (potential_loop_head && (lsd_iter->repeat_dist != start_distance ||
                lsd_iter->repeat_count != start_count)) {
                potential_loop_head = false;
            }
            if (lsd_iter->addr == cur_addr) {
                break;
            }
            cur_distance++;
            std::advance(lsd_iter, 1);
        } while (lsd_iter != lsd.end());

        // addr found in instruction history.
        if (cur_distance > 1 && cur_distance <= lsd_size) {
            cur.repeat_dist = cur_distance;

            // addr is showing a consistent repeat distance
            if (lsd_iter->repeat_dist == cur.repeat_dist) {
                cur.repeat_count = lsd_iter->repeat_count + 1;

                if (potential_loop_head) {
                    cur.is_loop_head = true;

                    if (loop_active && cur_loop.containsAddr(cur_addr)) {
                        cur_loop.iteration++;
                    } else {
                        cur_loop.start_addr = cur_addr;
                        cur_loop.end_addr = lsd.front().addr;
                        cur_loop.iteration = FIRST_OFFICIAL_ITERATION;
                        loop_active = true;
                    }
                }
            } else {
                loop_active = false;
            }
        } else {
            loop_active = false;
        }

        if (lsd_size == LSD_LENGTH) {
            lsd.pop_back();
        }
    }

    lsd.push_front(cur);
}

LSDUnit*
LoopStreamDetectorParams::create()
{
    return new LSDUnit(this);
}