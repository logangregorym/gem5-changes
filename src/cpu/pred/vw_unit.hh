#ifndef __CPU_PRED_VW_UNIT_HH
#define __CPU_PRED_VW_UNIT_HH

#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "cpu/pred/lsd_unit.hh"
#include "params/VectorWideningUnit.hh"
#include "sim/sim_object.hh"

class VWUnit : public SimObject
{
    public:
        typedef VectorWideningUnitParams Params;

        VWUnit(const Params *params);

        enum State {
            ANALYZE,
            TRANSFORM,
            INACTIVE
        };

        void clear();
        void processInst(fullAddr addr, StaticInstPtr &inst, uint32_t iteration);
        void reset(struct fullAddr start_addr, struct fullAddr end_addr);
    
    private:
        struct fullAddr start_addr;
        struct fullAddr end_addr;
        State state;
        uint32_t transform_iteration;

        void transformVecInst(StaticInstPtr &inst, uint32_t iteration);
};

#endif