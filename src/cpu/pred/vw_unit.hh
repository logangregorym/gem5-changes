#ifndef __CPU_PRED_VW_UNIT_HH
#define __CPU_PRED_VW_UNIT_HH

#include "arch/x86/x86_traits.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "cpu/pred/lsd_unit.hh"
#include "params/VectorWideningUnit.hh"
#include "sim/sim_object.hh"

#include <unordered_set>
#include <array>

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

        void deactivate();
        void processInst(fullAddr addr, StaticInstPtr &inst, uint32_t iteration, bool &widen, bool &skip);
        void reset(struct fullAddr start_addr, struct fullAddr end_addr);
    
    private:
        struct fullAddr start_addr;
        struct fullAddr end_addr;
        State state;
        uint32_t transform_iteration;
        uint32_t cur_loop_insts;
        const uint32_t MAX_LOOP_INSTS;
        const static uint8_t NUM_XMM_REGS = X86ISA::NumXMMRegs * X86ISA::NumXMMSubRegs;
        const static uint8_t XMM_REG_BASE_IDX = X86ISA::NumMMXRegs;

        std::array<bool, VWUnit::NUM_XMM_REGS> validVectorRegisters;
        std::unordered_set<std::string> supportedVectorInsts = {"ldfp128", "stfp128", "vaddi"};

        void clear();
        bool isVectorInstSupported(StaticInstPtr &inst);
        bool isXMMReg(RegId &regId);
        uint8_t getXMMRelativeIndex(RegId &regId);
        bool analyzeVectorInstRegisters(StaticInstPtr &inst);
        StaticInstPtr widenVecInst(StaticInstPtr &inst);
};

#endif
