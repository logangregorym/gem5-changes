#ifndef __CPU_PRED_VW_UNIT_HH
#define __CPU_PRED_VW_UNIT_HH

#include "arch/x86/registers.hh"
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
        void processInst(fullAddr addr, StaticInstPtr &inst, uint32_t iteration, bool &skip, bool &branch_pred);
        void reset(struct fullAddr start_addr, struct fullAddr end_addr);
        Addr getEndPC();
    
    private:
        struct fullAddr start_addr;
        struct fullAddr end_addr;
        State state;
        uint32_t transform_iteration;
        uint8_t cur_loop_insts;
        const uint32_t MAX_LOOP_INSTS;
        RegIndex vector_stride_reg;
        uint8_t vector_stride_seen;
        bool vector_instruction_seen;

        static const RegIndex VEC_REG_BASE = X86ISA::NumMMXRegs;
        static const RegIndex NUM_VEC_REGS = X86ISA::NumXMMRegs * X86ISA::NumXMMSubRegs + X86ISA::NumMicroFpRegs;
        static const RegIndex NUM_INT_REGS = X86ISA::NumIntRegs;
        static const uint8_t VECTOR_STRIDE = 16;

        std::array<bool, NUM_VEC_REGS> valid_vector_registers;
        std::array<bool, NUM_INT_REGS> validated_vector_stride_registers;
        std::array<bool, NUM_INT_REGS> required_vector_stride_registers;
        std::array<bool, NUM_INT_REGS> set_int_registers;
        std::unordered_set<std::string> supported_vector_insts = {"ldfp128", "stfp128", "vaddi"};

        void clear();
        bool analyzeInstRegisters(StaticInstPtr &inst);
        bool isVectorInstSupported(StaticInstPtr &inst);
        bool verifyVectorStrideRegisters();
        StaticInstPtr widenVecInst(StaticInstPtr &inst);

        bool isVectorRegister(RegIndex &reg_index) {
            return ((reg_index >= VEC_REG_BASE) && (reg_index < VEC_REG_BASE + NUM_VEC_REGS));
        };
};

#endif
