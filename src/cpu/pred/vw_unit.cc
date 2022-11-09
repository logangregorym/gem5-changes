#include "arch/x86/generated/decoder.hh"
#include "cpu/pred/vw_unit.hh"

#include "debug/VW.hh"

VWUnit::VWUnit(const Params *params)
    : SimObject(params), state(INACTIVE), MAX_LOOP_INSTS(params->maxLoopInsts)
{}

void
VWUnit::clear()
{
    this->transform_iteration = 0;
    this->cur_loop_insts = 0;
    this->validVectorRegisters.fill(false);
}

void VWUnit::deactivate()
{
    this->state = INACTIVE;
}

bool
VWUnit::isVectorInstSupported(StaticInstPtr &inst)
{
    std::string mnemonic = inst->mnemonic;
    return (this->supportedVectorInsts.find(mnemonic) != this->supportedVectorInsts.end());
}

bool
VWUnit::analyzeVectorInstRegisters(StaticInstPtr &inst)
{
    for (uint8_t i = 0; i < inst->numSrcRegs(); i++) {
        RegId regId = inst->srcRegIdx(i);
        DPRINTF(VW, "%s Src Reg #%u: %s\n", inst->mnemonic, i, regId);
        if (regId.isFloatReg()) {
            RegIndex regIndex = regId.index();
            if (regIndex >= XMM_REG_BASE_IDX && regIndex < (XMM_REG_BASE_IDX + NUM_XMM_REGS)) {
                regIndex -= XMM_REG_BASE_IDX;
                if (!validVectorRegisters[regIndex]) return false;
            }
        }
    }
    for (uint8_t i = 0; i < inst->numDestRegs(); i++) {
        RegId regId = inst->destRegIdx(i);
        DPRINTF(VW, "%s Dest Reg #%u: %s\n", inst->mnemonic, i, regId);
        if (regId.isFloatReg()) {
            RegIndex regIndex = regId.index();
            if (regIndex >= XMM_REG_BASE_IDX && regIndex < (XMM_REG_BASE_IDX + NUM_XMM_REGS)) {
                regIndex -= XMM_REG_BASE_IDX;
                validVectorRegisters[regIndex] = true;
            }
        }
    }
    return true;
}

void
VWUnit::processInst(struct fullAddr addr, StaticInstPtr &inst, uint32_t iteration, bool &widen, bool &skip)
{
    DPRINTF(VW, "Addr: %s, Mnem: %s, State: %s, Iteration: %u\n", addr.str(), inst->mnemonic, this->state, iteration);

    switch(this->state) {
        case INACTIVE :
            break;

        case ANALYZE :
            assert(addr >= this->start_addr && addr <= this->end_addr);

            this->cur_loop_insts++;
            if (this->cur_loop_insts > this->MAX_LOOP_INSTS) {
                DPRINTF(VW, "Too many instructions in loop to support vector widening\n");
                this->state = INACTIVE;
                break;
            }

            if (inst->isVector()) {
                if (!isVectorInstSupported(inst)) {
                    DPRINTF(VW, "Unsupported vector instruction found: %s\n", inst->disassemble(addr.pc));
                    this->state = INACTIVE;
                    break;
                }
                if (!analyzeVectorInstRegisters(inst)) {
                    DPRINTF(VW, "Vector registers prevent widening: %s\n", inst->disassemble(addr.pc));
                    this->state = INACTIVE;
                    break;
                }
            }

            // the second half of this conditional is specific to microbenchmark
            if (addr == end_addr && (iteration % 2 == 1)) {
                DPRINTF(VW, "Successfully reached end of loop iteration, moving to transformation\n");
                this->transform_iteration = iteration + 1;
                this->state = TRANSFORM;
            }
            break;

        case TRANSFORM :
            assert(addr >= this->start_addr && addr <= this->end_addr);
            if (inst->isVector()) {
                if ((this->transform_iteration % 2) == (iteration % 2)) {
                    DPRINTF(VW, "Widening vector instruction\n");
                    DPRINTF(VW, "Before transformation: %s\n", inst->disassemble(addr.pc));
                    StaticInstPtr instWider = widenVecInst(inst);
                    if (inst->isMicroop()) {
                        instWider->macroOp = inst->macroOp;
                    }
                    //no memory leak with inst because '=' is overloaded to decrement reference count
                    inst = instWider;
                    widen = true;
                    DPRINTF(VW, "After transformation: %s\n", inst->disassemble(addr.pc));
                } else {
                    DPRINTF(VW, "Skipping vector instruction: %s\n", inst->disassemble(addr.pc));
                    skip = true;
                }
            }
            break;
    }
}

void
VWUnit::reset(struct fullAddr start_addr, struct fullAddr end_addr)
{
    assert(this->state == INACTIVE);

    this->clear();
    this->start_addr.pc = start_addr.pc;
    this->start_addr.upc = start_addr.upc;
    this->end_addr.pc = end_addr.pc;
    this->end_addr.upc = end_addr.upc;
    this->state = ANALYZE;
}

StaticInstPtr
VWUnit::widenVecInst(StaticInstPtr &inst)
{
    std::string mnemonic = inst->mnemonic;
    if (mnemonic == "ldfp128") {
        X86ISAInst::Ldfp128 * sis = (X86ISAInst::Ldfp128 *) inst.get();
        X86ISAInst::Ldfp256 * sis_wider = new X86ISAInst::Ldfp256(
            (StaticInst::ExtMachInst) sis->machInst,
            "WIDENED",
            (uint64_t) sis->flags.to_ullong(),
            (uint8_t) sis->scale,
            X86ISA::InstRegIndex((RegIndex) sis->index),
            X86ISA::InstRegIndex((RegIndex) sis->base),
            (uint64_t) sis->disp,
            X86ISA::InstRegIndex((RegIndex) sis->segment),
            X86ISA::InstRegIndex((RegIndex) sis->data),
            (uint8_t) sis->dataSize * 2,
            (uint8_t) sis->addressSize,
            (Request::FlagsType) sis->memFlags
        );
        return (StaticInstPtr) sis_wider;
    } else if (mnemonic == "stfp128") {
        X86ISAInst::Stfp128 * sis = (X86ISAInst::Stfp128 *) inst.get();
        X86ISAInst::Stfp256 * sis_wider = new X86ISAInst::Stfp256(
            (StaticInst::ExtMachInst) sis->machInst,
            "WIDENED",
            (uint64_t) sis->flags.to_ullong(),
            (uint8_t) sis->scale,
            X86ISA::InstRegIndex((RegIndex) sis->index),
            X86ISA::InstRegIndex((RegIndex) sis->base),
            (uint64_t) sis->disp,
            X86ISA::InstRegIndex((RegIndex) sis->segment),
            X86ISA::InstRegIndex((RegIndex) sis->data),
            (uint8_t) sis->dataSize * 2,
            (uint8_t) sis->addressSize,
            (Request::FlagsType) sis->memFlags
        );
        return (StaticInstPtr) sis_wider;
    } else if (mnemonic == "vaddi") {
        X86ISAInst::Vaddi * sis = (X86ISAInst::Vaddi *) inst.get();
        X86ISAInst::Vaddi * sis_wider = new X86ISAInst::Vaddi(
            (StaticInst::ExtMachInst) sis->machInst,
            "WIDENED",
            (uint64_t) sis->flags.to_ullong(),
            (X86ISA::AVXOpBase::SrcType) sis->srcType,
            X86ISA::InstRegIndex((RegIndex) sis->dest),
            X86ISA::InstRegIndex((RegIndex) sis->src1),
            X86ISA::InstRegIndex((RegIndex) sis->src2),
            (uint8_t) sis->destSize,
            (uint8_t) sis->destVL * 2,
            (uint8_t) sis->srcSize,
            (uint8_t) sis->srcVL * 2,
            (uint8_t) sis->imm8,
            (uint8_t) sis->ext
        );
        return (StaticInstPtr) sis_wider;
    } else {
        panic("Attempted to widen unsupported vector instruction\n");
    }
}

VWUnit*
VectorWideningUnitParams::create()
{
    return new VWUnit(this);
}