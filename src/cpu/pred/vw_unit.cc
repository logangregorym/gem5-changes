#include "arch/x86/generated/decoder.hh"
#include "cpu/pred/vw_unit.hh"

#include "debug/VW.hh"

VWUnit::VWUnit(const Params *params)
    : SimObject(params), state(INACTIVE)
{}

void
VWUnit::clear()
{
    if (this->state != INACTIVE) {
        DPRINTF(VW, "Transitioning from %u to INACTIVE\n", this->state);
    }
    this->transform_iteration = 0;
    this->state = INACTIVE;
}

void
VWUnit::processInst(struct fullAddr addr, StaticInstPtr &inst, uint32_t iteration)
{
    DPRINTF(VW, "Addr: %s, State: %s, Iteration: %u\n", addr.str(), this->state, iteration);
    auto op_class = inst->opClass();
    switch(this->state) {
        case INACTIVE :
            break;

        case ANALYZE :
            assert(addr >= this->start_addr && addr <= this->end_addr);

            // TODO: if disqualifying condition, this->state = INACTIVE
            if (inst->isVector()) {
                if (op_class != FloatMemReadOp && op_class != FloatMemWriteOp &&
                    op_class != SimdAddOp)
                {
                    DPRINTF(VW, "Unsupported vector instruction found: %s\n", inst->disassemble(addr.pc));
                    this->state = INACTIVE;
                }
            }

            if (addr == end_addr && (iteration % 2 == 1)) {
                DPRINTF(VW, "Successfully reached end of trace, moving to transformation\n");
                this->transform_iteration = iteration + 1;
                this->state = TRANSFORM;
            }
            break;

        case TRANSFORM :
            assert(addr >= this->start_addr && addr <= this->end_addr);
            if (inst->isVector()) {
                DPRINTF(VW, "Before transformation: %s\n", inst->disassemble(addr.pc));
                transformVecInst(inst, iteration);
                DPRINTF(VW, "After transformation: %s\n", inst->disassemble(addr.pc));
            }
            break;
    }
}

void
VWUnit::reset(struct fullAddr start_addr, struct fullAddr end_addr)
{
    assert(this->state == INACTIVE);

    this->start_addr.pc = start_addr.pc;
    this->start_addr.upc = start_addr.upc;
    this->end_addr.pc = end_addr.pc;
    this->end_addr.upc = end_addr.upc;
    this->state = ANALYZE;
}

void
VWUnit::transformVecInst(StaticInstPtr &inst, uint32_t iteration)
{
    if ((this->transform_iteration % 2) == (iteration % 2)) {
        auto op_class = inst->opClass();
        if (op_class == FloatMemReadOp) {
            X86ISAInst::Ldfp128 * sis = (X86ISAInst::Ldfp128 * ) inst.get(); 
            X86ISAInst::Ldfp256 * sis_wider = new X86ISAInst::Ldfp256(
                (StaticInst::ExtMachInst) sis->machInst,
                (const char *) "TRANSFORMED",
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
            sis_wider->macroOp = inst->macroOp;
            inst = sis_wider;
        } else if (op_class == FloatMemWriteOp) {
            X86ISAInst::Stfp128 * sis = (X86ISAInst::Stfp128 * ) inst.get(); 
            X86ISAInst::Stfp256 * sis_wider = new X86ISAInst::Stfp256(
                (StaticInst::ExtMachInst) sis->machInst,
                (const char *) "TRANSFORMED",
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
            sis_wider->macroOp = inst->macroOp;
            inst = sis_wider;
        } else if (op_class == SimdAddOp) {
            X86ISAInst::Vaddi * sis = (X86ISAInst::Vaddi *) inst.get();
            X86ISAInst::Vaddi * sis_wider = new X86ISAInst::Vaddi(
                (StaticInst::ExtMachInst) sis->machInst,
                (const char *) "TRANSFORMED",
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
            sis_wider->macroOp = inst->macroOp;
            inst = sis_wider;
        } else {
            panic("Attempted to widen unsupported vector instruction\n");
        }
    } else {
        // TODO: this does not work for macroops with more than one microop
        inst = StaticInst::nopStaticInstPtr;
    }
}

VWUnit*
VectorWideningUnitParams::create()
{
    return new VWUnit(this);
}