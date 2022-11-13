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
    this->required_vector_stride_registers.fill(false);
    this->validated_vector_stride_registers.fill(false);
    this->valid_vector_registers.fill(false);
    this->set_int_registers.fill(false);
    this->vector_stride_seen = this->MAX_LOOP_INSTS + 1;
    this->vector_stride_reg = this->NUM_INT_REGS;
    this->vector_instruction_seen = false;
}

void
VWUnit::deactivate()
{
    this->state = INACTIVE;
}

Addr
VWUnit::getEndPC()
{
    return this->end_addr.pc;
}

bool
VWUnit::isVectorInstSupported(StaticInstPtr &inst)
{
    std::string mnemonic = inst->mnemonic;
    return (this->supported_vector_insts.find(mnemonic) != this->supported_vector_insts.end());
}

bool
VWUnit::analyzeInstRegisters(StaticInstPtr &inst)
{
    std::string mnemonic = inst->mnemonic;
    // Analyze source registers
    if (inst->isVector()) {
        // Ensure that every vector register being read by this instruction
        // has been set
        for (uint8_t i = 0; i < inst->numSrcRegs(); i++) {
            RegId reg_id = inst->srcRegIdx(i);
            if (reg_id.isFloatReg()) {
                RegIndex reg_index = reg_id.index();
                if (isVectorRegister(reg_index)) {
                    reg_index -= VEC_REG_BASE;
                    if (!valid_vector_registers[reg_index]) {
                        DPRINTF(VW, "Vector register being read before being set: %s\n", mnemonic);
                        return false;
                    }
                }
            }
        }
        // Mark that the register base for all memory operations needs to be
        // validated as strided by the vector width
        if (inst->isMemRef()) {
            if (mnemonic == "ldfp128") {
                X86ISAInst::Ldfp128 * sis = (X86ISAInst::Ldfp128 *) inst.get();
                DPRINTF(VW, "Setting load reg as required: %u\n", sis->base);
                required_vector_stride_registers[sis->base] = true;
            } else if (mnemonic == "stfp128") {
                X86ISAInst::Stfp128 * sis = (X86ISAInst::Stfp128 *) inst.get();
                DPRINTF(VW, "Setting store reg as required: %u\n", sis->base);
                required_vector_stride_registers[sis->base] = true;
            } else {
                DPRINTF(VW, "Unexpected vector memref inst found: %s\n", mnemonic);
                return false;
            }
        }
    }

    // Analyze current instruction
    if (inst->isInteger()) {
        std::string mnemonic = inst->mnemonic;
        if (mnemonic == "limm") {
            X86ISAInst::Limm * sis = (X86ISAInst::Limm *) inst.get();
            if (sis->imm == this->VECTOR_STRIDE) {
                this->vector_stride_seen = this->cur_loop_insts;
                this->vector_stride_reg = sis->dest;
            }
        } else if (mnemonic == "add") {
            X86ISAInst::Add * sis = (X86ISAInst::Add *) inst.get();
            if ((this->cur_loop_insts == this->vector_stride_seen + 1) && (this->vector_stride_reg == sis->src2)
                 && (!this->set_int_registers[sis->src1])) {
                DPRINTF(VW, "Vector strided register found: %u\n", sis->src1);
                this->validated_vector_stride_registers[sis->src1] = true;
            }
        }
    }

    // Analyze destination registers
    for (uint8_t i = 0; i < inst->numDestRegs(); i++) {
        RegId reg_id = inst->destRegIdx(i);
        if (reg_id.isFloatReg()) {
            // Record all vector registers being set by this instruction
            RegIndex reg_index = reg_id.index();
            if (isVectorRegister(reg_index)) {
                reg_index -= VEC_REG_BASE;
                this->valid_vector_registers[reg_index] = true;
            }
        } else if (reg_id.isIntReg()) {
            RegIndex reg_index = reg_id.index();
            // This case is true if we have already declared a register as a stride
            // register but we are setting it again, so we need to reverse that
            // decision.
            if (this->validated_vector_stride_registers[reg_index] && this->set_int_registers[reg_index]) {
                DPRINTF(VW, "Previusly declared vector stride register is being set again: %u\n", reg_index);
                this->validated_vector_stride_registers[reg_index] = false;
            }
            this->set_int_registers[reg_index] = true;
        }
    }
    return true;
}

bool
VWUnit::verifyVectorStrideRegisters()
{
    RegIndex size = this->required_vector_stride_registers.size();
    assert(size == this->validated_vector_stride_registers.size());
    for (RegIndex index = 0; index < size; index++) {
        if (this->required_vector_stride_registers[index] != this->validated_vector_stride_registers[index]) {
            return false;
        }
    }
    return true;
}

void
VWUnit::processInst(struct fullAddr addr, StaticInstPtr &inst, uint32_t iteration, bool &skip, bool &branch_pred)
{
    DPRINTF(VW, "Addr: %s, Mnem: %s, State: %s, Iteration: %u\n", addr.str(), inst->mnemonic, this->state, iteration);

    switch(this->state) {
        case INACTIVE :
            break;

        case ANALYZE :
            assert(addr >= this->start_addr && addr <= this->end_addr);

            if (addr == start_addr) {
                if (inst->isControl()) {
                    DPRINTF(VW, "First instruction is a conditional branch\n");
                    this->state = INACTIVE;
                    break;
                }
            }

            this->cur_loop_insts++;
            if (this->cur_loop_insts > this->MAX_LOOP_INSTS) {
                DPRINTF(VW, "Too many instructions in loop to support vector widening\n");
                this->state = INACTIVE;
                break;
            }

            if (inst->isVector()) {
                vector_instruction_seen = true;
                if (!isVectorInstSupported(inst)) {
                    DPRINTF(VW, "Unsupported vector instruction found: %s\n", inst->disassemble(addr.pc));
                    this->state = INACTIVE;
                    break;
                }
            }

            if (!analyzeInstRegisters(inst)) {
                DPRINTF(VW, "Register dependencies prevent widening: %s\n", inst->disassemble(addr.pc));
                this->state = INACTIVE;
                break;
            }

            if (addr == end_addr) {
                if (!vector_instruction_seen) {
                    DPRINTF(VW, "No vector instruction seen so no need to widen\n");
                    this->state = INACTIVE;
                    break;
                }
                if (!inst->isCondCtrl()) {
                    DPRINTF(VW, "Last instruction is not a conditional branch\n");
                    this->state = INACTIVE;
                    break;
                }
                if (!verifyVectorStrideRegisters()) {
                    DPRINTF(VW, "Not all vector memory references could be verified as strided\n");
                    this->state = INACTIVE;
                    break;
                }
                DPRINTF(VW, "Successfully reached end of loop iteration. Transitioning to transformation\n");
                this->transform_iteration = iteration + 1;
                this->state = TRANSFORM;
            }
            break;

        case TRANSFORM :
            assert(addr >= this->start_addr && addr <= this->end_addr);
            if((this->transform_iteration % 2) == (iteration % 2)) {
                if (addr == this->start_addr && !branch_pred) {
                    DPRINTF(VW, "The next branch is predicted as not taken, so not widening\n");
                    this->state = INACTIVE;
                    break;
                }
                if (inst->isVector()) {
                    DPRINTF(VW, "Widening vector instruction\n");
                    DPRINTF(VW, "Before transformation: %s\n", inst->disassemble(addr.pc));
                    StaticInstPtr inst_wider = widenVecInst(inst);
                    if (inst->isMicroop()) {
                        inst_wider->macroOp = inst->macroOp;
                    }
                    //no memory leak with inst because '=' is overloaded to decrement reference count
                    inst = inst_wider;
                    DPRINTF(VW, "After transformation: %s\n", inst->disassemble(addr.pc));
                }
                if (addr == this->start_addr) {
                    inst->unsafeSequenceStart = true;
                }
            } else {
                if (inst->isVector()) {
                    DPRINTF(VW, "Skipping vector instruction: %s\n", inst->disassemble(addr.pc));
                    skip = true;
                }
                if (addr == this->end_addr) {
                    inst->unsafeSequenceEnd = true;
                }
            }
            inst->setUnsafe(true);
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
            "VEC_WIDENED",
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