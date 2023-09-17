#include "common.h"
#include "SdCache/SdCache.h"
#include "JitCommon.h"
#include "JitArm.h"

static void __attribute__((noinline)) armJitNotImplemented()
{
    asm volatile ("bkpt #0");
    while (1);
}

void jit_processArmBlock(u32* ptr)
{
    // logAddress(0xA);
    // logAddress((u32)ptr);
    void* const blockStart = jit_findBlockStart(ptr);
    void* blockEnd = jit_findBlockEnd(ptr);
    u32* jitBits = jit_getJitBits(ptr);
    do
    {
        if (jitBits)
        {
            u32 bitIdx = ((u32)ptr & 0x3F) >> 1;
            u32 bitMask = 3 << bitIdx;
            if (*jitBits & bitMask)
            {
                // stop because this instruction was already processed
                break;
            }
            *jitBits |= bitMask;
            if (bitIdx == 30)
                jitBits++;
        }
        u32 instruction = *ptr;
        if ((instruction & 0x0E000000) == 0x0A000000)
        {
            // B and BL imm
            if (instruction & 0x01000000)
            {
                // BL imm
                *ptr = (instruction & ~0x0E000000) | 0x0C000000;
                if ((instruction >> 28) == 0xE)
                {
                    break;
                }
            }
            else
            {
                // B imm
                // *ptr = (instruction & ~0xFE000000) | 0xEC000000;
                // *(u32*)(((u32)ptr & ~0x01000000) + 0x00400000) = instruction;
                *ptr = (instruction & ~0x0E000000) | 0x0C000000;
                if ((instruction >> 28) == 0xE)
                {
                    break;
                }
            }
        }
        else if ((instruction & 0x0FBF0FFF) == 0x010F0000)
        {
            // MRS
            *ptr = 0x01A00090 | (instruction & 0xF0400000) | ((instruction & 0x0000F000) >> 12);
        }
        else if ((instruction & 0x0FB0FFF0) == 0x0120F000)
        {
            // MSR reg
            *ptr = 0x1800090 | (instruction & 0xF04F000F) | ((instruction & 0x02000000) >> 5);
        }
        else if ((instruction & 0x0FFFFFF0) == 0x012FFF10)
        {
            // BX
            *ptr = 0x01B00090 | (instruction & 0xF000000F);
            if ((instruction >> 28) == 0xE)
            {
                break;
            }
        }
        else if ((instruction & 0x0E108000) == 0x08108000)
        {
            // LDM pc
            *ptr = 0x06400010
                | (instruction & 0xF1A00000) // cond, P, U, W
                | ((instruction & 0x000F0000) >> 16) // Rn
                | ((instruction & 0x7FFF) << 5); // rlist
            if ((instruction >> 28) == 0xE)
            {
                break;
            }
        }
        else if ((instruction & 0x0C50F000) == 0x0410F000)
        {
            // LDR pc
            if (!(instruction & 0x01000000))
            {
                // post
                armJitNotImplemented();
            }
            else
            {
                *ptr = 0x0E800010
                    | (instruction & 0xF0200000) // cond, W
                    | ((instruction & 0x000F0000) >> 16) // Rd
                    | ((instruction & 0xF) << 5) // op2
                    | ((instruction & 0xFF0) << 8) // op2
                    | ((instruction & 0x01800000) >> 14) // P, U
                    | ((instruction & 0x02000000) >> 3); // I
            }
            if ((instruction >> 28) == 0xE)
            {
                break;
            }
        }
        else if ((instruction & 0x0C5F0000) == 0x041F0000)
        {
            // LDR Rd, [pc, ...]
            if (!(instruction & 0x02000000))
            {
                // LDR Rd, [pc, #imm]
                int offset = instruction & 0xFFF;
                if (!(instruction & 0x00800000))
                    offset = -offset;                
                u32 targetAddress = (u32)ptr + 8 + offset;
                if (targetAddress >= (u32)blockStart && targetAddress < (u32)blockEnd)
                {
                    if (targetAddress > (u32)ptr)
                        blockEnd = (void*)targetAddress;
                    // safe pool address needs no patching
                    continue;
                }
            }
        }
        else if ((instruction & 0x0E00F010) == 0x0000F000)
        {
            // ALU{S} pc, Rn, Rm (imm shift)
            *ptr = 0x0E000000
                | (instruction & 0xF0000000) // cond
                | ((instruction & 0x01E00000) >> 4) // op
                | ((instruction & 0x00100000) << 1) // S
                | ((instruction & 0x000F0000) >> 16) // Rn
                | ((instruction & 0x00000FFF) << 5); // op2
            if ((instruction >> 28) == 0xE)
            {
                break;
            }
        }
        else if ((instruction & 0x0E00F010) == 0x0000F010)
        {
            // ALU{S} pc, Rn, Rm (reg shift)
            armJitNotImplemented();
            if ((instruction >> 28) == 0xE)
            {
                break;
            }
        }
        else if ((instruction & 0x0E00F000) == 0x0200F000)
        {
            // ALU{S} pc, Rn, #imm
            *ptr = 0x0E400000
                | (instruction & 0xF0000000) // cond
                | ((instruction & 0x01E00000) >> 4) // op
                | ((instruction & 0x00100000) << 1) // S
                | ((instruction & 0x000F0000) >> 16) // Rn
                | ((instruction & 0x00000FFF) << 5); // op2
            if ((instruction >> 28) == 0xE)
            {
                break;
            }
        }
        else if ((instruction & 0x0F000000) == 0x0F000000)
        {
            // SWI
            u32 swiOp = instruction & 0xFFFFFF;
            if (swiOp == 0 || swiOp == 0x260000)
            {
                if ((instruction >> 28) == 0xE)
                {
                    break;
                }
            }
        }
    } while ((u32)++ptr < (u32)blockEnd);
}

[[gnu::section(".itcm")]]
u32* jit_handleArmUndefined(u32 instruction, u32* instructionPtr, u32* registers, u32 cpsr)
{
    // if ((instruction & 0x0F000000) == 0x0C000000)
    // {
    //     // b cond
    //     u32 originalInstruction = *(u32*)(((u32)instructionPtr & ~0x01000000) + 0x00400000);
    //     u32 condition = originalInstruction >> 28;
    //     bool conditionPass = jit_conditionPass(cpsr, condition);
    //     u32 branchDestination = (u32)instructionPtr + 8 + ((int)(instruction << 8) >> 6);
    //     bool putBack;
    //     u32 address;
    //     if (conditionPass)
    //     {
    //         address = branchDestination;
    //         jit_ensureBlockJitted((void*)address);
    //         putBack = jit_isBlockJitted(instructionPtr + 1);
    //     }
    //     else
    //     {
    //         address = (u32)(instructionPtr + 1);
    //         jit_ensureBlockJitted((void*)address);
    //         putBack = jit_isBlockJitted((void*)branchDestination);
    //     }
    //     if (putBack)
    //     {
    //         *instructionPtr = originalInstruction;
    //         dc_drainWriteBuffer();
    //         ic_invalidateAll();
    //     }
    //     return (u32*)address;
    // }
    if ((instruction & 0x0FC00010) == 0x0E000000)
    {
        // ALU{S} pc, Rn, Rm (imm shift)
        u32 op = (instruction >> 17) & 0xF;
        u32 rn = registers[instruction & 0xF];
        u32 rm = registers[(instruction >> 5) & 0xF];
        u32 shiftType = (instruction >> 10) & 0x3;
        u32 shiftAmount = (instruction >> 12) & 0x1F;
        if (shiftType != 0 && shiftAmount == 0)
        {
            armJitNotImplemented();
        }
        u32 op2;
        switch (shiftType)
        {
            case 0: // lsl
                op2 = rm << shiftAmount;
                break;
            case 1: // lsr
                op2 = rm >> shiftAmount;
                break;
            case 2: // asr
                op2 = (int)rm >> shiftAmount;
                break;
            case 3: // ror
                armJitNotImplemented();
                break;
        }
        u32 branchTarget;
        switch (op)
        {
            case 2: // sub
                branchTarget = rn - op2;
                break;
            case 4: // add
                branchTarget = rn + op2;
                break;
            case 0xD: // mov
                branchTarget = op2;
                break;
            default:
                armJitNotImplemented();
                break;
        }
        branchTarget &= ~3;
        if (instruction & 0x00200000)
        {
            // ALUS pc, Rn, Rm (imm shift)
            armJitNotImplemented();
        }
        jit_ensureBlockJitted((void*)branchTarget);
        return (u32*)branchTarget;
    }
    else if ((instruction & 0x0FC00010) == 0x0E400000)
    {
        // ALU{S} pc, Rn, #imm
        u32 imm8 = (instruction >> 5) & 0xFF;
        u32 ror = ((instruction >> 13) & 0xF) << 1;
        u32 op = (instruction >> 17) & 0xF;
        u32 rn = registers[instruction & 0xF];
        u32 imm = (imm8 >> ror) | (imm8 << (32 - ror));
        u32 branchTarget;
        switch (op)
        {
            case 2: // sub
                branchTarget = rn - imm;
                break;
            case 4: // add
                branchTarget = rn + imm;
                break;
            case 0xD: // mov
                branchTarget = imm;
                break;
            default:
                armJitNotImplemented();
                break;
        }
        branchTarget &= ~3;
        if (instruction & 0x00200000)
        {
            // ALUS pc, Rn, #imm
            armJitNotImplemented();
        }
        jit_ensureBlockJitted((void*)branchTarget);
        return (u32*)branchTarget;
    }
    else if ((instruction & 0x0E500010) == 0x06400010)
    {
        // LDM Rn{!}, {...,pc}
        u32 rn = instruction & 0xF;
        if ((instruction & 0x01A00000) == 0x00A00000)
        {
            // LDMIA Rn!, {...,pc}
            u32* src = (u32*)registers[rn];
            for (u32 i = 0; i < 15; i++)
            {
                if (instruction & (0x20 << i))
                {
                    registers[i] = *src++;
                }
            }
            u32 branchDestination = *src++;
            registers[rn] = (u32)src;
            jit_ensureBlockJitted((void*)branchDestination);
            return (u32*)branchDestination;
        }
        armJitNotImplemented();
    }
    else if ((instruction & 0x0F900810) == 0x0E800010)
    {
        // ldr pc
        if ((instruction & 0x00600400) == 0x00400400)
        {
            //ldr pc, [Rn, Rm, shift]
            u32 op = (instruction >> 17) & 0xF;
            u32 rn = registers[instruction & 0xF];
            u32 rm = registers[(instruction >> 5) & 0xF];
            u32 shiftType = (instruction >> 13) & 0x3;
            u32 shiftAmount = (instruction >> 15) & 0x1F;
            if (shiftType != 0 && shiftAmount == 0)
            {
                armJitNotImplemented();
            }
            u32 op2;
            switch (shiftType)
            {
                case 0: // lsl
                    op2 = rm << shiftAmount;
                    break;
                case 1: // lsr
                    op2 = rm >> shiftAmount;
                    break;
                case 2: // asr
                    op2 = (int)rm >> shiftAmount;
                    break;
                case 3: // ror
                    armJitNotImplemented();
                    break;
            }
            u32 branchDestination = *(u32*)(rn + op2);
            jit_ensureBlockJitted((void*)branchDestination);
            return (u32*)branchDestination;
        }
        armJitNotImplemented();
    }
    armJitNotImplemented();
    return instructionPtr + 1;
}