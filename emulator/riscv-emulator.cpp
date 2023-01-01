//-----------------------------------------------------------------------------
//
// Add copyright message ??
//
//-----------------------------------------------------------------------------

#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <termios.h>
#include <arpa/inet.h>  // For htonl
#include <sys/ioctl.h>
#include <sys/time.h>
#include "default64mbdtc.h" //??

using namespace std;

const bool debug = false;
uint8_t *memory = nullptr;
const int MemorySize = 64*1024*1024;
const int ImageStart = 0x80000000;
uint32_t begin_signature, end_signature;    //??

uint32_t pc = ImageStart;   // Program counter
uint32_t xreg[32] = {0};    // Integer registers
uint32_t csreg[4096] = {0}; // Control & Status registers

// Fixed locations inside the C&S registers
// See https://people.eecs.berkeley.edu/~krste/papers/riscv-privileged-v1.9.pdf

// Machine trap setup
const int mstatus  = 0x300; // Machine status register
const int mie      = 0x304; // ??
const int mtvec    = 0x305; // Machine trap-handler base address
// Machine trap handling
const int mscratch = 0x340; // ??
const int mepc     = 0x341; // Machine exception program counter
const int mcause   = 0x342; // Machine trap cause
const int mtval    = 0x343; // Machine bad address
const int mip      = 0x344; // Machine interrupt pending
// Add support for misa??
bool waitingForInterrupt = false;
int privilege = 3;         // Just 2 bits for 4 levels. Start in machine mode

// CLINT registers, see https://chromitem-soc.readthedocs.io/en/latest/clint.html
uint32_t timerl = 0;
uint32_t timerh = 0;
uint32_t timermatchl = 0;
uint32_t timermatchh = 0;

/*
0 0 1 1 0 0 0 0 0 1 0 0 mie
0 0 1 1 0 1 0 0 0 1 0 0 mip
*/

static void errorExit(const string &message)
{
    cerr << "Error: " << message << endl;
    exit(1);
}


static string toBin(uint32_t val)
{
    char buffer[33];

    for (int i = 31; i >= 0; --i, val >>=1)
        buffer[i] = (val & 1) ? '1' : '0';

    buffer[32] = '\0';
    return buffer;
}


static string toHex(uint32_t val)
{
    char buffer[9];

    for (int i = 7; i >= 0; --i, val >>=4)
        buffer[i] = "0123456789abcdef"[val & 0x0f];

    buffer[8] = '\0';
    return buffer;
}


//-------------------------------------------------------------------------
//
// We want unbuffered keyboard input so we don't have to wait for a
// trailing newline.
//
//-------------------------------------------------------------------------

static void setUnbufferedInput()
{
    termios settings;
    tcgetattr(0, &settings);

    settings.c_lflag &= (~ICANON);  // Disable line buffered input
    settings.c_lflag &= (~ECHO);    // Disable character echo

    settings.c_cc[VTIME] = 0;   // Timeout
    settings.c_cc[VMIN] = 1;    // Minimum number of characters

    tcsetattr(0, TCSANOW, &settings);
}


//-------------------------------------------------------------------------
//
// Check if there is an input character available. We don't want to block
// the emulation if there is nothing to read.
//
//-------------------------------------------------------------------------

static bool isCharAvailable()
{
    int n;
    ioctl(fileno(stdin), FIONREAD, &n);

    return n > 0;
    //??return false;
}


//-------------------------------------------------------------------------
//
// Restore normal buffered input (when we are exiting).
//
//-------------------------------------------------------------------------

static void resetBufferedInput()
{
    termios settings;
    tcgetattr(0, &settings);

    settings.c_lflag |= ICANON;  // Re-enable line buffered input
    settings.c_lflag |= ECHO;    // Re-enable character echo

    tcsetattr(0, TCSANOW, &settings);
}


static void unknownInstruction(uint32_t instr, int line)
{
    errorExit("unknown instruction " + toHex(instr) + " at address " + toHex(pc-4) + " line " + to_string(line));
}


static uint32_t adjustAddress(uint32_t addr)
{
    addr -= ImageStart;

    if (addr > MemorySize - 4)
        errorExit("invalid memory access " + toHex(addr+ImageStart));  // Add memory mapped i/o ??

    return addr;
}


static void writeUart(uint32_t addr, uint8_t val)
{
    if (addr == 0x10000000)  // 8250 Uart transmitt holding buffer
    {
        printf("%c", val);
        fflush(stdout);
    }
}


static uint32_t readHardware(uint32_t addr)
{
    if (addr == 0x10000005)     // 8250 Uart line status register
        return 0x60 | isCharAvailable();
    if ((addr == 0x10000000) && isCharAvailable()) // 8250 Uart receive buffer
        return getchar();
    if (addr == 0x1100bffc)     // CLINT timer register (64-bit)
        return timerh;
    if (addr == 0x1100bff8)
        return timerl;

    return 0;
}


static void writeHardware(uint32_t addr, uint32_t val)
{
    if (addr == 0x11004000)         //CLINT
        timermatchl = val;
    else if (addr == 0x11004004)    //CLINT
        timermatchh = val;
	else if (addr == 0x11100000)    //SYSCON (reboot, poweroff, etc.)
        exit(0);
}


static uint32_t read32(uint32_t addr)
{
    if (addr & 0x3)
    {
        csreg[mtval] = addr;
        throw (uint32_t)4;  // Load-address-misaligned exception
    }

    if ((addr >= 0x10000000) && (addr < 0x12000000))
        return readHardware(addr);
    else
        return *(uint32_t *)&memory[adjustAddress(addr)];
}


static void checkAndExit()
{
    for (uint32_t addr = begin_signature; addr < end_signature; addr += 4)
        printf("%08x\n", read32(addr));
    exit(0);
}


static void write32(uint32_t addr, uint32_t val)
{
    if (addr & 0x3)
    {
        csreg[mtval] = addr;
        throw (uint32_t)6;  // Store-address-misaligned exception
    }

    //if (addr == 0x80001000)    // Hack for testcases ??
        //checkAndExit();
    if ((addr >= 0x10000000) && (addr < 0x12000000))
        writeHardware(addr, val);
    else
        *(uint32_t *)&memory[adjustAddress(addr)] = val;
}


static uint16_t read16(uint32_t addr)
{
    if (addr & 0x1)
    {
        csreg[mtval] = addr;
        throw (uint32_t)4;  // Load-address-misaligned exception
    }

    if ((addr >= 0x10000000) && (addr < 0x12000000))
        return readHardware(addr);
    else
        return *(uint16_t *)&memory[adjustAddress(addr)];
}


static void write16(uint32_t addr, uint16_t val)
{
    if (addr & 0x1)
    {
        csreg[mtval] = addr;
        throw (uint32_t)6;  // Store-address-misaligned exception
    }

    *(uint16_t *)&memory[adjustAddress(addr)] = val;
}


static uint8_t read8(uint32_t addr)
{
    if ((addr >= 0x10000000) && (addr < 0x12000000))
        return readHardware(addr);
    else
        return memory[adjustAddress(addr)];
}


static void write8(uint32_t addr, uint8_t val)
{
    if ((addr >= 0x10000000) && (addr < 0x10000008))
        writeUart(addr, val);
    else
        memory[adjustAddress(addr)] = val;
}


static void dumpState()
{
	printf("%08x ", pc);
    if (pc - ImageStart <=  MemorySize - 4)
		printf("[%08x] ", read32(pc)); 
	else
		printf("[xxxxxxxxxx] "); 

    printf("0:%08x 1:%08x 2:%08x 3:%08x 4:%08x 5:%08x 6:%08x 7:%08x 8:%08x 9:%08x 10:%08x 11:%08x 12:%08x 13:%08x 14:%08x 15:%08x ",
		0, xreg[1], xreg[2], xreg[3],
        xreg[4], xreg[5], xreg[6], xreg[7],
		xreg[8], xreg[9], xreg[10], xreg[11],
        xreg[12], xreg[13], xreg[14], xreg[15]);
    printf("16:%08x 17:%08x 18:%08x 19:%08x 20:%08x 21:%08x 22:%08x 23:%08x 24:%08x 25:%08x 26:%08x 27:%08x 28:%08x 29:%08x 30:%08x 31:%08x ",
		xreg[16], xreg[17], xreg[18], xreg[19],
        xreg[20], xreg[21], xreg[22], xreg[23],
		xreg[24], xreg[25], xreg[26], xreg[27],
        xreg[28], xreg[29], xreg[30], xreg[31]);
    printf("p:%d w:%c ms:%08x ie:%08x sc:%08x tv:%08x ip:%08x",
        privilege, (waitingForInterrupt?'T':'F'), csreg[mstatus], csreg[mie], csreg[mscratch], csreg[mtval], csreg[mip]);
    printf(" tmr %08x %08x %08x %08x", timerh, timerl, timermatchh, timermatchl);
    //printf(" [%08x]", read32(0x8040f034));
    //printf("%08x@%08x %08x %08x %08x\n", instr, pc-4, csreg[mip], csreg[mie], csreg[mstatus]);
    //printf("%08x@%08x %08x %08x %08x %08x %08x\n", instr, pc-4, xreg[10], xreg[11], xreg[13], xreg[14], xreg[21]);
    //printf("%08x@%08x %08x\n", instr, pc-4, read32(0x802efe28));
    //printf("%08x@%08x %08x %08x %08x %08x\n", instr, pc-4, timerh, timerl, timermatchh, timermatchl);
    printf("\n");
}


static void readTestcaseElf(const string &filename)
{
    ifstream in(filename);
    if (!in.good())
        errorExit("cannot read elf file: " + filename);

    in.read((char *)memory, 0x1000);   // Skip the start of elf files ??
    in.read((char *)memory, MemorySize);

    if (!in.eof())
        errorExit("image file is larger than available memory");
}


static void readImage(const string &filename)
{
    ifstream in(filename);
    if (!in.good())
        errorExit("cannot read image file: " + filename);

    in.read((char *)memory, MemorySize);

    if (!in.eof())
        errorExit("image file is larger than available memory");
}


static void loadDtb()
{
    uint32_t dtb = MemorySize - sizeof(default64mbdtb) - 48*4;  //??
    memcpy(&memory[dtb], default64mbdtb, sizeof(default64mbdtb));

    uint32_t addr = dtb + 0x13c;
    //printf("Dtb 1 %08x\n", *(uint32_t *)&memory[addr]);
    //printf("%08x\n", read32(addr + ImageStart));

    write32(dtb + 0x13c + ImageStart, htonl(dtb));

    //printf("Dtb 2 %08x\n", *(uint32_t *)&memory[addr]);

    xreg[11] = dtb + ImageStart;
}


static void setpc(uint32_t addr)
{
    // Check is PC 4-byte aligned?

    if (addr & 0x3)
    {
        csreg[mtval] = addr;
        throw (uint32_t)0;  // Instruction-address-misaligned exception
    }

    pc = addr;
}

static void executeInstruction()
{
    uint32_t instr = read32(pc); pc += 4;

    xreg[0] = 0;   // Just in case an instruction used x0 as the destination register

    // Decode instructions for each type

    // R-type
    int funct7 = (instr >> 25);         // 7 bits
    int rs2    = (instr >> 20) & 0x1f;  // 5 bits
    int rs1    = (instr >> 15) & 0x1f;  // 5 bits
    int funct3 = (instr >> 12) & 0x7;   // 3 bits
    int rd     = (instr >> 7) & 0x1f;   // 5 bits
    int opcode = (instr >> 0) & 0x7f;   // 7 bits

    // I-type
    int immI   = instr >> 20;
    int immIs  = (int32_t)instr >> 20; // Sign extended

    // S-type
    int immS = ((instr >>  7) & 0x0000001f) |
               ((instr >> 20) & 0x00000fe0);
    int immSs = ((instr >>  7) & 0x0000001f) |
                (((int32_t)instr >> 20) & 0xffffffe0);

    // B-type
    int immB = ((instr >>  7) & 0x0000001e) |    // Lowest bit is always zero
               ((instr >> 20) & 0x000007e0) |
               ((instr <<  4) & 0x00000800) |
               ((instr >> 19) & 0x00001000);
    int immBs = ((instr >>  7) & 0x0000001e) |    // Lowest bit is always zero
                ((instr >> 20) & 0x000007e0) |
                ((instr <<  4) & 0x00000800) |
                (((int32_t)instr >> 19) & 0xfffff000);

    // U-type

    int immU   = instr & 0xfffff000;    // Not shifted! Effectively << 12;

    // J-type
    int immJ = ((instr >> 20) & 0x000007fe) |
               ((instr >>  9) & 0x00000800) |
               ((instr >>  0) & 0x000ff000) |
               ((instr >> 11) & 0x00100000);
    int immJs = ((instr >> 20) & 0x000007fe) |
                ((instr >>  9) & 0x00000800) |
                ((instr >>  0) & 0x000ff000) |
                (((int32_t)instr >> 11) & 0xfff00000);

    /*
    inst[4:2] 000  001      010      011      100    101      110            111
    inst[6:5] (> 32b)
    00      LOAD   LOAD-FP  custom-0 MISC-MEM OP-IMM AUIPC    OP-IMM-32      48b
    01      STORE  STORE-FP custom-1 AMO      OP     LUI      OP-32          64b
    10      MADD   MSUB     NMSUB    NMADD    OP-FP  OP-V     custom-2/rv128 48b
    11      BRANCH JALR     reserved JAL      SYSTEM reserved custom-3/rv128 ≥ 80b
    */

    switch (opcode)
    {
        case 0b0000011: // LOAD, I-type
            switch (funct3)
            {
                case 0b000: // LB, Load byte, signed
                    xreg[rd] = (int8_t)read8(xreg[rs1] + immIs);
                    break;
                case 0b001: // LH, Load short, signed
                    xreg[rd] = (int16_t)read16(xreg[rs1] + immIs);
                    break;
                case 0b010: // LW, Load word
                    xreg[rd] = read32(xreg[rs1] + immIs);
                    break;
                case 0b100: // LBU, Load unsigned byte
                    xreg[rd] = read8(xreg[rs1] + immIs);
                    break;
                case 0b101: // LHU, Load unsigned short 
                    xreg[rd] = read16(xreg[rs1] + immIs);
                    break;
                default:
                    unknownInstruction(instr, __LINE__);
            }
            break;

        case 0b0001111: // MISC-MEM, I-type
            switch (funct3)
            {
                case 0b000: // Fence
                    break;  // Nothing to do because no cache
                case 0b001: // Fence.i
                    break;  // Nothing to do because no cache
                default:
                    unknownInstruction(instr, __LINE__);
            }
            break;

        case 0b0010011: // OP-IMM, I-type
            switch (funct3)
            {
                case 0b000: // ADDI, add immediate
                    xreg[rd] = xreg[rs1] + immIs;
                    break;
                case 0b001:
                    switch (funct7)
                    {
                        case 0b0000000: // SLLI, Logical shift left immediate
                            xreg[rd] = xreg[rs1] << (immI & 0x1f);
                            break;
                        default:
                            unknownInstruction(instr, __LINE__);
                    }
                    break;
                case 0b010: // SLTI, Set less than immediate (signed)
                    xreg[rd] = ((int32_t)xreg[rs1] < immIs) ? 1 : 0;
                    break;
                case 0b011: // SLTIU, Set less than immediate unsigned
                    xreg[rd] = (xreg[rs1] < immIs) ? 1 : 0;
                    break;
                case 0b100: // XORI, xor immediate
                    xreg[rd] = xreg[rs1] ^ immIs;
                    break;
                case 0b101:
                    switch (funct7)
                    {
                        case 0b0000000: // SRLI, Logical shift right immediate
                            xreg[rd] = xreg[rs1] >> (immIs & 0x1f);
                            break;
                        case 0b0100000: // SRAI, Arithmetic shift right immediate
                            xreg[rd] = (int32_t)xreg[rs1] >> (immIs & 0x1f);
                            break;
                        default:
                            unknownInstruction(instr, __LINE__);
                    }
                    break;
                case 0b110: // ORI, or immediate
                    xreg[rd] = xreg[rs1] | immIs;
                    break;
                case 0b111: // ANDI, and immediate
                    xreg[rd] = xreg[rs1] & immIs;
                    break;
                default:
                    unknownInstruction(instr, __LINE__);
            }
            break;

        case 0b0010111: // AUIPC, U-type
            xreg[rd] = pc - 4 + immU;
            break;

        case 0b0100011: // STORE, S-type
            switch (funct3)
            {
                case 0b000: // SB, Store byte (8-bits)
                    write8(xreg[rs1] + immSs, xreg[rs2]);
                    break;
                case 0b001: // SH, Store short (16-bits)
                    write16(xreg[rs1] + immSs, xreg[rs2]);
                    break;
                case 0b010: // SW, Store word (32-bits)
                    write32(xreg[rs1] + immSs, xreg[rs2]);
                    break;
                default:
                    unknownInstruction(instr, __LINE__);
            }
            break;

        case 0b0101111: // AMO , Atomic memory operations, R-type
            switch  (funct7 & 0x7c)     // Ignore lower two bits
            {
                uint32_t addr;
                uint32_t tmp;

                case 0b0000000:     // AMOADD.W
                    addr = xreg[rs1];
                    tmp = xreg[rs2];
                    xreg[rd] = read32(addr);
                    write32(addr, xreg[rd] + tmp);
                    break;
                case 0b0000100:     // AMOSWAP.W
                    addr = xreg[rs1];
                    tmp = xreg[rs2];
                    xreg[rd] = read32(addr);
                    write32(addr, tmp);
                    break;
                case 0b0001000:     // LR.W
                    xreg[rd] = read32(xreg[rs1]);
                    break;
                case 0b0001100:     // SC.W
                    write32(xreg[rs1], xreg[rs2]);
                    xreg[rd] = 0;   // Always succeeds
                    break;
                case 0b0010000:     // AMOXOR.W
                    addr = xreg[rs1];
                    tmp = xreg[rs2];
                    xreg[rd] = read32(addr);
                    write32(addr, xreg[rd] ^ tmp);
                    break;
                case 0b0100000:     // AMOOR.W
                    addr = xreg[rs1];
                    tmp = xreg[rs2];
                    xreg[rd] = read32(addr);
                    write32(addr, xreg[rd] | tmp);
                    break;
                case 0b0110000:     // AMOAND.W
                    addr = xreg[rs1];
                    tmp = xreg[rs2];
                    xreg[rd] = read32(addr);
                    write32(addr, xreg[rd] & tmp);
                    break;
                default:
                    unknownInstruction(instr, __LINE__);
            }
            break;

        case 0b0110011: // OP, R-type
            switch (funct7)
            {
                case 0b0000000:
                    switch (funct3)
                    {
                        case 0b000:     // ADD
                            xreg[rd] = xreg[rs1] + xreg[rs2];
                            break;
                        case 0b001:     // SLL, Shift left logical
                            xreg[rd] = xreg[rs1] << (xreg[rs2] & 0x1f);
                            break;
                        case 0b010:     // SLT, Set if less than (signed)
                            xreg[rd] = ((int32_t)xreg[rs1] < (int32_t)xreg[rs2]) ? 1 : 0;
                            break;
                        case 0b011:     // SLTU, Set if less than (unsigned)
                            xreg[rd] = (xreg[rs1] < xreg[rs2]) ? 1 : 0;
                            break;
                        case 0b100:     // XOR
                            xreg[rd] = xreg[rs1] ^ xreg[rs2];
                            break;
                        case 0b101:     // SRL, shift right logical
                            xreg[rd] = xreg[rs1] >> (xreg[rs2] & 0x1f);
                            break;
                        case 0b110:     // OR
                            xreg[rd] = xreg[rs1] | xreg[rs2];
                            break;
                        case 0b111:     // AND
                            xreg[rd] = xreg[rs1] & xreg[rs2];
                            break;
                    }
                    break;
                case 0b0000001:
                    switch (funct3)
                    {
                        case 0b000:     // MUL, multiply
                            xreg[rd] = xreg[rs1] * xreg[rs2];
                            break;
                        case 0b001:     // MULH, multiply high word (signed x signed)
                            xreg[rd] = ((int64_t)xreg[rs1] * (int64_t)xreg[rs2]) >> 32;
                            break;
                        case 0b010:     // MULHSU, multiply high word (signed x unsigned)
                            xreg[rd] = ((int64_t)xreg[rs1] * (uint64_t)xreg[rs2]) >> 32;
                            break;
                        case 0b011:     // MULHU, multiply high word (unsigned x unsigned)
                            xreg[rd] = ((uint64_t)xreg[rs1] * (uint64_t)xreg[rs2]) >> 32;
                            break;
                        case 0b100:     // DIV, signed division
                            xreg[rd] = (xreg[rs2] == 0) ? -1 : (int32_t)xreg[rs1] / (int32_t)xreg[rs2];
                            break;
                        case 0b101:     // DIVU, unsigned division
                            xreg[rd] = (xreg[rs2] == 0) ? -1 : xreg[rs1] / xreg[rs2];
                            break;
                        case 0b110:     // REM
                            xreg[rd] = (int32_t)xreg[rs1] % (uint32_t)xreg[rs2];
                            break;
                        case 0b111:     // REMU
                            xreg[rd] = xreg[rs1] % xreg[rs2];
                            break;
                    }
                    break;
                case 0b0100000:
                    switch (funct3)
                    {
                        case 0b000:     // SUB, subtract
                            xreg[rd] = xreg[rs1] - xreg[rs2];
                            break;
                        case 0b101:     // SRL, shift right arithmetic
                            xreg[rd] = (int32_t)xreg[rs1] >> (xreg[rs2] & 0x1f);
                            break;
                        default:
                            unknownInstruction(instr, __LINE__);
                    }
                    break;
                default:
                    unknownInstruction(instr, __LINE__);
            }
            break;

        case 0b0110111: // LUI, U-type
            xreg[rd] = immU;
            break;

        case 0b1100011: // Branch, B-type
            switch (funct3)
            {
                case 0b000: // BEQ, Branch if equal
                    if (xreg[rs1] == xreg[rs2])
                        setpc(pc - 4 + immBs);
                    break;
                case 0b001: // BNE, Branch if not equal
                    if (xreg[rs1] != xreg[rs2])
                        setpc(pc - 4 + immBs);
                    break;
                case 0b100: // BLT, Branch if less than, signed
                    if ((int32_t)xreg[rs1] < (int32_t)xreg[rs2])
                        setpc(pc - 4 + immBs);
                    break;
                case 0b101: // BGE, Branch if greater or equal, signed
                    if ((int32_t)xreg[rs1] >= (int32_t)xreg[rs2])
                        setpc(pc - 4 + immBs);
                    break;
                case 0b110: // BLTU, Branch if less than, unsigned
                    if (xreg[rs1] < xreg[rs2])
                        setpc(pc - 4 + immBs);
                    break;
                case 0b111: // BGEU, Branch if greater or equal, unsigned
                    if (xreg[rs1] >= xreg[rs2])
                        setpc(pc - 4 + immBs);
                    break;
                default:
                    unknownInstruction(instr, __LINE__);
            }
            break;

        case 0b1100111: // JALR, I-type
        {
            uint32_t tmp = pc;
            setpc((xreg[rs1] + immIs) & ~1);
            xreg[rd] = tmp;
        }
            break;

        case 0b1101111: // JAL, Jump and link, J-type
            xreg[rd] = pc;
            setpc(pc - 4 + immJs);
            break;

        // No need to worry about atomic instructions, because only 1 core and no cache.

        case 0b1110011: // SYSTEM, I-type
            switch (funct3)
            {
                uint32_t tmp;

                case 0b000:
                    switch (immI)
                    {
                        case 0b000000000000:    // ECALL
                            csreg[mtval] = pc-4;

                            if (privilege == 0)
                                throw (uint32_t)8; // Environment call from user mode
                            else
                                throw (uint32_t)11;  // Environment call from machine mode
                            break;
                        case 0b000000000001:    // EBREAK
                            throw (uint32_t)3;
                            break;
                        case 0b000100000101:    // WFI, wait for interrupt
                            waitingForInterrupt = true;
                            csreg[mstatus] |= 8;    // Enable interrupts??
                            return;             // Nothing more to do
                        case 0b001100000010:    // MRET
                            {
							uint32_t tmp_status = csreg[mstatus];
							uint32_t tmp_privilege = privilege;
							csreg[mstatus] = ((tmp_status & 0x80) >> 4)
                                            | ((tmp_privilege & 0x3) << 11)
                                            | 0x80;
							privilege = (tmp_status >> 11) & 0x3;
                            }

                            pc = csreg[mepc];
                            break;
                        default:
                            unknownInstruction(instr, __LINE__);
                    }
                case 0b001: // CSRRW, Atomic read/write CSR
                    tmp = csreg[immI];
                    csreg[immI]  = xreg[rs1];
                    xreg[rd]     = tmp;
                    break;
                case 0b010: // CSRRS, Atomic read and set bits in CSR
                    tmp = csreg[immI];
                    csreg[immI] |= xreg[rs1];
                    xreg[rd]     = tmp;
                    break;
                case 0b011: // CSRRC, Atomic read and clear bits in CSR
                    tmp = csreg[immI];
                    csreg[immI] &= ~xreg[rs1];
                    xreg[rd]     = tmp;
                    break;
                case 0b101: // CSRRWI, Atomic read/write CSR immediate
                    xreg[rd] = csreg[immI];
                    csreg[immI] = rs1;
                    break;
                case 0b110: // CSRRSI, Atomic read/write CSR immediate
                    xreg[rd] = csreg[immI];
                    csreg[immI] |= rs1;
                    break;
                case 0b111: // CSRRCI, Atomic read and clear bits in CSR immediate
                    xreg[rd] = csreg[immI];
                    csreg[immI] &= ~rs1;
                    break;
                default:
                    unknownInstruction(instr, __LINE__);
            }
            break;

        default:
            unknownInstruction(instr, __LINE__);
    }
}


static uint64_t gettime()
{
    struct timeval tv;
    gettimeofday(&tv, 0);

    return (uint64_t)tv.tv_usec + (uint64_t)tv.tv_sec * 1000000;
}


static void run()
{
    uint64_t startTime = 0; //??gettime();
    uint64_t instrCount = 0;

    while (1)
    {
        if (debug)
            dumpState();

        try
        {
            if (++instrCount % 1000 == 0)
            {
            // Update the timer. No need to check every single instruction ??

            uint64_t now = gettime(); //??startTime++;
            timerh = now >> 32;
            timerl = now & 0xffffffff;
            }

            // Timer expired?

            if ((timermatchl || timermatchh)
             && ((timerh > timermatchh) || ((timerh == timermatchh) && (timerl > timermatchl))))
            {
                waitingForInterrupt = false;
                csreg[mip] |= 1<<7;
            }
            else
                csreg[mip] &= ~(1<<7);

            if (!waitingForInterrupt)
                executeInstruction();

            if ((csreg[mip] & (1<<7))
             && (csreg[mie] & (1<<7))
             && (csreg[mstatus] & 0x8))
            {
                csreg[mtval] = 0;
                throw (uint32_t)0x80000007;  // Timer interrupt
            }
        }
        catch (uint32_t trap)
        { 
            if (debug)
                printf("Caught trap %08x\n", trap);

            csreg[mcause] = trap;
            csreg[mstatus] = ((csreg[mstatus] & 0x8) << 4) | ((privilege & 0x3) << 11);

            if (trap & 0x80000000)  // It's an interrupt
                csreg[mepc] = pc;   // PC is where we will return to
            else
            {
                csreg[mepc] = pc - 4;   // PC is the address of the instruction which caused the trap
                privilege = 3;  // Switch to machine mode. Only on traps, not interrupts??
            }

            pc = csreg[mtvec];
        }
    }
}


int main(int argc, char *argv[])
{
    atexit(resetBufferedInput);
    setUnbufferedInput();

    cout << "Welcome to Colm's risc-v emulator!\n"
         << "This emulator supports rv32ima instruction set." << endl;

    memory = new uint8_t[MemorySize];

    if (argc == 2)
    {
        // ./riscv-emulator ../mini-rv32ima-master/buildroot/output/images/Image
        readImage(argv[1]);
        loadDtb();
    }
    else if (argc == 4)
    {
        sscanf(argv[1], "%x", &begin_signature);
        sscanf(argv[2], "%x", &end_signature);
        readTestcaseElf(argv[3]);
    }

    run();
}
