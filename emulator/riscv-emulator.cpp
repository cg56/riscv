//-----------------------------------------------------------------------------
//
// Copyright 2023 Colm Gavin
//
// This emulator emulates a simple single-core RISC-V processor with 64Mbytes
// of RAM and a memory-mapped 8250 UART for serial I/O (keyboard input and
// text output).
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

// The ram is memory mapped into the address space from RamStart. So there is
// lots of spare space for memory-mapped I/O before the start of the ram.

uint8_t   *ram = nullptr;
const int RamStart = 0x80000000;
const int RamSize = 64*1024*1024;   // 64 Mbytes

// Internal CPU registers
uint32_t pc = RamStart;     // Program counter
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
// These are memory-mapped timer registers

uint32_t timerl = 0;
uint32_t timerh = 0;
uint32_t timermatchl = 0;
uint32_t timermatchh = 0;


//-----------------------------------------------------------------------------
//
// Print an error message and exit.
//
//-----------------------------------------------------------------------------

static void errorExit(const string &message)
{
    cerr << "Error: " << message << endl;
    exit(1);
}

#if 0
//-----------------------------------------------------------------------------
//
// Convert "val" to a binary string.
//
//-----------------------------------------------------------------------------

static string toBin(uint32_t val)
{
    char buffer[33];

    for (int i = 31; i >= 0; --i, val >>=1)
        buffer[i] = (val & 1) ? '1' : '0';

    buffer[32] = '\0';
    return buffer;
}
#endif

//-----------------------------------------------------------------------------
//
// Convert "val" to a hexidecimal string.
// Easier than trying to use std::hex an a stringstream.
//
//-----------------------------------------------------------------------------

static string toHex(uint32_t val)
{
    char buffer[9];

    for (int i = 7; i >= 0; --i, val >>=4)
        buffer[i] = "0123456789abcdef"[val & 0x0f];

    buffer[8] = '\0';
    return buffer;
}


//-----------------------------------------------------------------------------
//
// We want unbuffered keyboard input so we don't have to wait for a
// trailing newline. We can give each typed character to the emulated
// machine as soon as it's typed.
//
//-----------------------------------------------------------------------------

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


//-----------------------------------------------------------------------------
//
// Check if there is an input character available. We don't want to block
// the emulation if there is nothing to read.
//
//-----------------------------------------------------------------------------

static bool isCharAvailable()
{
    int n;
    ioctl(fileno(stdin), FIONREAD, &n);

    return n > 0;
}


//-----------------------------------------------------------------------------
//
// Restore normal buffered input. Used when we are exiting.
//
//-----------------------------------------------------------------------------

static void resetBufferedInput()
{
    termios settings;
    tcgetattr(0, &settings);

    settings.c_lflag |= ICANON;  // Re-enable line buffered input
    settings.c_lflag |= ECHO;    // Re-enable character echo

    tcsetattr(0, TCSANOW, &settings);
}


//-----------------------------------------------------------------------------
//
// If we hit an unknown instruction, there's probably something wrong with
// our emulator. So print out the location and the instruction to help
// debugging.
//
//-----------------------------------------------------------------------------

static void unknownInstruction(uint32_t instr, int line)
{
    errorExit("unknown instruction " + toHex(instr) + " at address " + toHex(pc-4) + " line " + to_string(line));
}


//-----------------------------------------------------------------------------
//
// The ram array is not mapped into the address space from zero, so we need
// to convert an absolute address into a location in the ram array. 
//
//-----------------------------------------------------------------------------

static uint32_t adjustAddress(uint32_t addr)
{
    addr -= RamStart;

    if (addr > RamSize - 4)
        errorExit("invalid memory access " + toHex(addr+RamStart));

    return addr;
}


//-----------------------------------------------------------------------------
//
// Check if an address is within the memory-mapped I/O range.
//
//-----------------------------------------------------------------------------

static bool isMemoryMappedIoAddress(uint32_t addr)
{
    return (addr >= 0x10000000) && (addr < 0x12000000);
}


//-----------------------------------------------------------------------------
//
// The UART has a few readable registers. We silently ignore most of them
// except for the status register and receive buffer.
// 
// See https://web.archive.org/web/20160503070506/http://archive.pcjs.org/pubs/pc/datasheets/8250A-UART.pdf
//
//-----------------------------------------------------------------------------

static uint8_t readUart(uint32_t addr)
{
    const uint8_t TransmitterHoldingRegisterIsEmpty = 0x40;
    const uint8_t TransmitterIsEmpty                = 0x20;

    // Our transmitter can never be non-empty, because we flush all writes
    // to the transmitter holding buffer immediately.

    if (addr == 0x10000005)     // 8250 Uart line status register
        return TransmitterHoldingRegisterIsEmpty |
               TransmitterIsEmpty |
               isCharAvailable();

    // We don't want to hang up in getchar(), so make sure a character
    // is available before we call it just in case the caller didn't
    // check the status register.

    if (addr == 0x10000000)    // 8250 UART receive buffer
        return isCharAvailable() ? getchar() : 0;

    return 0;
}


//-----------------------------------------------------------------------------
//
// The UART has a few writable registers. We silently ignore most of them
// except for bytes written to the transmit holding buffer.
// 
//-----------------------------------------------------------------------------

static void writeUart(uint32_t addr, uint8_t val)
{
    if (addr == 0x10000000)  // 8250 Uart transmitter holding buffer
    {
        printf("%c", val);
        fflush(stdout);     // Not the most efficient, but keeps the output flowing.
    }
}


//-----------------------------------------------------------------------------
//
// We support four 32-bit memory mapped registers for the CLINT timer and
// timer match registers.
// 
//-----------------------------------------------------------------------------

static uint32_t readHardware32(uint32_t addr)
{
    if (addr == 0x11004000)
        return timermatchl;
    if (addr == 0x11004004)
        return timermatchh;
    if (addr == 0x1100bffc)
        return timerh;
    if (addr == 0x1100bff8)
        return timerl;

    return 0;
}


//-----------------------------------------------------------------------------
//
// Although we can read four 32-bit registers, only two of them are
// writable to set the timer match in the CLINT. We also support writing
// to a system control register for power-off.
// 
//-----------------------------------------------------------------------------

static void writeHardware32(uint32_t addr, uint32_t val)
{
    if (addr == 0x11004000)
        timermatchl = val;
    else if (addr == 0x11004004)
        timermatchh = val;
	else if (addr == 0x11100000)    // System control
        exit(0);
}


//-----------------------------------------------------------------------------
// 
// Perform a 32-bit aligned read, either from ram or memory-mapped I/O.
//
//-----------------------------------------------------------------------------

static uint32_t read32(uint32_t addr)
{
    // Is the address aligned to 32-bits?

    if (addr & 0x3)
    {
        csreg[mtval] = addr;
        throw (uint32_t)4;  // Load-address-misaligned exception
    }

    // Read from ram or memory-mapped I/O

    if (isMemoryMappedIoAddress(addr))
        return readHardware32(addr);
    else
        return *(uint32_t *)&ram[adjustAddress(addr)];
}


//-----------------------------------------------------------------------------
// 
// Perform a 32-bit aligned write, either to ram or memory-mapped I/O.
//
//-----------------------------------------------------------------------------

static void write32(uint32_t addr, uint32_t val)
{
    // Is the address aligned to 32-bits?

    if (addr & 0x3)
    {
        csreg[mtval] = addr;
        throw (uint32_t)6;  // Store-address-misaligned exception
    }

    // Write to ram or memory-mapped I/O

    if (isMemoryMappedIoAddress(addr))
        writeHardware32(addr, val);
    else
        *(uint32_t *)&ram[adjustAddress(addr)] = val;
}


//-----------------------------------------------------------------------------
// 
// Perform a 16-bit aligned read, only from ram because we don't have any
// 16-bit memory-mapped I/O registers.
//
//-----------------------------------------------------------------------------

static uint16_t read16(uint32_t addr)
{
    // Is the address aligned to 32-bits?

    if (addr & 0x1)
    {
        csreg[mtval] = addr;
        throw (uint32_t)4;  // Load-address-misaligned exception
    }

    // Read from ram

    return *(uint16_t *)&ram[adjustAddress(addr)];
}


//-----------------------------------------------------------------------------
// 
// Perform a 16-bit aligned write, only to ram because we don't have any
// 16-bit memory-mapped I/O registers.
//
//-----------------------------------------------------------------------------

static void write16(uint32_t addr, uint16_t val)
{
    // Is the address aligned to 32-bits?

    if (addr & 0x1)
    {
        csreg[mtval] = addr;
        throw (uint32_t)6;  // Store-address-misaligned exception
    }

    // Write to ram

    *(uint16_t *)&ram[adjustAddress(addr)] = val;
}


//-----------------------------------------------------------------------------
// 
// Perform a 8-bit read, either from ram or memory-mapped I/O. There are
// no alignment issues for byte reads.
//
//-----------------------------------------------------------------------------

static uint8_t read8(uint32_t addr)
{
    if (isMemoryMappedIoAddress(addr))
        return readUart(addr);
    else
        return ram[adjustAddress(addr)];
}


//-----------------------------------------------------------------------------
// 
// Perform a 8-bit write, either to ram or memory-mapped I/O. There are
// no alignment issues for byte reads.
//
//-----------------------------------------------------------------------------

static void write8(uint32_t addr, uint8_t val)
{
    if ((addr >= 0x10000000) && (addr < 0x10000008))
        writeUart(addr, val);
    else
        ram[adjustAddress(addr)] = val;
}


//-----------------------------------------------------------------------------
// 
// Dump the internal state of all the registers plus a few important CSR
// registers too.
//
//-----------------------------------------------------------------------------

static void dumpState()
{
	printf("%08x ", pc);
    if (pc - RamStart <=  RamSize - 4)
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
    printf("\n");
}


//-----------------------------------------------------------------------------
// 
// Read in an image to run.
//
//-----------------------------------------------------------------------------

static void readImage(const string &filename)
{
    ifstream in(filename);
    if (!in.good())
        errorExit("cannot read image file: " + filename);

    in.read((char *)ram, RamSize);

    if (!in.eof())
        errorExit("image file is larger than available memory");
}


//-----------------------------------------------------------------------------
// 
// Put the DTB at the end of the ram and update it with the size of the 
// available memory.
//
//-----------------------------------------------------------------------------

static void loadDtb()
{
    uint32_t dtb = RamSize - sizeof(default64mbdtb);

    memcpy(&ram[dtb], default64mbdtb, sizeof(default64mbdtb));

    // The size of the available memory is actually just the location of
    // the DTB

    write32(dtb + 0x13c + RamStart, htonl(dtb));

    // Pass the DTB location to the image that is being run

    xreg[11] = dtb + RamStart;
}


//-----------------------------------------------------------------------------
// 
// Validates the alignment of the address before setting the PC.
// 
//-----------------------------------------------------------------------------

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


//-----------------------------------------------------------------------------
// 
// This is it, the emulator engine! This function emulates a single
// RISC-V instruction supporting the IMA configuration.
// 
//-----------------------------------------------------------------------------

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
    uint32_t immIu = instr >> 20;           // Unsigned, not sign extended
    int32_t  immI  = (int32_t)instr >> 20;  // Sign extended

    // S-type
    int32_t immS = ((instr >>  7) & 0x0000001f) |
                   (((int32_t)instr >> 20) & 0xffffffe0);  // Sign extended

    // B-type
    int immB = ((instr >>  7) & 0x0000001e) |   // Lowest bit is always zero
               ((instr >> 20) & 0x000007e0) |
               ((instr <<  4) & 0x00000800) |
               (((int32_t)instr >> 19) & 0xfffff000);   // Sign extended

    // U-type

    int immU   = instr & 0xfffff000;    // Not shifted! Effectively << 12;

    // J-type
    int immJ = ((instr >> 20) & 0x000007fe) |
               ((instr >>  9) & 0x00000800) |
               ((instr >>  0) & 0x000ff000) |
               (((int32_t)instr >> 11) & 0xfff00000);   // Sign extended

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
                    xreg[rd] = (int8_t)read8(xreg[rs1] + immI);
                    break;
                case 0b001: // LH, Load short, signed
                    xreg[rd] = (int16_t)read16(xreg[rs1] + immI);
                    break;
                case 0b010: // LW, Load word
                    xreg[rd] = read32(xreg[rs1] + immI);
                    break;
                case 0b100: // LBU, Load unsigned byte
                    xreg[rd] = read8(xreg[rs1] + immI);
                    break;
                case 0b101: // LHU, Load unsigned short 
                    xreg[rd] = read16(xreg[rs1] + immI);
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
                    xreg[rd] = xreg[rs1] + immI;
                    break;
                case 0b001:
                    switch (funct7)
                    {
                        case 0b0000000: // SLLI, Logical shift left immediate
                            xreg[rd] = xreg[rs1] << (immIu & 0x1f);
                            break;
                        default:
                            unknownInstruction(instr, __LINE__);
                    }
                    break;
                case 0b010: // SLTI, Set less than immediate (signed)
                    xreg[rd] = ((int32_t)xreg[rs1] < immI) ? 1 : 0;
                    break;
                case 0b011: // SLTIU, Set less than immediate unsigned
                    xreg[rd] = (xreg[rs1] < immIu) ? 1 : 0;
                    break;
                case 0b100: // XORI, xor immediate
                    xreg[rd] = xreg[rs1] ^ immI;
                    break;
                case 0b101:
                    switch (funct7)
                    {
                        case 0b0000000: // SRLI, Logical shift right immediate
                            xreg[rd] = xreg[rs1] >> (immI & 0x1f);
                            break;
                        case 0b0100000: // SRAI, Arithmetic shift right immediate
                            xreg[rd] = (int32_t)xreg[rs1] >> (immI & 0x1f);
                            break;
                        default:
                            unknownInstruction(instr, __LINE__);
                    }
                    break;
                case 0b110: // ORI, or immediate
                    xreg[rd] = xreg[rs1] | immI;
                    break;
                case 0b111: // ANDI, and immediate
                    xreg[rd] = xreg[rs1] & immI;
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
                    write8(xreg[rs1] + immS, xreg[rs2]);
                    break;
                case 0b001: // SH, Store short (16-bits)
                    write16(xreg[rs1] + immS, xreg[rs2]);
                    break;
                case 0b010: // SW, Store word (32-bits)
                    write32(xreg[rs1] + immS, xreg[rs2]);
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
                        setpc(pc - 4 + immB);
                    break;
                case 0b001: // BNE, Branch if not equal
                    if (xreg[rs1] != xreg[rs2])
                        setpc(pc - 4 + immB);
                    break;
                case 0b100: // BLT, Branch if less than, signed
                    if ((int32_t)xreg[rs1] < (int32_t)xreg[rs2])
                        setpc(pc - 4 + immB);
                    break;
                case 0b101: // BGE, Branch if greater or equal, signed
                    if ((int32_t)xreg[rs1] >= (int32_t)xreg[rs2])
                        setpc(pc - 4 + immB);
                    break;
                case 0b110: // BLTU, Branch if less than, unsigned
                    if (xreg[rs1] < xreg[rs2])
                        setpc(pc - 4 + immB);
                    break;
                case 0b111: // BGEU, Branch if greater or equal, unsigned
                    if (xreg[rs1] >= xreg[rs2])
                        setpc(pc - 4 + immB);
                    break;
                default:
                    unknownInstruction(instr, __LINE__);
            }
            break;

        case 0b1100111: // JALR, I-type
        {
            uint32_t tmp = pc;
            setpc((xreg[rs1] + immI) & ~1);
            xreg[rd] = tmp;
        }
            break;

        case 0b1101111: // JAL, Jump and link, J-type
            xreg[rd] = pc;
            setpc(pc - 4 + immJ);
            break;

        // No need to worry about atomic instructions, because only 1 core and no cache.

        case 0b1110011: // SYSTEM, I-type
            switch (funct3)
            {
                uint32_t tmp;

                case 0b000:
                    switch (immIu)
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
                    tmp = csreg[immIu];
                    csreg[immIu]  = xreg[rs1];
                    xreg[rd]     = tmp;
                    break;
                case 0b010: // CSRRS, Atomic read and set bits in CSR
                    tmp = csreg[immIu];
                    csreg[immIu] |= xreg[rs1];
                    xreg[rd]     = tmp;
                    break;
                case 0b011: // CSRRC, Atomic read and clear bits in CSR
                    tmp = csreg[immIu];
                    csreg[immIu] &= ~xreg[rs1];
                    xreg[rd]     = tmp;
                    break;
                case 0b101: // CSRRWI, Atomic read/write CSR immediate
                    xreg[rd] = csreg[immIu];
                    csreg[immIu] = rs1;
                    break;
                case 0b110: // CSRRSI, Atomic read/write CSR immediate
                    xreg[rd] = csreg[immIu];
                    csreg[immIu] |= rs1;
                    break;
                case 0b111: // CSRRCI, Atomic read and clear bits in CSR immediate
                    xreg[rd] = csreg[immIu];
                    csreg[immIu] &= ~rs1;
                    break;
                default:
                    unknownInstruction(instr, __LINE__);
            }
            break;

        default:
            unknownInstruction(instr, __LINE__);
    }
}


//-----------------------------------------------------------------------------
// 
// Get the time since the epoch (in microseconds) as a single 64-bit
// number.
// 
//-----------------------------------------------------------------------------

static uint64_t gettime()
{
    struct timeval tv;
    gettimeofday(&tv, 0);

    return (uint64_t)tv.tv_usec + (uint64_t)tv.tv_sec * 1000000;
}


//-----------------------------------------------------------------------------
// 
// Run the emulator forever until the image exits or something goes wrong.
// 
//-----------------------------------------------------------------------------

static void run()
{
#ifdef FIXED_CLOCK
    uint64_t startTime = 0;
#endif // FIXED_CLOCK

    while (1)
    {
        if (debug)
            dumpState();

        try
        {
            // Update the timer. Very inefficint to set for every single instruction,
            // but this is just an emulator so performance isn't critical.

#ifdef FIXED_CLOCK
            uint64_t now = startTime++;
#else
            uint64_t now = gettime();
#endif // FIXED_CLOCK

            timerh = now >> 32;
            timerl = now & 0xffffffff;

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


//-----------------------------------------------------------------------------
// 
// Let's go!
// 
//-----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    atexit(resetBufferedInput);
    setUnbufferedInput();

    cout << "Welcome to Colm's risc-v emulator!\n"
         << "This emulator supports rv32ima instruction set." << endl;

    ram = new uint8_t[RamSize];

    if (argc == 2)
    {
        // ./riscv-emulator ../mini-rv32ima-master/buildroot/output/images/Image
        readImage(argv[1]);
        loadDtb();
    }
    else
        errorExit("usage: riscv-emulator <ImageFile>");

    run();
}
