//-----------------------------------------------------------------------------
//
// Copyright Colm Gavin, 2023. All rights reserved.
//
// This code is the boot image for the FPGA RISC-V processor.
// It assumes the CF card is formatted as FAT32.
// It reads "Image.bin" from the CF into the processors RAM.
//
// Thanks to http://elm-chan.org/docs/fat_e.html for explaining the FAT format.
//
//-----------------------------------------------------------------------------

/*
/ ___|(_)_ __ ___  _ __ | | ___  |  _ \(_)___  ___   \ \   / /
\___ \| | '_ ` _ \| '_ \| |/ _ \ | |_) | / __|/ __|___\ \ / / 
 ___) | | | | | | | |_) | |  __/ |  _ <| \__ \ (_|_____\ V /  
|____/|_|_| |_| |_| .__/|_|\___| |_| \_\_|___/\___|     \_/   
                  |_|                                         
  ____      _              ____             _       
 / ___|___ | |_ __ ___    / ___| __ ___   _(_)_ __  
| |   / _ \| | '_ ` _ \  | |  _ / _` \ \ / / | '_ \ 
| |__| (_) | | | | | | | | |_| | (_| |\ V /| | | | |
 \____\___/|_|_| |_| |_|  \____|\__,_| \_/ |_|_| |_|
*/

#if SIM
#include <stdio.h>
#include <stdlib.h>
#endif

//#define DEBUG

typedef int bool;
#define false   0
#define true    1

#define SECTOR_SIZE	512


// These structures define the layout of the Master Boot Record

struct MbrPartition
{
	unsigned char dummy1[8];
	unsigned int  startLba;
	unsigned char dummy2[4];
};

// We need the packed attribute because unfortunately some of the fields are not alligned

struct __attribute__ ((packed)) Mbr
{
    unsigned char  jmpBoot;				// Jump instruction to the bootstrap code
    unsigned char  dummy1[10];
	unsigned short bytesPerSector;		// 11
    unsigned char  clusterSize;			// 13
	unsigned short reservedSectors;		// 14
	unsigned char  numFats;				// 16
    unsigned char  dummy2[15];
	unsigned int   totalSectors;		// 32
	unsigned short sectorsPerFat;		// 36
    unsigned char  dummy3[4];
	unsigned short fileSystemVersion;	// 42
	unsigned int   rootDirStart;		// 44
    unsigned char  dummy4[34];
    unsigned char  fileSystemType[8];	// 82 - string
	unsigned char  dummy5[356];
	struct MbrPartition partition[4];	// 446
    unsigned short bootSig;         	// 510
};


struct DirEntry
{
	unsigned char  name[11];		// 0
	unsigned char  attributes;		// 11
	unsigned char  dummy1[8];		// 12
	unsigned short highCluster;		// 20
	unsigned char  dummy2[4];		// 22
	unsigned short lowCluster;		// 26
	unsigned int   fileSize;		// 28
};


// Everything we need to know about the disk image on the CF card

struct CardInfo
{
	unsigned char sectorsPerCluster;
	unsigned int  fatClusters;
	unsigned int  startFat;
	unsigned int  startRootDir;
	unsigned int  startData;
    unsigned int  cachedSector;

    // When a sector is read in, it can be interpreted in multiple
    // ways depending on the type of the sector.

    union
    {
	    unsigned char cache[SECTOR_SIZE];   // One sector's worth of data from the CF disk
        struct Mbr mbr;
		struct DirEntry dirs[SECTOR_SIZE/sizeof(struct DirEntry)];
    };
};

// Important locations in memory

#if !SIM
volatile unsigned char  *uartData    = (unsigned char  *)0x10000000;
volatile unsigned char  *uartCtrl    = (unsigned char  *)0x10000005;
volatile unsigned char  *spiStatus   = (unsigned char  *)0x12000000;
volatile unsigned char  *spiData     = (unsigned char  *)0x12000001;
volatile unsigned char  *spiResponse = (unsigned char  *)0x12000002;
volatile unsigned char  *spiZero     = (unsigned char  *)0x12000003;
volatile unsigned short *spiDivisor  = (unsigned short *)0x12000004;
volatile unsigned short *spiCrc      = (unsigned short *)0x12000006;
volatile unsigned int   *spiWord     = (unsigned int   *)0x12000008;
volatile unsigned char  *switches    = (unsigned char  *)0x13000008;

unsigned char *memory = (unsigned char *)0x80000000;
#endif


//-----------------------------------------------------------------------------
//
// Read from a CSR register.
//
//-----------------------------------------------------------------------------

#define CSR_READ(v, csr)            \
__asm__ __volatile__ ("csrr %0, %1" \
    : "=r" (v)                      \
    : "n" (csr)                     \
    : /* clobbers: none */)


//-----------------------------------------------------------------------------
//
// Swap bytes from network (CF card big-endian) to host order (little-endian).
// A possible speed improvement would be to do this in hardware (in the SPI
// verilog module).
//
//-----------------------------------------------------------------------------

static unsigned int ntohl(unsigned int val)
{
    return ((val >> 24) & 0x000000ff) |
           ((val >> 8)  & 0x0000ff00) |
           ((val << 8)  & 0x00ff0000) |
           ((val << 24) & 0xff000000);
}


//-----------------------------------------------------------------------------
//
// Outputs a single character to the terminal attached to the serial port.
//
//-----------------------------------------------------------------------------

static void printChar(char ch)
{
#if SIM
    printf("%c", ch);
    fflush(stdout);
#else
    // Wait for the UART to have space for another character

    while ((*uartCtrl & 0x40) == 0)
        ;

    // Send it

    *uartData = ch;
#endif
}


//-----------------------------------------------------------------------------
//
// Outputs a string to the terminal attached to the serial port.
//
//-----------------------------------------------------------------------------


static void printStr(const char *str)
{
    while (*str)
        printChar(*(str++));
}


//-----------------------------------------------------------------------------
//
// Useful debug print statements becuase we don't have printf when not in
// simulation mode.
//
//-----------------------------------------------------------------------------

static void printByteHex(unsigned char val)
{
    static char hex[] = "0123456789abcdef";

    printChar(hex[(val>>4)&0xf]);
    printChar(hex[(val>>0)&0xf]);
}


static void printIntHex(unsigned int val)
{
    printByteHex((val>>24)&0xff);
    printByteHex((val>>16)&0xff);
    printByteHex((val>>8)&0xff);
    printByteHex((val>>0)&0xff);
}


static void printInt(unsigned int val)
{
    if (val == 0)
    {
        printChar('0');
        return;
    }

    char buf[12];
    char *p = buf;

    while (val > 0)
    {
        *p = (val % 10) + '0';
        val /= 10;
        ++p;
    }

    while (--p >= buf)
        printChar(*p);
}


//-----------------------------------------------------------------------------
//
// Print an error message and then halt (in an infinite loop) until the user
// presses the reset button.
//
//-----------------------------------------------------------------------------

static void error(const char *message)
{
    printStr("\r\nError: ");
    printStr(message);
    printStr("\r\n");

#if SIM
    exit(1);
#else
    while(1)
        ;
#endif
}


//-----------------------------------------------------------------------------
//
// Compare two strings.
//
//  1 - if s1 == s2
//  0 - if s1 != s2;
//
//-----------------------------------------------------------------------------

static int strEqual(const char *s1, const char *s2, int n)
{
	for (int i = 0; i < n; ++i)
	{
		if (s1[i] != s2[i])
            return 0;
	}

	return 1;
}

#if !SIM
//-----------------------------------------------------------------------------
//
// Enable access to the CF card
//
//-----------------------------------------------------------------------------

static void setChipSelect(int enable)
{
    *spiData = 0xff;
    *spiStatus = (enable) ? 0x02 : 0x00;    // Bit 1 is chip select
    *spiData = 0xff;
}


//-----------------------------------------------------------------------------
//
// Send a command to the CF card over the SPI interface.
//
//-----------------------------------------------------------------------------

static unsigned char sendCommand(unsigned char *bytes)
{
#ifdef DEBUG
    if (bytes[0] != 0x51)
    {
        printStr("Sending ");
        printByteHex(bytes[0]);
    }
#endif

    *spiData = 0xff;

    for (int i = 0; i < 6; ++i)
        *spiData = bytes[i];

    unsigned char resp = *spiResponse;

#ifdef DEBUG
    if (bytes[0] != 0x51)
    {
        printStr("=>");
        printByteHex(resp);
        printStr("\r\n");
    }
#endif

    return resp;
}


static unsigned char readBytes(int len)
{
    unsigned char val;

    for (int i = 0; i < len; ++i)
        val = *spiData;

    return val;
}


//-----------------------------------------------------------------------------
//
// Initialize the SPI communications with the CF card.
//
//-----------------------------------------------------------------------------

static void initSpi()
{
    unsigned char resp;

    // Is there a CF card inserted?

    if (*spiStatus != 0x01)
        error("no card");

    setChipSelect(true);

    // Send the inialization sequence

    unsigned char cmd0[] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};
    if (sendCommand(cmd0) & 0xfe)
        error("CMD0");

    unsigned char cmd8[] = {0x48, 0x00, 0x00, 0x01, 0x56, 0x43};
    if (sendCommand(cmd8) & 0xfe)
        error("CMD8");

    if (readBytes(4) != 0x56)  // Check the mirror byte
        error("CMD8");

    // Wait (forever) for the card to be ready

    int errorCount = 0;

    do
    {
        unsigned char cmd55[] = {0x77, 0x00, 0x00, 0x00, 0x00, 0x65};
        if ((sendCommand(cmd55) & 0xfe) == 0x00)
        {
            unsigned char cmd41[] = {0x69, 0x40, 0x00, 0x00, 0x00, 0xe5};
            resp = sendCommand(cmd41);
            if (resp & 0xfe) {
                ++errorCount;
                printStr("Error CMD41 => ");
                printByteHex(resp);
                printStr("\r\n");
            }
        }
        else
        {
            printStr("Error CMD55 => ");
            printByteHex(resp);
            printStr("\r\n");
        }
    }
    while ((resp != 0) && (errorCount < 5));
#if 0
    do
    {
        unsigned char cmd55[] = {0x77, 0x00, 0x00, 0x00, 0x00, 0x65};
        if (sendCommand(cmd55) & 0xfe)
            error("CMD55");

        unsigned char cmd41[] = {0x69, 0x40, 0x00, 0x00, 0x00, 0xe5};
        resp = sendCommand(cmd41);
        if (resp & 0xfe)
            error("CMD41");
    }
    while (resp != 0);
#endif

    // Increase the clock frequency from the default 100KHz up to 25MHz
    // 100MHz input clock / 4 = 25MHz

    *spiDivisor = 4;

    // All done

    setChipSelect(false);
}


//-----------------------------------------------------------------------------
//
// Reads a contiguous set of sectors from the CF card. Assumes all sectors are
// 512 bytes long.
//
// sector   - first sector to start reading
// count    - number of sectors to read
//
//-----------------------------------------------------------------------------

void readSectors(unsigned int sector, unsigned int count, unsigned char *buff)
{
    printChar('.');         // Visual progress

    setChipSelect(true);    // Start the transaction

    while (count-- > 0)
    {
        unsigned char cmd17[] = {0x51, 0x00, 0x00, 0x00, 0x00, 0x55};
        cmd17[1] = (sector >> 24) & 0xff;
        cmd17[2] = (sector >> 16) & 0xff;
        cmd17[3] = (sector >> 8)  & 0xff;
        cmd17[4] = (sector >> 0)  & 0xff;

        if (sendCommand(cmd17) != 0)
            error("CMD17");

        volatile unsigned char dummy = *spiZero;   // Skip until the leading zero

        for (int i = 0; i < 512/4; ++i)
            ((unsigned int *)buff)[i] = ntohl(*spiWord);

        unsigned short hwCrc = *spiCrc;
        unsigned short readCrc = *spiData << 8;   // Read the 2-byte CRC
        readCrc |= *spiData;

        if (hwCrc != readCrc)
            error("CRC");

        buff += 512;
        ++sector;
    }

    setChipSelect(false);   // Delimit the transaction
}
#else
//-----------------------------------------------------------------------------
//
// Nothing to do in simulation mode.
//
//-----------------------------------------------------------------------------

static void initSpi()
{
}


//-----------------------------------------------------------------------------
//
// Simulated sector data for a FAT32 foramtted card. Originally dumped from a
// real CF card.
//
//-----------------------------------------------------------------------------

static unsigned char sector0000[] =
{
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x82,
0x03,0x00,0x0C,0xFE,0xFF,0xFF,0x00,0x20,0x00,0x00,0x00,0xAC,0xDA,0x01,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x55,0xAA
};

static unsigned char sector2000[] =
{
0xEB,0x00,0x90,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x02,0x40,0x56,0x02,
0x02,0x00,0x00,0x00,0x00,0xF8,0x00,0x00,0x3F,0x00,0xFF,0x00,0x00,0x20,0x00,0x00,
0x00,0xAC,0xDA,0x01,0xD5,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,
0x01,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x80,0x00,0x29,0x61,0x61,0x33,0x62,0x4E,0x4F,0x20,0x4E,0x41,0x4D,0x45,0x20,0x20,
0x20,0x20,0x46,0x41,0x54,0x33,0x32,0x20,0x20,0x20,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x55,0xAA
};

static unsigned char sector2257[] =
{
0x81,0x00,0x00,0x00,0x82,0x00,0x00,0x00,0x83,0x00,0x00,0x00,0x84,0x00,0x00,0x00,
0x85,0x00,0x00,0x00,0x86,0x00,0x00,0x00,0x87,0x00,0x00,0x00,0x88,0x00,0x00,0x00,
0x89,0x00,0x00,0x00,0x8A,0x00,0x00,0x00,0x8B,0x00,0x00,0x00,0x8C,0x00,0x00,0x00,
0x8D,0x00,0x00,0x00,0x8E,0x00,0x00,0x00,0x8F,0x00,0x00,0x00,0x90,0x00,0x00,0x00,
0x91,0x00,0x00,0x00,0x92,0x00,0x00,0x00,0x93,0x00,0x00,0x00,0x94,0x00,0x00,0x00,
0x95,0x00,0x00,0x00,0x96,0x00,0x00,0x00,0x97,0x00,0x00,0x00,0x98,0x00,0x00,0x00,
0xFF,0xFF,0xFF,0x0F,0x9A,0x00,0x00,0x00,0x9B,0x00,0x00,0x00,0x9C,0x00,0x00,0x00,
0x9D,0x00,0x00,0x00,0x9E,0x00,0x00,0x00,0x9F,0x00,0x00,0x00,0xA0,0x00,0x00,0x00,
0xA1,0x00,0x00,0x00,0xA2,0x00,0x00,0x00,0xA3,0x00,0x00,0x00,0xA4,0x00,0x00,0x00,
0xA5,0x00,0x00,0x00,0xA6,0x00,0x00,0x00,0xA7,0x00,0x00,0x00,0xA8,0x00,0x00,0x00,
0xA9,0x00,0x00,0x00,0xAA,0x00,0x00,0x00,0xAB,0x00,0x00,0x00,0xAC,0x00,0x00,0x00,
0xAD,0x00,0x00,0x00,0xAE,0x00,0x00,0x00,0xAF,0x00,0x00,0x00,0xB0,0x00,0x00,0x00,
0xB1,0x00,0x00,0x00,0xB2,0x00,0x00,0x00,0xB3,0x00,0x00,0x00,0xB4,0x00,0x00,0x00,
0xB5,0x00,0x00,0x00,0xB6,0x00,0x00,0x00,0xB7,0x00,0x00,0x00,0xB8,0x00,0x00,0x00,
0xB9,0x00,0x00,0x00,0xBA,0x00,0x00,0x00,0xBB,0x00,0x00,0x00,0xBC,0x00,0x00,0x00,
0xBD,0x00,0x00,0x00,0xBE,0x00,0x00,0x00,0xBF,0x00,0x00,0x00,0xC0,0x00,0x00,0x00,
0xC1,0x00,0x00,0x00,0xC2,0x00,0x00,0x00,0xC3,0x00,0x00,0x00,0xC4,0x00,0x00,0x00,
0xC5,0x00,0x00,0x00,0xC6,0x00,0x00,0x00,0xC7,0x00,0x00,0x00,0xC8,0x00,0x00,0x00,
0xC9,0x00,0x00,0x00,0xCA,0x00,0x00,0x00,0xCB,0x00,0x00,0x00,0xCC,0x00,0x00,0x00,
0xCD,0x00,0x00,0x00,0xCE,0x00,0x00,0x00,0xCF,0x00,0x00,0x00,0xD0,0x00,0x00,0x00,
0xD1,0x00,0x00,0x00,0xD2,0x00,0x00,0x00,0xD3,0x00,0x00,0x00,0xD4,0x00,0x00,0x00,
0xD5,0x00,0x00,0x00,0xD6,0x00,0x00,0x00,0xD7,0x00,0x00,0x00,0xD8,0x00,0x00,0x00,
0xD9,0x00,0x00,0x00,0xDA,0x00,0x00,0x00,0xDB,0x00,0x00,0x00,0xDC,0x00,0x00,0x00,
0xDD,0x00,0x00,0x00,0xDE,0x00,0x00,0x00,0xDF,0x00,0x00,0x00,0xE0,0x00,0x00,0x00,
0xE1,0x00,0x00,0x00,0xE2,0x00,0x00,0x00,0xE3,0x00,0x00,0x00,0xE4,0x00,0x00,0x00,
0xE5,0x00,0x00,0x00,0xE6,0x00,0x00,0x00,0xE7,0x00,0x00,0x00,0xE8,0x00,0x00,0x00,
0xE9,0x00,0x00,0x00,0xEA,0x00,0x00,0x00,0xEB,0x00,0x00,0x00,0xEC,0x00,0x00,0x00,
0xED,0x00,0x00,0x00,0xEE,0x00,0x00,0x00,0xEF,0x00,0x00,0x00,0xF0,0x00,0x00,0x00,
0xF1,0x00,0x00,0x00,0xF2,0x00,0x00,0x00,0xF3,0x00,0x00,0x00,0xF4,0x00,0x00,0x00,
0xF5,0x00,0x00,0x00,0xF6,0x00,0x00,0x00,0xF7,0x00,0x00,0x00,0xF8,0x00,0x00,0x00,
0xF9,0x00,0x00,0x00,0xFA,0x00,0x00,0x00,0xFB,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,
0xFD,0x00,0x00,0x00,0xFE,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0xFF,0xFF,0xFF,0x0F
};

static unsigned char sector4000[] =
{
0x42,0x20,0x00,0x49,0x00,0x6E,0x00,0x66,0x00,0x6F,0x00,0x0F,0x00,0x72,0x72,0x00,
0x6D,0x00,0x61,0x00,0x74,0x00,0x69,0x00,0x6F,0x00,0x00,0x00,0x6E,0x00,0x00,0x00,
0x01,0x53,0x00,0x79,0x00,0x73,0x00,0x74,0x00,0x65,0x00,0x0F,0x00,0x72,0x6D,0x00,
0x20,0x00,0x56,0x00,0x6F,0x00,0x6C,0x00,0x75,0x00,0x00,0x00,0x6D,0x00,0x65,0x00,
0x53,0x59,0x53,0x54,0x45,0x4D,0x7E,0x31,0x20,0x20,0x20,0x16,0x00,0x07,0xF9,0xB8,
0xE8,0x52,0xE8,0x52,0x00,0x00,0xFA,0xB8,0xE8,0x52,0x0C,0x00,0x00,0x00,0x00,0x00,
0xE5,0x4F,0x50,0x20,0x20,0x20,0x20,0x20,0x42,0x49,0x54,0x20,0x18,0x05,0x8A,0xB9,
0xE8,0x52,0xE8,0x52,0x00,0x00,0x53,0xB9,0xE8,0x52,0x24,0x00,0xE0,0x60,0x3A,0x00,
0x54,0x4F,0x50,0x20,0x20,0x20,0x20,0x20,0x42,0x49,0x54,0x20,0x18,0x05,0x8A,0xB9,
0xE8,0x52,0xF2,0x52,0x00,0x00,0x1C,0x7B,0xF2,0x52,0x24,0x00,0xE0,0x60,0x3A,0x00,
0x49,0x4D,0x41,0x47,0x45,0x20,0x20,0x20,0x42,0x49,0x4E,0x20,0x18,0x20,0xD3,0xB9,
0x76,0x56,0x76,0x56,0x00,0x00,0xD4,0xB9,0x76,0x56,0x99,0x00,0x48,0x26,0x33,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

//-----------------------------------------------------------------------------
//
// Returns fixed FAT32 data.
//
//-----------------------------------------------------------------------------

static void readSectors(unsigned int sector, unsigned int count, unsigned char *buff)
{
    printf("Reading sector %08x count %u\n", sector, count);

    for (int s = 0; s < count; ++s)
    {
        switch (sector+s)
        {
        case 0x0000:
            for (int i = 0; i < 512; ++i)
                buff[i] = sector0000[i];
            break;
        case 0x2000:
            for (int i = 0; i < 512; ++i)
                buff[i] = sector2000[i];
            break;
        case 0x2257:
            for (int i = 0; i < 512; ++i)
                buff[i] = sector2257[i];
            break;
        case 0x4000:
            for (int i = 0; i < 512; ++i)
                buff[i] = sector4000[i];
            break;
        default:    // Fill all other sectors with a fixed value
            for (int i = 0; i < 512; ++i)
                buff[i] = sector+s;
            break;
        }

        buff += 512;
    }
}
#endif

//-----------------------------------------------------------------------------
//
// Handle unaligned accesses. Unfortunately some of the variables in the boot
// sector are not aligned to short/int boundaries.
//
//-----------------------------------------------------------------------------

static unsigned short readUnalignedShort(const unsigned short *s)
{
	unsigned char *ptr = (unsigned char *)s;

	return (ptr[1] << 8) | ptr[0];
}


static unsigned int readUnalignedInt(const unsigned int *i)	
{
	unsigned char *ptr = (unsigned char *)i;

	return (ptr[3] << 24) | (ptr[2] << 16) | (ptr[1] << 8) | ptr[0];
}


//-----------------------------------------------------------------------------
//
// Read a sector into the cache (if it's not already in there).
//
//-----------------------------------------------------------------------------

static void loadCache(struct CardInfo *cinfo, unsigned int sector)
{
	if (sector != cinfo->cachedSector)
	{
		readSectors(sector, 1, cinfo->cache);
		cinfo->cachedSector = sector;
	}
}


//-----------------------------------------------------------------------------
//
// Load all the information we need from the FAT.
//
//-----------------------------------------------------------------------------

static void readFat(struct CardInfo *cinfo, unsigned int fatSector)
{
	unsigned int sectorsPerFat   = readUnalignedShort(&cinfo->mbr.sectorsPerFat);
	unsigned int fatSize         = sectorsPerFat * cinfo->mbr.numFats;
	unsigned int   totalSectors    = cinfo->mbr.totalSectors;
	unsigned short reservedSectors = cinfo->mbr.reservedSectors;

	cinfo->sectorsPerCluster = cinfo->mbr.clusterSize;
	unsigned int numClusters = (totalSectors - reservedSectors - fatSize) / cinfo->sectorsPerCluster;			/* Number of clusters */

	/* Boundaries and Limits */
	cinfo->fatClusters = numClusters + 2;						/* Number of FAT entries */
	cinfo->startFat  = fatSector + reservedSectors; 					/* FAT start sector */
	cinfo->startData = fatSector + reservedSectors + fatSize;					/* Data start sector */
	cinfo->startRootDir = cinfo->mbr.rootDirStart;
}


//-----------------------------------------------------------------------------
//
// Finds the first partition on the card that is formatted as FAT32 and reads
// in it's information.
//
//-----------------------------------------------------------------------------

static void loadFat(struct CardInfo *cinfo)
{
    const int NumPartitions = 4;

	// Make sure the boot sector is formatted correctly

    cinfo->cachedSector = -1;   // Force load
	loadCache(cinfo, 0);

	if (cinfo->mbr.bootSig != 0xaa55)
		error("missing fat");

    // Check all the partitions. Have to store the partition sectors
    // beforehand or otherwise they will be wiped out when we reload
    // the cache.

	unsigned int partition_lba[NumPartitions];

	for (int i = 0; i < NumPartitions; ++i)
		partition_lba[i] = readUnalignedInt(&cinfo->mbr.partition[i].startLba);

	for (int i = 0; i < NumPartitions; ++i)
	{
		loadCache(cinfo, partition_lba[i]);

		unsigned char b = cinfo->mbr.jmpBoot;

        // Is the partition formatted for FAT32?

		if ((cinfo->mbr.bootSig == 0xaa55) &&
		    ((b == 0xeb) || (b == 0xe9) || (b == 0xe8)) &&
			strEqual(cinfo->mbr.fileSystemType, "FAT32   ", 8))
        {
			readFat(cinfo, partition_lba[i]);
            return;
		}
	}

    // We didn't find a FAT formatted partition

	error("missing fat");
}


//-----------------------------------------------------------------------------
//
// Convert a cluster index to a sector on the disk.
//
//-----------------------------------------------------------------------------

static unsigned int clusterToSector(struct CardInfo *cinfo, unsigned int cluster)
{
	return cinfo->startData + cinfo->sectorsPerCluster * (cluster - 2);
}


//-----------------------------------------------------------------------------
//
// Get the next cluster in the FAT linked-list.
//
//-----------------------------------------------------------------------------

static unsigned int getNextCluster(struct CardInfo *cinfo, unsigned int cluster)
{
	loadCache(cinfo, cinfo->startFat + (cluster * 4 / SECTOR_SIZE));

	return cinfo->cache[cluster * 4 % SECTOR_SIZE] & 0x0FFFFFFF;	/* Simple unsigned int array but mask out upper 4 bits */
}


//-----------------------------------------------------------------------------
//
// Find the location of the file in the root directory struture.
//
//-----------------------------------------------------------------------------

static int findFile(struct CardInfo *cinfo, const char *filename)
{
	unsigned int cluster = cinfo->startRootDir;				// Start at the beginning

	// Search each cluster

	while (cluster < cinfo->fatClusters)
	{
		unsigned int sector = clusterToSector(cinfo, cluster);	

		for (int s = 0; s < cinfo->sectorsPerCluster; ++s)
		{
			loadCache(cinfo, sector++);

			for (int i = 0; i < SECTOR_SIZE/sizeof(struct DirEntry); ++i)
			{
                // End of the table?

				if (cinfo->dirs[i].name[0] == 0)
					error("no image");

                // Found the file?

				if (strEqual(cinfo->dirs[i].name, filename, 11))
					return i;
			}
		}

		// Move to the next cluster

		cluster = getNextCluster(cinfo, cluster);
	}

	error("no image");
}


//-----------------------------------------------------------------------------
//
// Reads the entire contents of a file from the CF card into memory. No
// validation checks are performed to ensure that the memory has enough space
// for the file. So be careful you don't overwrite anything. Also, the last
// sector of the file is read in in it's entirity, even if it is not full.
//
//-----------------------------------------------------------------------------

static unsigned int readFile(struct CardInfo *cinfo, const char *filename, unsigned char *ptr)
{
    int i = findFile(cinfo, filename);

	unsigned int cluster = cinfo->dirs[i].lowCluster | (cinfo->dirs[i].highCluster << 16);
	unsigned int fileSize = cinfo->dirs[i].fileSize;
	int remaining = fileSize;   // Won't handle files > 2Gbytes :)

	do
	{
        // Find the first sector in the cluster

		unsigned int sector = clusterToSector(cinfo, cluster);

        // How many more sectors do we need to read? Make sure to round up

		int sectors = (remaining + SECTOR_SIZE - 1) / SECTOR_SIZE;
		if (sectors > cinfo->sectorsPerCluster) // Clip to the size of a cluster
			sectors = cinfo->sectorsPerCluster;

		readSectors(sector, sectors, ptr);

		remaining -= SECTOR_SIZE * sectors;
		ptr       += SECTOR_SIZE * sectors;

        // Anything more to do?

        if (remaining > 0)
			cluster = getNextCluster(cinfo, cluster);
	}
	while (remaining > 0);

    printStr("\r\n");
	return fileSize;
}


//-----------------------------------------------------------------------------
//
// Initialize the DTB so linux knows where everything is.
//
//-----------------------------------------------------------------------------

#include "dtb.h"

static void initdtb()
{
#if !SIM
    unsigned char *dtb = (unsigned char *)0x83fffa00;

    for (int i = 0; i < sizeof(default64mbdtb); ++i)
        dtb[i] = default64mbdtb[i];
#endif
}


//-----------------------------------------------------------------------------
//
// The linux image reads from some uninitialized memory locations. This
// function zeros them to make tracing consistent and easier to correlate with
// the emulator.
//
//-----------------------------------------------------------------------------

static void clearMemory()
{
    unsigned int *p = (unsigned int *)0x80402bb8;

    while (p <= (unsigned int *)0x8040f53c)
        *(p++) = 0;

    p = (unsigned int *)0x8043a678;

    while (p <= (unsigned int *)0x8043e2fc)
        *(p++) = 0;
}

#if !SIM
//-----------------------------------------------------------------------------
//
// Load the image to be executed from the CF card.
//
//-----------------------------------------------------------------------------

static void bootFromCard()
{
    struct CardInfo cinfo;

    printStr("Booting");

	initSpi();
	loadFat(&cinfo);
    (void)readFile(&cinfo, "IMAGE   BIN", memory);
    initdtb();
}


//-----------------------------------------------------------------------------
//
// Load the code to be executed from the serial port.
//
//-----------------------------------------------------------------------------

static void bootFromSerial()
{
    error("not yet implemented");
}


//-----------------------------------------------------------------------------
//
// Load the code to be executed from the serial port.
//
//-----------------------------------------------------------------------------

__attribute__((optimize("O0"))) static void performanceTest()
{
    printStr("Measuring performance\r\n");

    while (1)
    {
        unsigned int sum = 0;

        for (int i = 0; i < 1000000; ++i)
            sum += i;

        unsigned int mcycle;
        unsigned int minstret;

        CSR_READ(mcycle,   0xf00);
        CSR_READ(minstret, 0xf01);

        printStr("mcycle ");
        printInt(mcycle);
        printStr(", minstret ");
        printInt(minstret);
        printStr(", sum ");
        printInt(sum);
        printStr("\r\n");
    }
}


//-----------------------------------------------------------------------------
//
// Load the code to be executed from the serial port.
//
//-----------------------------------------------------------------------------

static void memoryTest()
{
    printStr("Memory test\r\n");

    const int Size = 128*1024;

    for (int s = 0; s <=24; ++s)
    {
        printInt(s);

        unsigned char *ptr = memory;
        for (int i = 0; i < Size; ++i)
            ptr[i] = (i >> s);

        for (int i = 0; i < Size; ++i)
        {
            if (ptr[i] != ((i >> s) & 0xff))
                error("readback");
        }

        printStr(" ok\r\n");
    }
}


//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

int main()
{
    printStr("RISC-V by Colm Gavin\r\n");

    switch (*switches & 0x03)
    {
        case 0:
            bootFromCard();
            break;
        case 1:
            bootFromSerial();
            break;
        case 2:
            performanceTest();  // Will (and must) never return
            break;
        case 3:
            while (1)
                memoryTest();   // Must never return
            break;
    }

    // When main returns, the assembly code will jump to 0x80000000
}
#else
int main()
{
    printStr("RISC-V by Colm Gavin\r\nBooting\r\n");

    struct CardInfo cinfo;
    char *memory = malloc(4*1024*1024);

	initSpi();
	loadFat(&cinfo);
    unsigned int size = readFile(&cinfo, "IMAGE   BIN", memory);
    initdtb();

    // Confirm everything we expect was read

    printf("Read %d bytes\n", size);

    if (size != 3352136)
        printf("ERROR!\n");

    unsigned int sum = 0;
    for (int i = 0; i < 3352136; ++i)
        sum += memory[i];

    printf("Sum of memory = %u\n", sum);

    if (sum != 4294012248)
        printf("ERROR!\n");
}
#endif
