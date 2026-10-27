#include <stdint.h>
#include "ata.h"
#include "kprintf.h"

/* ----------------------------------------------------------------------------
 *  Lomsourt OS - ATA / IDE PIO driver : implementation (Step 3.3)
 *
 *  COMMAND FLOW (identical shape for READ, WRITE, IDENTIFY):
 *
 *   1.  SELECT + SET-UP LATCHES (port writes, in this order):
 *         Drive/Select  <- 0xE0 | (lba >> 24)          (or plain 0xE0)
 *         Sector Count  <- 1
 *         LBA Low/Mid   <- lba bytes 0..2
 *         Command       <- 0x20 / 0x30 / 0xEC
 *   2.  POLL the Status register (0x1F7) until the drive is ready, then
 *       until DRQ (Data Request) raises: the host may now stream words.
 *   3.  TRANSFER DATA over the 16-bit PIO bus: inw/outw to the Data port,
 *       one 16-bit word per access, 256 accesses per 512-byte sector.
 *   4.  COMPLETE: poll Status until BSY clears; a write is followed by a
 *       CACHE FLUSH (0xE7) so the data is on the platter, not in a buffer.
 *
 *  POLLING + RESET DELAYS (the ATA specification's 400 ns rule):
 *  After a drive SELECT you must wait >= 400 ns for the drive's registers to
 *  settle before touching the command latch.  No timer is needed: four reads
 *  of the Status port take ~1 us of bus turnaround each, which is the
 *  canonical "drive pulse" delay used by hobby and Linux drivers alike
 *  (ata_iowait below).  Every poll loop also carries a bounded counter so a
 *  missing drive (register reads of 0x00/0xFF forever) degrades to an error
 *  return instead of an infinite stall.
 *
 *  WORD / BYTE ORDER: each 16-bit Data-port word is little-endian; word[i]
 *  supplies buffer[2i] (low byte) then buffer[2i+1] (high byte).  IDENTIFY's
 *  string fields are the same two-bytes-per-word packing, but with the HIGH
 *  byte leading (the "ASCII big-endian" convention of the spec).
 * ------------------------------------------------------------------------- */

/* ---- Minimal in/out helpers (freestanding, no libc) --------------------- */
static inline uint8_t inb(uint16_t port)
{
    uint8_t v;
    __asm__ volatile ("inb %1, %0" : "=a"(v) : "Nd"(port));
    return v;
}

static inline void outb(uint16_t port, uint8_t value)
{
    __asm__ volatile ("outb %0, %1" :: "a"(value), "Nd"(port));
}

static inline uint16_t inw(uint16_t port)
{
    uint16_t v;
    __asm__ volatile ("inw %1, %0" : "=a"(v) : "Nd"(port));
    return v;
}

static inline void outw(uint16_t port, uint16_t value)
{
    __asm__ volatile ("outw %0, %1" :: "a"(value), "Nd"(port));
}

/* ~400 ns settle-time pulse for the drive/head register. */
static void ata_iowait(void)
{
    inb(ATA_STATUS);
    inb(ATA_STATUS);
    inb(ATA_STATUS);
    inb(ATA_STATUS);
}

/* Poll Status until the drive is idle AND ready.  Returns 0 on ready, -1 on
 * timeout or a raised error flag. */
static int ata_wait_ready(void)
{
    unsigned guard = ATA_POLL_TIMEOUT;

    while (guard--) {
        uint8_t st = inb(ATA_STATUS);
        if (st & ATA_STAT_ERR)              /* a latched error means stop    */
            return -1;
        if (!(st & ATA_STAT_BSY) && (st & ATA_STAT_DRDY))
            return 0;
    }
    return -1;
}

/* Poll Status until DRQ is asserted (a 256-word transfer is now expected).
 * Returns 0 when the data phase is open, -1 on error/timeout. */
static int ata_wait_drq(void)
{
    unsigned guard = ATA_POLL_TIMEOUT;

    while (guard--) {
        uint8_t st = inb(ATA_STATUS);
        if (st & ATA_STAT_DRQ)
            return 0;
        if (st & ATA_STAT_ERR)              /* command rejected              */
            return -1;
    }
    return -1;
}

/* Program the LBA28 register block and latch the given command.  The drive
 * select byte is written first, followed by the mandated settle delay. */
static void ata_setup(uint32_t lba, uint8_t sectors, uint8_t cmd)
{
    outb(ATA_DRIVE_SELECT, (uint8_t)(ATA_DRIVE_MASTER | ((lba >> 24) & 0x0F)));
    ata_iowait();                           /* 400 ns drive pulse            */
    outb(ATA_SECTOR_COUNT, sectors);
    outb(ATA_LBA_LO, (uint8_t)(lba >>  0));
    outb(ATA_LBA_MID, (uint8_t)(lba >>  8));
    outb(ATA_LBA_HI,  (uint8_t)(lba >> 16));
    outb(ATA_COMMAND, cmd);
}

/* Issue IDENTIFY, verify a drive is present, and log its identity block.
 * Returns 0 when a drive answered, -1 when the bus is empty. */
int ata_init(void)
{
    uint16_t id[ATA_SECTOR_WORDS];
    unsigned i;

    kprintf("ATA: probing primary master (IDENTIFY 0xEC)...\r\n");

    if (ata_wait_ready() < 0) {                 /* bus idle before selecting */
        kprintf("ATA: no drive answered (bus never ready)\r\n");
        return -1;
    }
    outb(ATA_DRIVE_SELECT, ATA_DRIVE_MASTER);   /* select + fix unit          */
    ata_iowait();                               /* 400 ns settle             */
    outb(ATA_COMMAND, ATA_CMD_IDENTIFY);

    if (ata_wait_drq() < 0) {
        kprintf("ATA: no drive answered (no DRQ) - bus empty or absent\r\n");
        return -1;                              /* nothing attached           */
    }

    for (i = 0; i < ATA_SECTOR_WORDS; i++)      /* drain the 256-word block   */
        id[i] = inw(ATA_DATA);

    if (ata_wait_ready() < 0)
        kprintf("ATA: identify completion status unclear\r\n");

    /* ---- Decode a few fields worth logging ------------------------------
     * id[10..19] serial   (20 chars, words high-byte-first)
     * id[27..46] model    (40 chars, words high-byte-first)
     * id[60..61] LBA28 capacity in sectors (little-endian 32-bit). */
    {
        char model[41];
        char serial[21];
        uint32_t cap = (uint32_t)id[60] | ((uint32_t)id[61] << 16);

        /* serial = words 10..19 (20 chars), model = words 27..46 (40 chars);
         * each string byte is the HIGH byte of its 16-bit word first.  Two
         * separate loops: serial is only 10 words, running i to 20 there
         * would walk PAST the stack array and stomp the frame. */
        for (i = 0; i < 10; i++) {
            uint16_t s = id[10 + i];
            serial[i * 2]     = (char)(s >> 8);
            serial[i * 2 + 1] = (char)(s & 0xFF);
        }
        for (i = 0; i < 20; i++) {
            uint16_t m = id[27 + i];
            model[i * 2]      = (char)(m >> 8);
            model[i * 2 + 1]  = (char)(m & 0xFF);
        }
        serial[20] = '\0';
        model[40]  = '\0';

        kprintf("ATA: drive present: \"%s\" model \"%s\" LBA28 sectors=%u\r\n",
                serial, model, cap);
    }

    return 0;
}

/* PIO Read Sectors (0x20): 512 bytes from `lba` into `buffer`.
 * Returns 0 on success, -1 on any polling/transfer failure. */
int ata_read_sector(uint32_t lba, uint8_t* buffer)
{
    unsigned i;

    if (ata_wait_ready() < 0)
        return -1;

    ata_setup(lba, 1, ATA_CMD_READ);

    if (ata_wait_drq() < 0)
        return -1;

    for (i = 0; i < ATA_SECTOR_WORDS; i++) {
        uint16_t word = inw(ATA_DATA);          /* 16-bit PIO read           */
        buffer[i * 2]     = (uint8_t)(word & 0xFF);
        buffer[i * 2 + 1] = (uint8_t)(word >> 8);
    }

    return ata_wait_ready();                    /* drive must finish the read */
}

/* PIO Write Sectors (0x30): 512 bytes from `buffer` into `lba`, then issue a
 * CACHE FLUSH (0xE7) so the sector is on media before we report success.
 * Returns 0 on success, -1 on failure. */
int ata_write_sector(uint32_t lba, const uint8_t* buffer)
{
    unsigned i;

    if (ata_wait_ready() < 0)
        return -1;

    ata_setup(lba, 1, ATA_CMD_WRITE);

    if (ata_wait_drq() < 0)
        return -1;

    for (i = 0; i < ATA_SECTOR_WORDS; i++) {
        uint16_t word = (uint16_t)(buffer[i * 2] | (buffer[i * 2 + 1] << 8));
        outw(ATA_DATA, word);                   /* 16-bit PIO write          */
    }

    if (ata_wait_ready() < 0)                   /* drive finishing the write */
        return -1;

    /* Flush the write cache: 0xE7 needs no data phase, just completion. */
    outb(ATA_COMMAND, ATA_CMD_CACHE_FLUSH);
    return ata_wait_ready();
}