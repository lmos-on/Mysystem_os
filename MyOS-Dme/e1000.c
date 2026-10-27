#include <stdint.h>
#include <stddef.h>
#include "e1000.h"
#include "pci.h"
#include "pmm.h"
#include "vmm.h"
#include "pit.h"
#include "kprintf.h"
#include "net.h"

/* ----------------------------------------------------------------------------
 *  Lomsourt OS - Intel 82540EM (e1000) driver : implementation (Step 7.1)
 *
 *  ADDRESSING DISCIPLINE (three addresses per object, never confused)
 *    phys  PMM-owned DMA address programmed into descriptor BAR/ring fields.
 *    vaddr CPU view of the same byte: low RAM is identity-mapped, so for
 *          descriptor rings and packet buffers vaddr == phys.
 *    mmio  BAR0 register file, mapped 1:1 (VA == PA) page by page through
 *          vmm_map_page on the live PML4 (same mechanism as the ACPI table
 *          walk in Step 6.1).  No PAT/MTRR exists yet, so the mapping is
 *          write-back; QEMU intercepts MMIO by address, which is exact.
 *
 *  BRING-UP ORDER (Intel 8254x manual, software-init chapter)
 *    MAC -> CTRL reset -> link (SLU|ASDE) -> clear MTA -> RX ring -> TX ring
 *    -> link-status check.  Interrupts stay masked (polling driver).
 * ------------------------------------------------------------------------- */

/* CTRL / STATUS bits. */
#define CTRL_RST    (1u << 26)
#define CTRL_ASDE   (1u << 5)
#define CTRL_SLU    (1u << 6)
#define STATUS_LU   (1u << 1)

/* EERD bits. */
#define EERD_START  (1u << 0)
#define EERD_DONE   (1u << 1)

/* RCTL: EN|UPE|MPE|BAM, 2048-byte buffers (BSIZE 00, BSEX 0), strip CRC. */
#define RCTL_EN     (1u << 1)
#define RCTL_UPE    (1u << 3)
#define RCTL_MPE    (1u << 4)
#define RCTL_BAM    (1u << 15)
#define RCTL_SECRC  (1u << 26)

/* TCTL: EN|PSP, collision threshold 0x10, collision distance 0x40. */
#define TCTL_EN     (1u << 1)
#define TCTL_PSP    (1u << 3)

/* TX descriptor CMD bits. */
#define TXD_CMD_EOP   0x01u
#define TXD_CMD_IFCS  0x02u
#define TXD_CMD_RS    0x08u
#define TXD_STAT_DD   0x01u
#define RXD_STAT_DD   0x01u
#define RXD_STAT_EOP  0x02u

struct e1000_rx_desc {
    uint64_t addr;
    uint16_t length;
    uint16_t checksum;
    uint8_t  status;
    uint8_t  errors;
    uint16_t special;
};

struct e1000_tx_desc {
    uint64_t addr;
    uint16_t length;
    uint8_t  cso;
    uint8_t  cmd;
    uint8_t  status;
    uint8_t  css;
    uint16_t special;
};

uint8_t g_e1000_mac[6];
int     g_e1000_ready;

static volatile uint32_t* s_mmio;             /* BAR0 register file (VA)      */
static struct e1000_rx_desc* s_rx;            /* RX ring (identity VA)        */
static struct e1000_tx_desc* s_tx;            /* TX ring (identity VA)        */
static uint64_t s_rx_phys[E1000_RX_COUNT];    /* RX buffer physical pages     */
static uint64_t s_tx_phys[E1000_TX_COUNT];    /* TX buffer physical pages     */
static unsigned s_tx_cur;

static e1000_rx_handler_t s_rx_handler;

void e1000_set_rx_handler(e1000_rx_handler_t h)
{
    s_rx_handler = h;
}

static uint32_t e1000_r32(uint32_t reg)
{
    return s_mmio[reg / 4u];
}

static void e1000_w32(uint32_t reg, uint32_t value)
{
    s_mmio[reg / 4u] = value;
}

static void e1000_wmb(void)
{
    __asm__ volatile ("" ::: "memory");
}

/* memset over the identity map (freestanding kernel: no libc). */
static void e1000_zero(void* p, uint32_t n)
{
    volatile uint8_t* q = (volatile uint8_t*)p;
    uint32_t i;
    for (i = 0; i < n; i++)
        q[i] = 0;
}

/* Read one EEPROM word via EERD; returns 1 on success. */
static int e1000_eerd(uint8_t word, uint16_t* out)
{
    unsigned spin;

    e1000_w32(E1000_REG_EERD, 1u | ((uint32_t)word << 2));
    for (spin = 0; spin < 100000u; spin++) {
        uint32_t v = e1000_r32(E1000_REG_EERD);
        if (v & EERD_DONE) {
            *out = (uint16_t)(v >> 16);
            return 1;
        }
    }
    return 0;
}

static void e1000_read_mac(void)
{
    uint16_t w0 = 0, w1 = 0, w2 = 0;
    const char* src;

    if (e1000_eerd(0, &w0) && e1000_eerd(1, &w1) && e1000_eerd(2, &w2)) {
        src = "EEPROM";
        g_e1000_mac[0] = (uint8_t)(w0 & 0xFFu);
        g_e1000_mac[1] = (uint8_t)(w0 >> 8);
        g_e1000_mac[2] = (uint8_t)(w1 & 0xFFu);
        g_e1000_mac[3] = (uint8_t)(w1 >> 8);
        g_e1000_mac[4] = (uint8_t)(w2 & 0xFFu);
        g_e1000_mac[5] = (uint8_t)(w2 >> 8);
    } else {
        uint32_t ral, rah;
        src = "RAL/RAH";
        ral = e1000_r32(E1000_REG_RAL0);
        rah = e1000_r32(E1000_REG_RAH0);
        g_e1000_mac[0] = (uint8_t)(ral & 0xFFu);
        g_e1000_mac[1] = (uint8_t)((ral >> 8) & 0xFFu);
        g_e1000_mac[2] = (uint8_t)((ral >> 16) & 0xFFu);
        g_e1000_mac[3] = (uint8_t)((ral >> 24) & 0xFFu);
        g_e1000_mac[4] = (uint8_t)(rah & 0xFFu);
        g_e1000_mac[5] = (uint8_t)((rah >> 8) & 0xFFu);
    }

    kprintf("E1000: MAC %x:%x:%x:%x:%x:%x (via %s)\r\n",
            (unsigned)g_e1000_mac[0], (unsigned)g_e1000_mac[1],
            (unsigned)g_e1000_mac[2], (unsigned)g_e1000_mac[3],
            (unsigned)g_e1000_mac[4], (unsigned)g_e1000_mac[5], src);
}

static uint32_t s_mac_seed = 0;

static void s_mac_seed_init(void)
{
    s_mac_seed = (uint32_t)pmm_get_free_memory() ^
                 (uint32_t)timer_ticks ^
                 (uint32_t)(uintptr_t)&s_mac_seed;
}

static uint8_t s_mac_rand(void)
{
    s_mac_seed = s_mac_seed * 1103515245u + 12345u;
    return (uint8_t)((s_mac_seed >> 16) & 0xFFu);
}

int e1000_spoof_mac(void)
{
    uint8_t new_mac[6];
    unsigned i;
    uint32_t ral, rah;

    if (!g_e1000_ready)
        return -1;

    s_mac_seed_init();

    new_mac[0] = 0x02;
    new_mac[1] = s_mac_rand();
    new_mac[2] = s_mac_rand();
    new_mac[3] = s_mac_rand();
    new_mac[4] = s_mac_rand();
    new_mac[5] = s_mac_rand();

    for (i = 0; i < 6u; i++)
        g_e1000_mac[i] = new_mac[i];

    ral = ((uint32_t)new_mac[3] << 24) | ((uint32_t)new_mac[2] << 16) |
          ((uint32_t)new_mac[1] << 8) | new_mac[0];
    rah = ((uint32_t)new_mac[5] << 8) | new_mac[4] | 0x80000000u;

    e1000_w32(E1000_REG_RAL0, ral);
    e1000_w32(E1000_REG_RAH0, rah);

    kprintf("[NET] DYNAMIC MAC SPOOFED: %x:%x:%x:%x:%x:%x\r\n",
            (unsigned)new_mac[0], (unsigned)new_mac[1],
            (unsigned)new_mac[2], (unsigned)new_mac[3],
            (unsigned)new_mac[4], (unsigned)new_mac[5]);

    return 0;
}

const uint8_t* e1000_rotate_mac(void)
{
    e1000_spoof_mac();
    for (unsigned i = 0; i < 6u; i++)
        g_net_mac[i] = g_e1000_mac[i];
    return g_net_mac;
}

int e1000_init(void)
{
    uint64_t  bar;
    uint64_t  ring_phys;
    unsigned  i;
    unsigned  spin;
    uint32_t  v;

    g_e1000_ready = 0;

    if (!g_e1000_found) {
        kprintf("E1000: no PCI device latched - cannot init\r\n");
        return -1;
    }
    if (g_e1000_dev.bar0_is_io || g_e1000_dev.bar0 == 0) {
        kprintf("E1000: BAR0 is not usable MMIO (raw 0x%x)\r\n",
                (unsigned)g_e1000_dev.bar0);
        return -1;
    }

    /* 0. PCI COMMAND: enable I/O + MMIO + BUS MASTER.  SeaBIOS usually
     * leaves these set, but a driver must never assume it - with bus
     * mastering off the card cannot DMA its descriptor rings (the Step 7.1
     * bring-up failure: TDH advanced with no DD and no packet, exactly like
     * a master-aborted DMA engine). */
    {
        uint16_t cmd = pci_read_u16(g_e1000_dev.bus, g_e1000_dev.slot,
                                    g_e1000_dev.func, 0x04);
        kprintf("E1000: PCI COMMAND before = 0x%x\r\n", (unsigned)cmd);
        pci_write_u16(g_e1000_dev.bus, g_e1000_dev.slot, g_e1000_dev.func,
                      0x04, (uint16_t)(cmd | 0x07u));
        kprintf("E1000: PCI COMMAND after  = 0x%x\r\n",
                (unsigned)pci_read_u16(g_e1000_dev.bus, g_e1000_dev.slot,
                                       g_e1000_dev.func, 0x04));
    }

    /* 1. Map the 128 KiB BAR0 window 1:1 into the kernel address space. */
    bar = (uint64_t)g_e1000_dev.bar0;
    for (i = 0; i < E1000_MMIO_LEN; i += 0x1000u)
        vmm_map_page(bar + i, bar + i, PAGE_WRITABLE);
    s_mmio = (volatile uint32_t*)(uintptr_t)bar;
    kprintf("E1000: BAR0 mmio 0x%x mapped (%u pages)\r\n",
            (unsigned)g_e1000_dev.bar0, (unsigned)(E1000_MMIO_LEN / 0x1000u));

    /* 2. MAC address. */
    e1000_read_mac();

    /* 3. Reset, then set link up. */
    e1000_w32(E1000_REG_CTRL, e1000_r32(E1000_REG_CTRL) | CTRL_RST);
    for (spin = 0; spin < 100000u; spin++) {
        if (!(e1000_r32(E1000_REG_CTRL) & CTRL_RST))
            break;
    }
    e1000_w32(E1000_REG_CTRL, e1000_r32(E1000_REG_CTRL) | CTRL_SLU | CTRL_ASDE);

    /* 4. Clear the 128-entry multicast table array. */
    for (i = 0; i < 128u; i++)
        e1000_w32(E1000_REG_MTA + i * 4u, 0);

    /* 5. RX ring: one page of descriptors + one page per 2 KiB buffer. */
    ring_phys = (uint64_t)(uintptr_t)pmm_alloc_block();
    if (!ring_phys) {
        kprintf("E1000: out of memory for RX ring\r\n");
        return -1;
    }
    s_rx = (struct e1000_rx_desc*)(uintptr_t)ring_phys;
    e1000_zero(s_rx, 0x1000u);
    for (i = 0; i < E1000_RX_COUNT; i++) {
        uint64_t buf = (uint64_t)(uintptr_t)pmm_alloc_block();
        if (!buf) {
            kprintf("E1000: out of memory for RX buffers\r\n");
            return -1;
        }
        s_rx_phys[i]    = buf;
        s_rx[i].addr    = buf;
        s_rx[i].status  = 0;                  /* owned by the hardware        */
    }
    e1000_w32(E1000_REG_RDBAL, (uint32_t)ring_phys);
    e1000_w32(E1000_REG_RDBAH, (uint32_t)(ring_phys >> 32));
    e1000_w32(E1000_REG_RDLEN, E1000_RX_COUNT * 16u);
    e1000_w32(E1000_REG_RDH, 0);
    e1000_w32(E1000_REG_RDT, E1000_RX_COUNT - 1u);
    e1000_w32(E1000_REG_RCTL, RCTL_EN | RCTL_UPE | RCTL_MPE | RCTL_BAM |
                              RCTL_SECRC);
    kprintf("E1000: RX ring ready (%u x 2048 B, RCTL=0x%x)\r\n",
            (unsigned)E1000_RX_COUNT,
            (unsigned)e1000_r32(E1000_REG_RCTL));

    /* 6. TX ring: descriptors start life DONE (owned by software). */
    ring_phys = (uint64_t)(uintptr_t)pmm_alloc_block();
    if (!ring_phys) {
        kprintf("E1000: out of memory for TX ring\r\n");
        return -1;
    }
    s_tx = (struct e1000_tx_desc*)(uintptr_t)ring_phys;
    e1000_zero(s_tx, 0x1000u);
    for (i = 0; i < E1000_TX_COUNT; i++) {
        uint64_t buf = (uint64_t)(uintptr_t)pmm_alloc_block();
        if (!buf) {
            kprintf("E1000: out of memory for TX buffers\r\n");
            return -1;
        }
        s_tx_phys[i]    = buf;
        s_tx[i].addr    = buf;
        s_tx[i].status  = TXD_STAT_DD;
    }
    s_tx_cur = 0;
    e1000_w32(E1000_REG_TDBAL, (uint32_t)ring_phys);
    e1000_w32(E1000_REG_TDBAH, (uint32_t)(ring_phys >> 32));
    e1000_w32(E1000_REG_TDLEN, E1000_TX_COUNT * 16u);
    e1000_w32(E1000_REG_TDH, 0);
    e1000_w32(E1000_REG_TDT, 0);
    e1000_w32(E1000_REG_TIPG, 10u | (10u << 10) | (10u << 20));
    e1000_w32(E1000_REG_TCTL, TCTL_EN | TCTL_PSP | (0x10u << 4) | (0x40u << 12));
    kprintf("E1000: TX ring ready (%u desc, TCTL=0x%x)\r\n",
            (unsigned)E1000_TX_COUNT,
            (unsigned)e1000_r32(E1000_REG_TCTL));

    /* 7. Link state (informational; TX completion is the real verdict). */
    v = 0;
    for (i = 0; i < 200u; i++) {
        v = e1000_r32(E1000_REG_STATUS);
        if (v & STATUS_LU)
            break;
        sleep_ms(10);
    }
    kprintf("E1000: link %s (STATUS=0x%x)\r\n",
            (v & STATUS_LU) ? "UP" : "DOWN", (unsigned)v);

    g_e1000_ready = 1;
    return 0;
}

int e1000_send_packet(const uint8_t* data, uint16_t len)
{
    /* NOTE: the ring lives in DMA-shared memory - the NIC writes the status
     * byte behind our back, so every poll MUST be a volatile load.  A plain
     * struct pointer lets -O2 hoist the load out of the loop, which reads a
     * stale "not done" forever (the Step 7.1 bring-up failure). */
    volatile struct e1000_tx_desc* d;
    volatile uint8_t*               buf;
    unsigned                        spin;
    uint16_t                        i;

    if (!g_e1000_ready || !data || len == 0 || len > 1518u)
        return -1;

    d = (volatile struct e1000_tx_desc*)&s_tx[s_tx_cur];
    if (!(d->status & TXD_STAT_DD))
        return -1;                            /* ring full (never blocks)    */

    buf = (volatile uint8_t*)(uintptr_t)s_tx_phys[s_tx_cur];
    for (i = 0; i < len; i++)
        buf[i] = data[i];

    d->length = len;
    d->cmd    = (uint8_t)(TXD_CMD_EOP | TXD_CMD_IFCS | TXD_CMD_RS);
    d->status = 0;
    e1000_wmb();
    e1000_w32(E1000_REG_TDT, (s_tx_cur + 1u) % E1000_TX_COUNT);

    for (spin = 0; spin < 100000u; spin++) {
        if (d->status & TXD_STAT_DD) {
            s_tx_cur = (s_tx_cur + 1u) % E1000_TX_COUNT;
            return 0;
        }
    }
    /* Timeout policy: step over the dead descriptor instead of wedging the
     * ring on it forever.  A subsequently-completing DMA then reports DD on
     * an already-retired slot (harmless - we never re-arm without checking),
     * while forward progress - and a second, independent kick - stays
     * possible. */
    s_tx_cur = (s_tx_cur + 1u) % E1000_TX_COUNT;
    return -1;                                /* kicked but never completed  */
}

void e1000_debug_dump(void)
{
    volatile struct e1000_tx_desc* d =
        (volatile struct e1000_tx_desc*)&s_tx[s_tx_cur];

    if (!g_e1000_ready && !s_mmio)
        return;
    kprintf("E1000 DBG: TDH=0x%x TDT=0x%x TCTL=0x%x STATUS=0x%x\r\n",
            (unsigned)e1000_r32(E1000_REG_TDH),
            (unsigned)e1000_r32(E1000_REG_TDT),
            (unsigned)e1000_r32(E1000_REG_TCTL),
            (unsigned)e1000_r32(E1000_REG_STATUS));
    kprintf("E1000 DBG: RDH=0x%x RDT=0x%x RCTL=0x%x\r\n",
            (unsigned)e1000_r32(E1000_REG_RDH),
            (unsigned)e1000_r32(E1000_REG_RDT),
            (unsigned)e1000_r32(E1000_REG_RCTL));
    kprintf("E1000 DBG: tx_cur=%u desc{addr=0x%x len=%u cmd=0x%x status=0x%x}\r\n",
            s_tx_cur, (unsigned)(uint32_t)(d->addr & 0xFFFFFFFFu),
            (unsigned)d->length, (unsigned)d->cmd, (unsigned)d->status);
    kprintf("E1000 DBG: TDBAL=0x%x RDBAL=0x%x CR3=0x%x resolve(txring)=0x%x\r\n",
            (unsigned)e1000_r32(E1000_REG_TDBAL),
            (unsigned)e1000_r32(E1000_REG_RDBAL),
            (unsigned)vmm_current_cr3(),
            (unsigned)vmm_resolve(VMM_PML4_BASE,
                                  (uint64_t)(uintptr_t)s_tx));
}

void e1000_poll_rx(void)
{
    unsigned i;
    unsigned got = 0;

    if (!g_e1000_ready)
        return;

    for (i = 0; i < E1000_RX_COUNT; i++) {
        volatile struct e1000_rx_desc* r =
            (volatile struct e1000_rx_desc*)&s_rx[i];

        if (r->status & RXD_STAT_DD) {
            const volatile uint8_t* b =
                (const volatile uint8_t*)(uintptr_t)s_rx_phys[i];
            unsigned dump = r->length < 16u ? r->length : 16u;
            unsigned k;

            kprintf("E1000: RX frame %u bytes%s: ", (unsigned)r->length,
                    (r->status & RXD_STAT_EOP) ? "" : " (chained!)");
            for (k = 0; k < dump; k++)
                kprintf("%x ", (unsigned)b[k]);
            kprintf("\r\n");
            got++;

            if (s_rx_handler)
                s_rx_handler((const uint8_t*)b, r->length);

            r->status = 0;                    /* hand back to the hardware   */
            e1000_w32(E1000_REG_RDT, i);
        }
    }
    if (!got)
        kprintf("E1000: RX ring polled - no frames waiting\r\n");
}