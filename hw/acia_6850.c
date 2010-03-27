/*
 * 6850 ACIA
 *
 * Copyright (c) 2010 Tyler Hall.
 *
 * This code is licenced under the GPL
 */

#include <inttypes.h>

#include "hw.h"
#include "sysbus.h"

#define ACIA_CONTROL 0x1
#define ACIA_DATA 0x3

#define STATUS_IRQ      (1 << 7)
#define STATUS_PE       (1 << 6)
#define STATUS_OVRN     (1 << 5)
#define STATUS_FE       (1 << 4)
#define STATUS_CTS      (1 << 3)
#define STATUS_DCD      (1 << 2)
#define STATUS_TDRE     (1 << 1)
#define STATUS_RDRF     (1 << 0)

#define CONTROL_RXIRQ   (1 << 7)

#define CTRL_TXIRQ      0x20
#define CTRL_TXIRQMSK   0x60

struct acia_6850 {
    SysBusDevice busdev;
    CharDriverState *chr;
    qemu_irq irq;
    uint8_t status;
    uint8_t control;
    char buf;
};

static void acia_update_irq(struct acia_6850 *s)
{
    int rx_irq = s->status & STATUS_RDRF && s->control & CONTROL_RXIRQ;
    rx_irq |= ((s->control & CTRL_TXIRQMSK) == CTRL_TXIRQ) && s->status & STATUS_TDRE;

    if (rx_irq) {
        s->status |= STATUS_IRQ;
        qemu_irq_raise(s->irq);
    } else {
        s->status &= ~STATUS_IRQ;
        qemu_irq_lower(s->irq);
    }
}

static void acia_rx(void *opaque, const uint8_t *buf, int size)
{
    struct acia_6850 *s = opaque;

    if (size > 1) {
        printf("Uart dropped character\n");
    } else if (size == 1) {
        s->buf = *buf;
        s->status |= STATUS_RDRF;
    }
    acia_update_irq(s);
}

static int acia_can_rx(void *opaque)
{
    struct acia_6850 *s = opaque;

    if (s->status & STATUS_RDRF) {
        return 0;
    } else {
        return 1;
    }
}

static void acia_event(void *opaque, int event)
{
}

static uint32_t acia_6850_mem_readb(void *opaque, target_phys_addr_t addr)
{
    struct acia_6850 *s = opaque;
    uint32_t value = 0;

    switch (addr) {
        case ACIA_CONTROL:
            value = s->status;
            break;
        case ACIA_DATA:
            value = s->buf;
            s->status &= ~STATUS_RDRF;
            acia_update_irq(s);
            break;
        default:
            printf("Unknown acia mem read %"PRIx32"\n ", (uint32_t)addr);
    };
    return value;
}

static uint32_t acia_6850_mem_readf(void *opaque, target_phys_addr_t addr)
{
    printf("ACIA read more than one byte\n");
    exit(1);
    return 0;
}

static void acia_6850_mem_writeb(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    struct acia_6850 *s = opaque;

    switch (addr) {
        case ACIA_CONTROL:
            s->control = val;
            acia_update_irq(s);
            break;
        case ACIA_DATA:
            qemu_chr_write(s->chr, (uint8_t *)&val, 1);
            break;
        default:
            printf("Unknown acia mem write %"PRIx32"\n", (uint32_t)addr);
    };
}

static void acia_6850_mem_writef(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    printf("ACIA wrote more than one byte\n");
    exit(1);
}

static CPUReadMemoryFunc * const acia_6850_mem_read[] = {
    acia_6850_mem_readb,
    acia_6850_mem_readf,
    acia_6850_mem_readf,
};

static CPUWriteMemoryFunc * const acia_6850_mem_write[] = {
    acia_6850_mem_writeb,
    acia_6850_mem_writef,
    acia_6850_mem_writef,
};

static int acia_6850_init(SysBusDevice *dev)
{
    struct acia_6850 *s = (struct acia_6850 *) dev;
    int regs;

    /* Initialize */
    s->status = s->control = s->buf = 0;
    s->status |= STATUS_TDRE;
    s->irq = NULL;

    /* Set up memory-mapped io */
    regs = cpu_register_io_memory(acia_6850_mem_read,
                                  acia_6850_mem_write, s);
    sysbus_init_mmio(dev, 4, regs);

    /* Request an irq line */
    sysbus_init_irq(dev, &s->irq);

    /* Grab a qemu serial device */
    s->chr = qdev_init_chardev(&dev->qdev);
    if (s->chr)
        qemu_chr_add_handlers(s->chr, acia_can_rx, acia_rx, acia_event, s);
    else
        printf("ACIA 6850 could not allocate chardev\n");

    return 0;
}

static void acia_6850_register_devices(void)
{
    sysbus_register_dev("acia-6850", sizeof(struct acia_6850),
                        acia_6850_init);
}

device_init(acia_6850_register_devices);
