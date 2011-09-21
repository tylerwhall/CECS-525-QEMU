/*
 * Dumb timer
 * Creates an interrupt at a fixed interval and resets when read or written
 *
 * Copyright (c) 2010 Tyler Hall.
 *
 * This code is licenced under the GPL
 */

#include <inttypes.h>

#include "hw.h"
#include "qemu-timer.h"
#include "sysbus.h"

#define TICK_NS (1000*1000*1000 / 1200)

struct dumb_timer {
    SysBusDevice busdev;
    qemu_irq irq;
    QEMUTimer *timer;
};

static void dumb_timer_tick(void *opaque)
{
    struct dumb_timer *s = opaque;

    qemu_irq_raise(s->irq);
    qemu_mod_timer(s->timer, qemu_get_clock_ns(vm_clock) + TICK_NS);
}

static uint32_t dumb_timer_mem_readf(void *opaque, target_phys_addr_t addr)
{
    struct dumb_timer *s = opaque;

    qemu_irq_lower(s->irq);
    return 0;
}

static void dumb_timer_mem_writef(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    struct dumb_timer *s = opaque;

    qemu_irq_lower(s->irq);
}

static CPUReadMemoryFunc * const dumb_timer_mem_read[] = {
    dumb_timer_mem_readf,
    dumb_timer_mem_readf,
    dumb_timer_mem_readf,
};

static CPUWriteMemoryFunc * const dumb_timer_mem_write[] = {
    dumb_timer_mem_writef,
    dumb_timer_mem_writef,
    dumb_timer_mem_writef,
};

static int dumb_timer_init(SysBusDevice *dev)
{
    struct dumb_timer *s = (struct dumb_timer *) dev;
    int regs;

    /* Initialize */
    s->irq = NULL;
    s->timer = NULL;

    /* Set up memory-mapped io */
    regs = cpu_register_io_memory(dumb_timer_mem_read,
                                  dumb_timer_mem_write, s, DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 4, regs);

    /* Request an irq line */
    sysbus_init_irq(dev, &s->irq);

    /* Setup timer */
    s->timer = qemu_new_timer_ns(vm_clock, dumb_timer_tick, s);
    dumb_timer_tick(s);

    return 0;
}

static void dumb_timer_register_devices(void)
{
    sysbus_register_dev("dumb-timer", sizeof(struct dumb_timer),
                        dumb_timer_init);
}

device_init(dumb_timer_register_devices);
