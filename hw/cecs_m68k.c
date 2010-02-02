/*
 * 68K minimal computer.
 * Used for CECS 525 at the University of Louisville
 *
 * Copyright (c) 2010 Tyler Hall.
 *
 * This code is licenced under the GPL
 */

#include <stdio.h>
#include <inttypes.h>

#include "hw.h"
#include "sysemu.h"
#include "boards.h"
#include "loader.h"
#include "block.h"
#include "block_int.h"
#include "sysbus.h"

#define ROM_ADDR 0x0
#define ROM_SIZE (0x4000) //16k

#define RAM_ADDR 0x4000
#define RAM_SIZE (0x1000) //4k - 1k

typedef struct {
    target_phys_addr_t vectors[256];
} vector_state;

typedef enum {
    SREC_IDLE,
    SREC_TYPE,
    SREC_COUNT,
    SREC_ADDR,
    SREC_DATA,
} srec_state;

static inline char xatoi(char val)
{
    switch (val) {
        case '0' ... '9':
            val -= '0';
            break;
        case 'a' ... 'f':
            val = 10 + val - 'a';
            break;
        case 'A' ... 'F':
            val = 10 + val - 'A';
            break;
    }
    return val;
}

static void srec_byte(uint8_t input)
{
    static srec_state state;
    static int first_nibble;
    static uint8_t byte;
    static int addr_len;
    static uint32_t addr;
    static int len;
    static int buf_count;
    static uint8_t buf[256];

    if (input == 'S') {
        state = SREC_TYPE;
        return;
    }

    if (state >= SREC_COUNT) {
        if (first_nibble) {
            byte = xatoi(input) << 4;
            first_nibble = 0;
            return;
        } else {
            byte |= xatoi(input);
            first_nibble = 1;
        }
    }

    switch (state) {
        case SREC_IDLE:
            break;
        case SREC_TYPE: {
            int type;

            type = input - '0';
            if (type >= 1 && type <= 3) {
                addr_len = type + 1;
                state = SREC_COUNT;
                first_nibble = 1;
                addr = 0;
            } else {
                state = SREC_IDLE;
            }
            break;
        }
        case SREC_COUNT:
            len = byte - 1; //Ignore the checksum at the end
            len -= addr_len; //len is only the length of the actual data
            state = SREC_ADDR;
            buf_count = 0;
            break;
        case SREC_ADDR:
            addr <<= 8;
            addr |= byte;
            if (--addr_len == 0)
                state = SREC_DATA;
            break;
        case SREC_DATA:
            buf[buf_count++] = byte;
            if (buf_count >= sizeof(buf)) {
                printf("SREC Line buffer exceeded\n");
                state = SREC_IDLE;
            } else if (buf_count >= len) {
                cpu_physical_memory_write_rom(addr, buf, len);
                printf("Line read. Addr = 0x%x,Len = %d\n", addr, len);
                state = SREC_IDLE;
            }
            break;
    }
}

static void srec_input(uint8_t *buf, int size)
{
    int i;

    for (i = 0; i < size; i++)
        srec_byte(buf[i]);
}

static int copy_rom(const char *filename)
{
    FILE *fd;
    uint8_t buf[256];
    int count;

    fd = fopen(filename, "rb");
    if (!fd)
        return -1;
    do {
        count = fread(buf, 1, sizeof(buf), fd);
        srec_input(buf, count);
    } while (count);

    return 0;
}

static void cecs_m68k_reset(void *opaque)
{
    CPUState *env = (CPUState *)opaque;

    cpu_reset(env);
}

static void cecs_m68k_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
    CPUState *env;

    if (!cpu_model)
        cpu_model = "m68000";
    env = cpu_init(cpu_model);
    if (!env) {
        fprintf(stderr, "Unable to find m68k CPU definition\n");
        exit(1);
    }

    env->vbr = 0;

    /* Use ROMD instead of ROM so it can still be executable, but
     * partially writable. */
    cpu_register_physical_memory(ROM_ADDR, ROM_SIZE,
        qemu_ram_alloc(ROM_SIZE) | IO_MEM_ROM);

    cpu_register_physical_memory(RAM_ADDR, RAM_SIZE,
        qemu_ram_alloc(RAM_SIZE) | IO_MEM_RAM);

    if (!kernel_filename) {
        printf("Must specify rom as kernel\n");
        exit(1);
    }

    if(copy_rom(kernel_filename)) {
        printf("Failed to load rom\n");
        exit(1);
    }

    if (initrd_filename) {
        if(copy_rom(initrd_filename)) {
            printf("Failed to load initrd\n");
        } else {
            printf("Loaded initrd\n");
        }
    }

    sysbus_create_simple("acia-6850", 0x8000, NULL);

    qemu_register_reset(cecs_m68k_reset, env);
    cecs_m68k_reset(env); //Sets SSP and PC
}

static QEMUMachine cecs_m68k_machine = {
    .name = "cecs",
    .desc = "CECS 525 minimal computer",
    .init = cecs_m68k_init,
};

static void cecs_m68k_machine_init(void)
{
    qemu_register_machine(&cecs_m68k_machine);
}

machine_init(cecs_m68k_machine_init);
