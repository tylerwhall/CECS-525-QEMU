/*
 *  m68k execution defines
 *
 *  Copyright (c) 2005-2006 CodeSourcery
 *  Written by Paul Brook
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#include "dyngen-exec.h"

register struct CPUM68KState *env asm(AREG0);

#include "cpu.h"

extern const uint8_t rox8_table[64];
extern const uint8_t rox16_table[64];
extern const uint8_t rox32_table[64];

#if !defined(CONFIG_USER_ONLY)
#include "softmmu_exec.h"
#endif
