/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 */

/**
 * @file
 * @brief mips specific string optimizations
 *
 * Thanks to the Linux kernel here we can add many micro optimized string
 * functions. But currently it makes no sense, to do so.
 */
#ifndef __ASM_MIPS_STRING_H
#define __ASM_MIPS_STRING_H

#ifdef CONFIG_MIPS_OPTIMIZED_STRING_FUNCTIONS

#define __HAVE_ARCH_MEMCPY
extern void *memcpy(void *, const void *, __kernel_size_t);
#define __HAVE_ARCH_MEMSET
extern void *memset(void *, int, __kernel_size_t);

#endif

#endif
