/* Header file for libgcc2.c.  */
/* Copyright (C) 2000, 2001, 2004, 2005
   Free Software Foundation, Inc.

This file is part of GCC.

GCC is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free
Software Foundation; either version 2, or (at your option) any later
version.

GCC is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
for more details.

You should have received a copy of the GNU General Public License
along with GCC; see the file COPYING.  If not, write to the Free
Software Foundation, 51 Franklin Street, Fifth Floor, Boston, MA
02110-1301, USA.  */

/* As a special exception, if you link this library with other files,
   some of which are compiled with GCC, to produce an executable,
   this library does not by itself cause the resulting executable
   to be covered by the GNU General Public License.
   This exception does not however invalidate any other reasons why
   the executable file might be covered by the GNU General Public License.  */


#ifndef GCC_LIBGCC2_H
#define GCC_LIBGCC2_H

#ifndef HIDE_EXPORTS
#pragma GCC visibility push(default)
#endif

/* In the first part of this file, we are interfacing to calls generated
   by the compiler itself.  These calls pass values into these routines
   which have very specific modes (rather than very specific types), and
   these compiler-generated calls also expect any return values to have
   very specific modes (rather than very specific types).  Thus, we need
   to avoid using regular C language type names in this part of the file
   because the sizes for those types can be configured to be anything.
   Instead we use the following special type names.  */

typedef		 int QItype	__attribute__ ((mode (QI)));
typedef unsigned int UQItype	__attribute__ ((mode (QI)));
typedef		 int HItype	__attribute__ ((mode (HI)));
typedef unsigned int UHItype	__attribute__ ((mode (HI)));
/* These typedefs are usually forbidden on dsp's with UNITS_PER_WORD 1.  */
typedef 	 int SItype	__attribute__ ((mode (SI)));
typedef unsigned int USItype	__attribute__ ((mode (SI)));
/* These typedefs are usually forbidden on archs with UNITS_PER_WORD 2.  */
typedef		 int DItype	__attribute__ ((mode (DI)));
typedef unsigned int UDItype	__attribute__ ((mode (DI)));

/* Versions prior to 3.4.4 were not taking into account the word size for
   the 5 trapping arithmetic functions absv, addv, subv, mulv and negv.  As
   a consequence, the si and di variants were always and the only ones emitted.
   To maintain backward compatibility, COMPAT_SIMODE_TRAPPING_ARITHMETIC is
   defined on platforms where it makes sense to still have the si variants
   emitted.  As a bonus, their implementation is now correct.  Note that the
   same mechanism should have been implemented for the di variants, but it
   turns out that no platform would define COMPAT_DIMODE_TRAPPING_ARITHMETIC
   if it existed.  */

#if LIBGCC2_UNITS_PER_WORD == 8
#define W_TYPE_SIZE (8 * BITS_PER_UNIT)
#define Wtype	DItype
#define UWtype	UDItype
#define HWtype	DItype
#define UHWtype	UDItype
#define DWtype	TItype
#define UDWtype	UTItype
#define __NW(a,b)	__ ## a ## di ## b
#define __NDW(a,b)	__ ## a ## ti ## b
#define COMPAT_SIMODE_TRAPPING_ARITHMETIC
#elif LIBGCC2_UNITS_PER_WORD == 4
#define W_TYPE_SIZE (4 * BITS_PER_UNIT)
#define Wtype	SItype
#define UWtype	USItype
#define HWtype	SItype
#define UHWtype	USItype
#define DWtype	DItype
#define UDWtype	UDItype
#define __NW(a,b)	__ ## a ## si ## b
#define __NDW(a,b)	__ ## a ## di ## b
#elif LIBGCC2_UNITS_PER_WORD == 2
#define W_TYPE_SIZE (2 * BITS_PER_UNIT)
#define Wtype	HItype
#define UWtype	UHItype
#define HWtype	HItype
#define UHWtype	UHItype
#define DWtype	SItype
#define UDWtype	USItype
#define __NW(a,b)	__ ## a ## hi ## b
#define __NDW(a,b)	__ ## a ## si ## b
#else
#define W_TYPE_SIZE BITS_PER_UNIT
#define Wtype	QItype
#define UWtype  UQItype
#define HWtype	QItype
#define UHWtype	UQItype
#define DWtype	HItype
#define UDWtype	UHItype
#define __NW(a,b)	__ ## a ## qi ## b
#define __NDW(a,b)	__ ## a ## hi ## b
#endif

/* DWstructs are pairs of Wtype values in the order determined by
   LIBGCC2_WORDS_BIG_ENDIAN.  */

#if LIBGCC2_WORDS_BIG_ENDIAN
  struct DWstruct {Wtype high, low;};
#else
  struct DWstruct {Wtype low, high;};
#endif

/* We need this union to unpack/pack DImode values, since we don't have
   any arithmetic yet.  Incoming DImode parameters are stored into the
   `ll' field, and the unpacked result is read from the struct `s'.  */

typedef union
{
  struct DWstruct s;
  DWtype ll;
} DWunion;

/* Defined for L_popcount_tab.  Exported here because some targets may
   want to use it for their own versions of the __popcount builtins.  */
extern const UQItype __popcount_tab[256];

/* Defined for L_clz.  Exported here because some targets may want to use
   it for their own versions of the __clz builtins.  It contains the bit
   position of the first set bit for the numbers 0 - 255.  This avoids the
   need for a separate table for the __ctz builtins.  */
extern const UQItype __clz_tab[256];

#include "longlong.h"

#ifndef HIDE_EXPORTS
#pragma GCC visibility pop
#endif

#endif /* ! GCC_LIBGCC2_H */
