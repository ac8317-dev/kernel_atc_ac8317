#ifndef __ASM_ARCH_PINMUX_H
#define __ASM_ARCH_PINMUX_H

#if defined (__ARM2__)

/*
   arm2 build environment
*/
#include <pinmux_reg.h>
	
#define bsp_pinset __bsp_pinset
#define bsp_pinget __bsp_pinget


#elif defined (__UBOOT__)
//#error "uboot build test"


/*
   uboot build environment
*/
#include <pinmux_reg.h>

#define bsp_pinset __bsp_pinset
#define bsp_pinget __bsp_pinget

#else
//#error "kernel build test"

#include <mach/pinmux_reg.h>

int bsp_pinset(unsigned pinmux_sel, unsigned function);
int bsp_pinget(unsigned pinmux_sel);

int BSP_PinSet(int i4FuncSel, int i4Func);
int BSP_PinGet(int i4FuncSel);




#endif

#endif


