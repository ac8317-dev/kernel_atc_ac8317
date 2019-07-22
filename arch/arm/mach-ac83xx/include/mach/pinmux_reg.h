
#ifndef __ASM_ARCH_PINMUX_REG_H
#define __ASM_ARCH_PINMUX_REG_H




int __bsp_pinset(unsigned pinmux_sel, unsigned function);
int __bsp_pinget(unsigned pinmux_sel);
int GPIO_MultiFun_Set(int i4GpioNum,  int i4FuncSel);
unsigned int GPIO_MultiFun_Get(unsigned int i4GpioNum);


#endif


