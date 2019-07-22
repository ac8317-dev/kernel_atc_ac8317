
#ifndef __ASM_ARCH_GPIO_REG_H
#define __ASM_ARCH_GPIO_REG_H



int ac83xx_gpio_inout_sel_reg(unsigned gpio, int dir);
int ac83xx_gpio_get_value_reg(unsigned gpio);
int ac83xx_gpio_set_value_reg(unsigned gpio, int value);
int ac83xx_gpio_direction_input_reg(unsigned gpio);
int ac83xx_gpio_direction_output_reg(unsigned gpio, int value);
void GPIO_Pull_UpDown(int i4GpioNum, unsigned PullUpOrDown);


#endif
