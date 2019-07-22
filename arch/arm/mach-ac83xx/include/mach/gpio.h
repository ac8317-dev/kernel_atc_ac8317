/*
 * arch/arm/mach-dove/include/mach/gpio.h
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H

/*

bool gpio_is_valid(int number);

int gpio_request(unsigned gpio, const char *label);


gpio_request_one(unsigned gpio,
					unsigned long flags, const char *label);

int gpio_request_array(const struct gpio *array, size_t num);


void gpio_free(unsigned gpio);


void gpio_free_array(const struct gpio *array, size_t num);

int gpio_direction_input(unsigned gpio);


int gpio_direction_output(unsigned gpio, int value);


int gpio_set_debounce(unsigned gpio, unsigned debounce);


int gpio_get_value(unsigned gpio);


void gpio_set_value(unsigned gpio, int value);


int gpio_cansleep(unsigned gpio);

int gpio_get_value_cansleep(unsigned gpio);


void gpio_set_value_cansleep(unsigned gpio, int value);

int gpio_export(unsigned gpio, bool direction_may_change);

int gpio_export_link(struct device *dev, const char *name,
				unsigned gpio);


int gpio_sysfs_set_active_low(unsigned gpio, int value);


void gpio_unexport(unsigned gpio);


int gpio_to_irq(unsigned gpio);


int irq_to_gpio(unsigned irq);
*/
//#define __UBOOT__

#if defined (__ARM2__)
	
	/*
	   arm2 build environment
	*/
#include <gpio_reg.h>
	
#define gpio_get_value ac83xx_gpio_get_value_reg
#define gpio_set_value ac83xx_gpio_set_value_reg
#define gpio_direction_input ac83xx_gpio_direction_input_reg
#define gpio_direction_output ac83xx_gpio_direction_output_reg



#elif defined (__UBOOT__)
	
	//#error "uboot build test"
	/*
	   uboot build environment
	*/

#include <gpio_reg.h>

#define gpio_get_value ac83xx_gpio_get_value_reg
#define gpio_set_value ac83xx_gpio_set_value_reg
#define gpio_direction_input ac83xx_gpio_direction_input_reg
#define gpio_direction_output ac83xx_gpio_direction_output_reg


#else
/*
   kernel build environment
*/
//#error "kernel build test"

#include <linux/interrupt.h>
#include <mach/gpio_reg.h>
int gpio_to_irq(unsigned gpio);
void gpio_set_value(unsigned gpio, int value);
int gpio_get_value(unsigned gpio);
int  request_gpio_irq(unsigned int gpio, irq_handler_t handler, unsigned long flags,
	    const char *name, void *dev);



#endif

#endif

