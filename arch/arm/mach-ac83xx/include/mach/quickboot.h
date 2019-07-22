#ifndef __QUICKBOOT_PARAM_
#define __QUICKBOOT_PARAM_

#define GPIO_POLARITY_LOW  0
#define GPIO_POLARITY_HIGH 1



#define GPIO_WAKEUP_STS    0
#define GPIO_WAKEUP_SRC    1
#define GPIO_PAD_IR        2


struct quickboot_param {
 
       unsigned int version;

       /*ddr calibration address */
       unsigned int ddr_cal_addr;
       /*os resume entry(physical address) */
       unsigned int resume_entry;
       /* wakeup source gpio & polarity config*/ 
       unsigned int wakeup_src_gpio;
       unsigned int wakeup_src_polarity;
       /*wakeup state gpio & polarity config*/
       unsigned int wakeup_sts_gpio;
       unsigned int wakeup_sts_polarity;
       
       /* delay time for system broad power off*/ 
       unsigned int power_off_delay;
       
       /* delay time for cpu reset*/
       unsigned int power_on_delay;

};


#endif



