#include <linux/init.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <asm/hardware/gic.h>

#include "mach/sync_write.h"
#include <mach/hardware.h>
#include <mach/ac83xx_system.h>


#define REG_CORE1_MAGIC  (BIM_BASE_VA+0x114)
#define REG_RW_SLAVE_START  (BIM_BASE_VA+0x110)
#define CORE1_MAGIC_NUM    0x4c48462e

static cpumask_t dead_cpus;

extern void ac83xx_secondary_startup(void);
void ac83xx_smp_resume()
{

 ac83xx_reg_sync_writel(CORE1_MAGIC_NUM, REG_CORE1_MAGIC);
 ac83xx_reg_sync_writel(virt_to_phys(ac83xx_secondary_startup), REG_RW_SLAVE_START);

}


//kernel 3.12
/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen".
 */



/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */


void __cpuinit ac83xx_secondary_init(unsigned int cpu)
{

}

int __cpuinit ac83xx_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
    switch(cpu)
    {
        case 1:
        case 2:
        case 3:
            ac83xx_reg_sync_writel(CORE1_MAGIC_NUM, REG_CORE1_MAGIC);
            break;
    }
    //arch_send_wakeup_ipi_mask(cpumask_of(cpu));
    dsb_sev();

    return 0;
}

void __init ac83xx_smp_init_cpus(void)
{

}

void __init ac83xx_smp_prepare_cpus(unsigned int max_cpus)
{
    ac83xx_reg_sync_writel(virt_to_phys(ac83xx_secondary_startup), REG_RW_SLAVE_START);
}


//kernel  3.4.35


/*
 * Setup the set of possible CPUs (via set_cpu_possible)
 */
void smp_init_cpus(void)
{
    set_smp_cross_call(gic_raise_softirq);
}


/*
 * Perform platform specific initialisation of the specified CPU.
 */
void platform_secondary_init(unsigned int cpu)
{
    /*
     * If any interrupts are already enabled for the primary
     * core (e.g. timer irq), then they will not have been enabled
     * for us: do so
     */
    gic_secondary_init(0);
}

/*
 * Initialize cpu_possible map, and enable coherency
 */
void platform_smp_prepare_cpus(unsigned int max_cpus)
{
    ac83xx_reg_sync_writel(virt_to_phys(ac83xx_secondary_startup), REG_RW_SLAVE_START);
}


/*
 * Boot a secondary CPU, and assign it the specified idle task.
 * This also gives us the initial stack to use for this CPU.
 */
 int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
    
    
    
    switch(cpu)
    {
        case 1:
	   #ifdef CONFIG_HOTPLUG_CPU
	     spm_set_power(SPM_MODULE_CPU1,true);
	   #endif
	    
            ac83xx_reg_sync_writel(CORE1_MAGIC_NUM, REG_CORE1_MAGIC);
		case 2:
		case 3:
            break;
    }
	
	ac83xx_reg_sync_writel(0x0, BIM_BASE_VA + 0x118);

    //arch_send_wakeup_ipi_mask(cpumask_of(cpu));
    gic_raise_softirq(cpumask_of(cpu), 1);
    dsb_sev();

    return 0;
}

int platform_cpu_disable(unsigned int cpu)
{
    return 0;
}


void platform_cpu_enable(unsigned int cpu)
{

}
int platform_cpu_kill(unsigned int cpu)
{
          uint32_t rval = 0;
		  int k;
	/* this function is running on another CPU than the offline target,
     * here we need wait for shutdown code in platform_cpu_die() to
     * finish before asking SoC-specific code to power off the CPU core.
     */

	/*switch(cpu)
    {
        case 1:
	   #ifdef CONFIG_HOTPLUG_CPU
	     spm_set_power(SPM_MODULE_CPU1,false);
	   #endif
	    
		case 2:
		case 3:
            break;
    }
	*/

	
	for (k = 0; k < 1000; k++) 
	{
		if (cpumask_test_cpu(cpu, &dead_cpus))
		{
			 cpumask_clear_cpu(cpu, &dead_cpus);	
			 spm_set_power(SPM_MODULE_CPU1,false);
			 rval = __raw_readl(__io(BIM_BASE_VA + 0x118));
			 dsb();
			 rval |= 0x01;
			 ac83xx_reg_sync_writel(rval, BIM_BASE_VA + 0x118);

			 return 1;
		}

		mdelay(1);
	}

    return 0;
 

}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void platform_cpu_die(unsigned int cpu)
{
    uint32_t rval = 0;
    dsb();
    flush_cache_all();

	rval = __raw_readl(__io(BIM_BASE_VA + 0x118));
	dsb();
	rval |= 0x02;
	ac83xx_reg_sync_writel(rval, BIM_BASE_VA + 0x118);

	dsb();
	cpumask_set_cpu(cpu, &dead_cpus);

    /* wait for SoC code in platform_cpu_kill() to shut off CPU core
     * power. CPU bring up starts from the reset vector.
     */
    while (1) {
        asm("wfi"
                :
                :
                : "memory", "cc");
    }
}



