#include <linux/init.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>

#include <asm/mach/time.h>
#include <asm/smp_twd.h>
#include <mach/hardware.h>


#define TIMER1_BASE 0x0
#define TIMER2_BASE 0x8
#define TIMER3_BASE 0x50

#define REG_RW_TIMER0_LIM_OFFSET    0x00
#define REG_RW_TIMER0_VALUE         0x04
#define REG_RW_TIMER_CTRL_OFFSET    0x1C

#define VAL_T0_AUTOLOAD     0x2
#define VAL_T0_ENABLE       0x1



#define AC83XX_DEFAULT_TIMER_RATE  27000000

static void __iomem *timer_reg_base;



static void ac83xx_timer_set_mode(enum clock_event_mode mode, struct clock_event_device *evt)
{
	//u32 reg;
	



	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:

		
		/* Start timer irq */
    __raw_writel(VAL_T0_AUTOLOAD, IOMEM(timer_reg_base+REG_RW_TIMER_CTRL_OFFSET));
		__raw_writel(VAL_T0_AUTOLOAD | VAL_T0_ENABLE, IOMEM(timer_reg_base+REG_RW_TIMER_CTRL_OFFSET));

		break;
	case CLOCK_EVT_MODE_ONESHOT:
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}


static struct clock_event_device ac83xx_clockevent = {
	.name		= "timer0",
	.rating		= 300,
	.features	=  CLOCK_EVT_FEAT_PERIODIC,
	.set_mode	= ac83xx_timer_set_mode,
};


static irqreturn_t ac83xx_timer_interrupt(int irq, void *dev_id)
{
	u32 regval32;

	struct clock_event_device *evt = (struct clock_event_device *)dev_id;


	regval32 = (1 << (VECTOR_T0-IRQ_BIM_GIC_OFFSET));
    __bim_writel(regval32, REG_IRQCL);

    regval32 = (1 << (VECTOR_T0-IRQ_BIM_GIC_OFFSET));
    __bim_writel(regval32, REG_IRQST);
	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static struct irqaction ac83xx_timer_irq = {
	.name		= "timer0",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_TRIGGER_HIGH,
	.handler	= ac83xx_timer_interrupt,
	.dev_id		= &ac83xx_clockevent,
};

static void __init ac83xx_init_timer(struct device_node *np)
{
	//struct clk *clk;
	unsigned long rate;
	int ret;
	u32 u4_cfg_clock_per_ticks;

	timer_reg_base = of_iomap(np, 0);
	if (!timer_reg_base) {
		pr_err("Can't map timer registers\n");
		BUG();
	}

	ac83xx_timer_irq.irq = irq_of_parse_and_map(np, 2);
	if (ac83xx_timer_irq.irq <= 0) {
		pr_err("Failed to map timer IRQ\n");
		BUG();
	}
#if 0
	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		pr_warn("Unable to get timer clock. Assuming 27Mhz input clock.\n");
		rate = AC83XX_DEFAULT_TIMER_RATE;
	} else {
	//	clk_prepare_enable(clk);
	//	rate = clk_get_rate(clk);
	}
#endif
	of_node_put(np);

	rate = AC83XX_DEFAULT_TIMER_RATE;

	switch (rate) {
	case AC83XX_DEFAULT_TIMER_RATE:
		u4_cfg_clock_per_ticks = rate / CONFIG_HZ;
		__raw_writel(u4_cfg_clock_per_ticks, IOMEM(timer_reg_base+REG_RW_TIMER0_LIM_OFFSET));

		break;
	default:
		WARN(1, "Unknown clock rate");
	}

//	setup_sched_clock(tegra_read_sched_clock, 32, 1000000);

	//if (clocksource_mmio_init(timer_reg_base + TIMERUS_CNTR_1US,
	//	"timer_us", 1000000, 300, 32, clocksource_mmio_readl_up)) {
	//	pr_err("Failed to register clocksource\n");
	//	BUG();
	//}

	ret = setup_irq(ac83xx_timer_irq.irq, &ac83xx_timer_irq);
	if (ret) {
		pr_err("Failed to register timer IRQ: %d\n", ret);
		BUG();
	}

	ac83xx_clockevent.cpumask = cpu_all_mask;
	ac83xx_clockevent.irq = ac83xx_timer_irq.irq;
	clockevents_config_and_register(&ac83xx_clockevent, CONFIG_HZ,
					0, 0);
}
//CLOCKSOURCE_OF_DECLARE(ac83xx_timer, "Autochips,ac83xx-timer", ac83xx_init_timer);

struct sys_timer ac83xx_timer = {
	.init	= clocksource_of_init,
};
