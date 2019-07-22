#ifndef LINUX_MMC_TRACELOG_H
#define LINUX_MMC_TRACELOG_H
#include <linux/types.h>
#include <linux/mmc/core.h>

#ifdef MMC_TRACE_ENABLE
void trace_mmc_blk_erase_start(unsigned int index, unsigned int fr, unsigned int nr);
void trace_mmc_blk_erase_end(unsigned int index, unsigned int fr, unsigned int nr);
void trace_mmc_blk_rw_end(unsigned int opcode, unsigned int arg, struct mmc_data *data);
#else
static inline void trace_mmc_blk_erase_start(unsigned int index, unsigned int fr, unsigned int nr)
{
}

static inline void trace_mmc_blk_erase_end(unsigned int index, unsigned int fr, unsigned int nr)
{
}
static inline void trace_mmc_blk_rw_end(unsigned int opcode, unsigned int arg, struct mmc_data *data)
{
}

static inline void trace_mmc_blk_rw_start(unsigned int opcode, unsigned int arg, struct mmc_data *data)
{
}

#endif


#ifdef MMC_LOG_ENABLE
#define xlog_printk(level, tag, args...) printk(args)
#else
#define xlog_printk(level, tag, args...)  do {} while(0)
#endif


#endif
