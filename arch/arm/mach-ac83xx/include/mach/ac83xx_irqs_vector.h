/*
 *  linux/arch/arm/mach-ac83xx/include/mach/ac83xx_irqs_vector.h
 *
 *  Copyright (C) 2009 MediaTek Inc.
 */

#if !defined(AC83XX_IRQS_VECTOR_H)
#define AC83XX_IRQS_VECTOR_H

#if !defined(X_BIM_H)
#include "chip_ver.h"
#endif

#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_AC83XX)
#include "83xx_irqs_vector.h"
#else
#include "error_irqs_vector.h"  // need to add irq vector
#endif // end of #if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_AC83XX)

#endif /* AC83XX_IRQS_VECTOR_H*/

