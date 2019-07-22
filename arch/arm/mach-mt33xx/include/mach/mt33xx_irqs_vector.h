/*
 *  linux/include/asm-arm/arch-mt85xx/irqs.h
 *
 *  Copyright (C) 2009 MediaTek Inc.
 */

#if !defined(IRQS_VECTOR_H)
#define IRQS_VECTOR_H

#if !defined(X_BIM_H)
#include "chip_ver.h"
#endif

#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
#include "mt3360_irqs_vector.h"
#else
#include "error_irqs_vector.h"  // need to add irq vector
#endif // end of #if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)

#endif /*IRQS_VECTOR_H*/

