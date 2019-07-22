#include <linux/module.h>


void __aeabi_ui2d(void);
void __aeabi_ddiv(void);
void __aeabi_dmul(void);
void __aeabi_dadd(void);
void __aeabi_dsub(void);
void __aeabi_i2d(void);
void __aeabi_d2uiz(void);
void __aeabi_uldivmod(void);
void __aeabi_ldivmod(void);
void __aeabi_dcmpgt(void);
void __aeabi_dcmplt(void);
void sched_setscheduler_nocheck(void);


EXPORT_SYMBOL(__aeabi_ui2d);
EXPORT_SYMBOL(__aeabi_ddiv);
EXPORT_SYMBOL(__aeabi_dmul);
EXPORT_SYMBOL(__aeabi_dadd);
EXPORT_SYMBOL(__aeabi_dsub);
EXPORT_SYMBOL(__aeabi_i2d);
EXPORT_SYMBOL(__aeabi_d2uiz);
EXPORT_SYMBOL(__aeabi_uldivmod);
EXPORT_SYMBOL(__aeabi_ldivmod);
EXPORT_SYMBOL(__aeabi_dcmpgt);
EXPORT_SYMBOL(__aeabi_dcmplt);
EXPORT_SYMBOL(sched_setscheduler_nocheck);
