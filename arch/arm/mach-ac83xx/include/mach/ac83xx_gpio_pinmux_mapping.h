#ifndef __AC83XX_GPIO_MAPPING_H
#define __AC83XX_GPIO_MAPPING_H

#include "ac83xx_gpio_pinmux.h"

#define XXX_PIN PIN_5_GPIO5

#define BT_PIN_PWN PIN_43_GPIO43
#define BT_PIN_RST PIN_44_GPIO44


//#if defined(FIXED_TYPE_BOARD)
#define WIFI_POWER_PIN PIN_56_GPIO56
#define WIFI_RESET_PIN PIN_57_GPIO57
//#endif


#define GPS_PIN_RST  PIN_31_GPIO31

#define TP_EINT_PIN              PIN_37_EINT2
#define TP_INT_PORT             VECTOR_EXT3
#define TP_INT_FUNCTION    EINT2_SEL 
#define TP_EINT_NUM             2
#define TP_RESET_PORT        PIN_55_GPIO55  


#endif
