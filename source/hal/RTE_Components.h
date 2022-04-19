
#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H

/*
 * Define the Device Header File: 
 */
#if EI_CONFIG_ALIF_HP
#define CMSIS_device_header "M55_HP.h"
#endif

#if EI_CONFIG_ALIF_HE
#define CMSIS_device_header "M55_HE.h"
#endif

/* AlifSemiconductor::CMSIS Driver:SOC Peripherals:GPIO:1.0.0 */
#define RTE_Drivers_GPIO   1           /* Driver GPIO */
/* AlifSemiconductor::CMSIS Driver:SOC Peripherals:PINCONF:1.0.0 */
#define RTE_Drivers_PINCONF   1           /* Driver PinPAD and PinMux */
#define RTE_UART2 1
#define RTE_Drivers_SAI 1


#endif /* RTE_COMPONENTS_H */
