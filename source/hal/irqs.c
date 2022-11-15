/*
 * Copyright (c) 2021 Arm Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifdef __cplusplus
extern "C"
{
#endif

#include "irqs.h"
#include "cmsis.h"

#include <stdio.h>
#include <inttypes.h>

static uint64_t cpu_cycle_count = 0;

/**
 * External references
 */
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    extern uint32_t __STACK_SEAL;
#endif

extern __NO_RETURN void __PROGRAM_START(void);

/**
 * @brief   Dump core registers on stdout
 */
static void LogCoreCPURegisters(void)
{
    printf("CTRL    : 0x%08" PRIx32 "\n", __get_CONTROL());
    printf("IPSR    : 0x%08" PRIx32 "\n", __get_IPSR());
    printf("APSR    : 0x%08" PRIx32 "\n", __get_APSR());
    printf("xPSR    : 0x%08" PRIx32 "\n", __get_xPSR());
    printf("PSP     : 0x%08" PRIx32 "\n", __get_PSP());
    printf("MSP     : 0x%08" PRIx32 "\n", __get_MSP());
    printf("PRIMASK : 0x%08" PRIx32 "\n", __get_PRIMASK());
    printf("BASEPRI : 0x%08" PRIx32 "\n", __get_BASEPRI());
    printf("FAULTMSK: 0x%08" PRIx32 "\n", __get_FAULTMASK());
}

/**
 * @brief   Default interrupt handler - an infinite loop.
 **/
__attribute__((noreturn)) static void DefaultHandler(void)
{
    LogCoreCPURegisters();
    while (1) {
        /* Without the following line, armclang may optimize away the
         * infinite loop because it'd be without side effects and thus
         * undefined behaviour. */
        __ASM volatile("");
    }
}

#define DEFAULT_HANDLER_CALL(type)              \
    do {                                        \
        printf("\n");                           \
        printf("%s caught by function %s\n",    \
             type, __FUNCTION__);               \
        DefaultHandler();                       \
    } while (0)

#define DEFAULT_ERROR_HANDLER_CALL()            \
            DEFAULT_HANDLER_CALL("Exception")

#define DEFAULT_IRQ_HANDLER_CALL()              \
            DEFAULT_HANDLER_CALL("Interrupt")

/**
 * Dummy Exception Handlers for core interrupts.
 *
 * Weak definitions provided to be used if the user chooses not
 * to override them.
 **/

/**
 * @brief  Non maskable interrupt handler.
 **/
 __attribute__((weak)) void NMI_Handler(void)
{
    DEFAULT_ERROR_HANDLER_CALL();
}

/**
 * @brief  Hardfault interrupt handler.
 **/
 __attribute__((weak)) void HardFault_Handler(void)
{
    DEFAULT_ERROR_HANDLER_CALL();
}

/**
 * @brief  Memory management interrupt handler.
 **/
__attribute__((weak)) void MemManage_Handler(void)
{
    DEFAULT_IRQ_HANDLER_CALL();
}

/**
 * @brief  Bus fault interrupt handler.
 **/
__attribute__((weak)) void BusFault_Handler(void)
{
    DEFAULT_ERROR_HANDLER_CALL();
}

/**
 * @brief  Usage fault interrupt handler.
 **/
__attribute__((weak)) void UsageFault_Handler(void)
{
    DEFAULT_ERROR_HANDLER_CALL();
}

/**
 * @brief  Secure access fault interrupt handler.
 **/
__attribute__((weak)) void SecureFault_Handler(void)
{
    DEFAULT_ERROR_HANDLER_CALL();
}

/**
 * @brief  Supervisor call interrupt handler.
 **/
__attribute__((weak)) void SVC_Handler(void)
{
    DEFAULT_IRQ_HANDLER_CALL();
}

/**
 * @brief  Debug monitor interrupt handler.
 **/
__attribute__((weak)) void DebugMon_Handler(void)
{
    DEFAULT_IRQ_HANDLER_CALL();
}

/**
 * @brief  Pending SV call interrupt handler.
 */
__attribute__((weak)) void PendSV_Handler(void)
{
    DEFAULT_IRQ_HANDLER_CALL();
}

/**
 * @brief   System tick interrupt handler.
 **/
void SysTick_Handler(void)
{
    /* Increment the cycle counter based on load value. */
    cpu_cycle_count += SysTick->LOAD + 1;

    extern void lv_port_tick_inc();
    lv_port_tick_inc();
}

/**
 * Gets the current SysTick derived counter value
 */
uint64_t Get_SysTick_Cycle_Count(void)
{
    uint32_t systick_val;

    NVIC_DisableIRQ(SysTick_IRQn);
    systick_val = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
    NVIC_EnableIRQ(SysTick_IRQn);

    return cpu_cycle_count + (SysTick->LOAD - systick_val);
}

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void)
{
  while(1);
}

void arm_npu_irq_handler    (void) __attribute__ ((weak, alias("Default_Handler")));

void UART0_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UART1_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UART2_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UART3_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UART6_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UART7_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));

void I2C0_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));

void I3C0_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));

void GPIO4_PIN0_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO4_PIN1_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO4_PIN2_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO4_PIN3_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO4_PIN4_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO4_PIN5_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO4_PIN6_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO4_PIN7_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN0_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN1_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN2_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN3_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN4_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN5_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN6_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN7_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN8_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN9_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN10_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN11_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN12_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN13_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN14_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN15_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN16_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN17_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN18_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN19_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN20_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN21_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN22_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN23_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN24_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN25_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN26_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN27_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN28_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN29_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN30_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO1_PIN31_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN0_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN1_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN2_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN3_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN4_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN5_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN6_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN7_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN8_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN9_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN10_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN11_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN12_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN13_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN14_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN15_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN16_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN17_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN18_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN19_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN20_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN21_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN22_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN23_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN24_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN25_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN26_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN27_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN28_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN29_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN30_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO2_PIN31_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN0_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN1_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN2_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN3_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN4_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN5_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN6_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN7_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN8_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN9_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN10_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN11_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN12_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN13_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN14_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN15_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN16_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN17_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN18_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN19_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN20_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN21_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN22_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN23_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN24_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN25_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN26_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN27_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN28_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN29_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN30_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO3_PIN31_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));

void LPTIMER_CHANNEL0_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void LPTIMER_CHANNEL1_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void LPTIMER_CHANNEL2_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void LPTIMER_CHANNEL3_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));

void RTC0_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));

/**
 * Interrupt vector table.
 */
irq_vec_type __VECTOR_TABLE[] __VECTOR_TABLE_ATTRIBUTE = {
    (irq_vec_type)(&__INITIAL_SP),  /*     Initial Stack Pointer */
    Reset_Handler      , /* 1 Initial PC, set to entry point */

    NMI_Handler        , /* 2 (-14) NMI Handler            */
    HardFault_Handler  , /* 3 (-13) Hard Fault Handler     */
    MemManage_Handler  , /* 4 (-12) MPU Fault Handler      */
    BusFault_Handler   , /* 5 (-11) Bus Fault Handler      */
    UsageFault_Handler , /* 6 (-10) Usage Fault Handler    */
    SecureFault_Handler, /* 7 ( -9) Secure Fault Handler   */
    0                  , /* 8 ( -8) Reserved               */
    0                  , /* 9 ( -7) Reserved               */
    0                  , /* 10 (-6) Reserved              */
    SVC_Handler        , /* 11 (-5) SVCall Handler        */
    DebugMon_Handler   , /* 12 (-4) Debug Monitor Handler */
    0                  , /* 13 (-3) Reserved              */
    PendSV_Handler     , /* 14 (-2) PendSV Handler        */
    SysTick_Handler    , /* 15 (-1) SysTick Handler       */

    /* External sources to be populated by user. */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*   0 -  15 */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*  16 -  31 */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*  32 -  47 */
    0, 0, 0, 0, 0, 0, 0,                            /*  48 -  54 */
    arm_npu_irq_handler,                      /*    55 Interrupt  55 */
    0, 0, 0, 0, 0, 0, 0, 0,                         /*  56 -  63 */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*  64 -  79 */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*  80 -  95 */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*  96 -  111 */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,       /* 112 -  125 */
    UART0_IRQHandler,                         /*   126 Interrupt 126 */
    UART1_IRQHandler,                         /*   127 Interrupt 127 */
    UART2_IRQHandler,                         /*   128 Interrupt 128 */
    UART3_IRQHandler,                         /*   129 Interrupt 129 */
    UART4_IRQHandler,                         /*   130 Interrupt 130 */
    UART5_IRQHandler,                         /*   131 Interrupt 131 */
    UART6_IRQHandler,                         /*   132 Interrupt 132 */
    UART7_IRQHandler,                         /*   133 Interrupt 133 */
    I2C0_IRQHandler,                          /*   134 Interrupt 134 */
    I2C1_IRQHandler,                          /*   135 Interrupt 135 */
    I2C2_IRQHandler,                          /*   136 Interrupt 136 */
    I2C3_IRQHandler,                          /*   137 Interrupt 137 */
    I3C0_IRQHandler,                          /*   138 Interrupt 138 */
    0, 0, 0, 0, 0,                                  /*  139 -  143 */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*  144 -  159 */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*  160 -  175 */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                   /*  176 -  185 */
    GPIO4_PIN0_IRQHandler,                    /*   186  Interrupt 186 */
    GPIO4_PIN1_IRQHandler,                    /*   187  Interrupt 187 */
    GPIO4_PIN2_IRQHandler,                    /*   188  Interrupt 188 */
    GPIO4_PIN3_IRQHandler,                    /*   189  Interrupt 189 */
    GPIO4_PIN4_IRQHandler,                    /*   190  Interrupt 190 */
    GPIO4_PIN5_IRQHandler,                    /*   191  Interrupt 191 */
    GPIO4_PIN6_IRQHandler,                    /*   192  Interrupt 192 */
    GPIO4_PIN7_IRQHandler,                    /*   193  Interrupt 193 */
    GPIO1_PIN0_IRQHandler,                    /*   194  Interrupt 194 */
    GPIO1_PIN1_IRQHandler,                    /*   195  Interrupt 195 */
    GPIO1_PIN2_IRQHandler,                    /*   196  Interrupt 196 */
    GPIO1_PIN3_IRQHandler,                    /*   197  Interrupt 197 */
    GPIO1_PIN4_IRQHandler,                    /*   198  Interrupt 198 */
    GPIO1_PIN5_IRQHandler,                    /*   199  Interrupt 199 */
    GPIO1_PIN6_IRQHandler,                    /*   200  Interrupt 200 */
    GPIO1_PIN7_IRQHandler,                    /*   201  Interrupt 201 */
    GPIO1_PIN8_IRQHandler,                    /*   202  Interrupt 202 */
    GPIO1_PIN9_IRQHandler,                    /*   203  Interrupt 203 */
    GPIO1_PIN10_IRQHandler,                   /*   204  Interrupt 204 */
    GPIO1_PIN11_IRQHandler,                   /*   205  Interrupt 205 */
    GPIO1_PIN12_IRQHandler,                   /*   206  Interrupt 206 */
    GPIO1_PIN13_IRQHandler,                   /*   207  Interrupt 207 */
    GPIO1_PIN14_IRQHandler,                   /*   208  Interrupt 208 */
    GPIO1_PIN15_IRQHandler,                   /*   209  Interrupt 209 */
    GPIO1_PIN16_IRQHandler,                   /*   210  Interrupt 210 */
    GPIO1_PIN17_IRQHandler,                   /*   211  Interrupt 211 */
    GPIO1_PIN18_IRQHandler,                   /*   212  Interrupt 212 */
    GPIO1_PIN19_IRQHandler,                   /*   213  Interrupt 213 */
    GPIO1_PIN20_IRQHandler,                   /*   214  Interrupt 214 */
    GPIO1_PIN21_IRQHandler,                   /*   215  Interrupt 215 */
    GPIO1_PIN22_IRQHandler,                   /*   216  Interrupt 216 */
    GPIO1_PIN23_IRQHandler,                   /*   217  Interrupt 217 */
    GPIO1_PIN24_IRQHandler,                   /*   218  Interrupt 218 */
    GPIO1_PIN25_IRQHandler,                   /*   219  Interrupt 219 */
    GPIO1_PIN26_IRQHandler,                   /*   220  Interrupt 220 */
    GPIO1_PIN27_IRQHandler,                   /*   221  Interrupt 221 */
    GPIO1_PIN28_IRQHandler,                   /*   222  Interrupt 222 */
    GPIO1_PIN29_IRQHandler,                   /*   223  Interrupt 223 */
    GPIO1_PIN30_IRQHandler,                   /*   224  Interrupt 224 */
    GPIO1_PIN31_IRQHandler,                   /*   225  Interrupt 225 */
    GPIO2_PIN0_IRQHandler,                    /*   226  Interrupt 226 */
    GPIO2_PIN1_IRQHandler,                    /*   227  Interrupt 227 */
    GPIO2_PIN2_IRQHandler,                    /*   228  Interrupt 228 */
    GPIO2_PIN3_IRQHandler,                    /*   229  Interrupt 229 */
    GPIO2_PIN4_IRQHandler,                    /*   230  Interrupt 230 */
    GPIO2_PIN5_IRQHandler,                    /*   231  Interrupt 231 */
    GPIO2_PIN6_IRQHandler,                    /*   232  Interrupt 232 */
    GPIO2_PIN7_IRQHandler,                    /*   233  Interrupt 233 */
    GPIO2_PIN8_IRQHandler,                    /*   234  Interrupt 234 */
    GPIO2_PIN9_IRQHandler,                    /*   235  Interrupt 235 */
    GPIO2_PIN10_IRQHandler,                   /*   236  Interrupt 236 */
    GPIO2_PIN11_IRQHandler,                   /*   237  Interrupt 237 */
    GPIO2_PIN12_IRQHandler,                   /*   238  Interrupt 238 */
    GPIO2_PIN13_IRQHandler,                   /*   239  Interrupt 239 */
    GPIO2_PIN14_IRQHandler,                   /*   240  Interrupt 240 */
    GPIO2_PIN15_IRQHandler,                   /*   241  Interrupt 241 */
    GPIO2_PIN16_IRQHandler,                   /*   242  Interrupt 242 */
    GPIO2_PIN17_IRQHandler,                   /*   243  Interrupt 243 */
    GPIO2_PIN18_IRQHandler,                   /*   244  Interrupt 244 */
    GPIO2_PIN19_IRQHandler,                   /*   245  Interrupt 245 */
    GPIO2_PIN20_IRQHandler,                   /*   246  Interrupt 246 */
    GPIO2_PIN21_IRQHandler,                   /*   247  Interrupt 247 */
    GPIO2_PIN22_IRQHandler,                   /*   248  Interrupt 248 */
    GPIO2_PIN23_IRQHandler,                   /*   249  Interrupt 249 */
    GPIO2_PIN24_IRQHandler,                   /*   250  Interrupt 250 */
    GPIO2_PIN25_IRQHandler,                   /*   251  Interrupt 251 */
    GPIO2_PIN26_IRQHandler,                   /*   252  Interrupt 252 */
    GPIO2_PIN27_IRQHandler,                   /*   253  Interrupt 253 */
    GPIO2_PIN28_IRQHandler,                   /*   254  Interrupt 254 */
    GPIO2_PIN29_IRQHandler,                   /*   255  Interrupt 255 */
    GPIO2_PIN30_IRQHandler,                   /*   256  Interrupt 256 */
    GPIO2_PIN31_IRQHandler,                   /*   257  Interrupt 257 */
    GPIO3_PIN0_IRQHandler,                    /*   258  Interrupt 258 */
    GPIO3_PIN1_IRQHandler,                    /*   259  Interrupt 259 */
    GPIO3_PIN2_IRQHandler,                    /*   260  Interrupt 260 */
    GPIO3_PIN3_IRQHandler,                    /*   261  Interrupt 261 */
    GPIO3_PIN4_IRQHandler,                    /*   262  Interrupt 262 */
    GPIO3_PIN5_IRQHandler,                    /*   263  Interrupt 263 */
    GPIO3_PIN6_IRQHandler,                    /*   264  Interrupt 264 */
    GPIO3_PIN7_IRQHandler,                    /*   265  Interrupt 265 */
    GPIO3_PIN8_IRQHandler,                    /*   266  Interrupt 266 */
    GPIO3_PIN9_IRQHandler,                    /*   267  Interrupt 267 */
    GPIO3_PIN10_IRQHandler,                   /*   268  Interrupt 268 */
    GPIO3_PIN11_IRQHandler,                   /*   269  Interrupt 269 */
    GPIO3_PIN12_IRQHandler,                   /*   270  Interrupt 270 */
    GPIO3_PIN13_IRQHandler,                   /*   271  Interrupt 271 */
    GPIO3_PIN14_IRQHandler,                   /*   272  Interrupt 272 */
    GPIO3_PIN15_IRQHandler,                   /*   273  Interrupt 273 */
    GPIO3_PIN16_IRQHandler,                   /*   274  Interrupt 274 */
    GPIO3_PIN17_IRQHandler,                   /*   275  Interrupt 275 */
    GPIO3_PIN18_IRQHandler,                   /*   276  Interrupt 276 */
    GPIO3_PIN19_IRQHandler,                   /*   277  Interrupt 277 */
    GPIO3_PIN20_IRQHandler,                   /*   278  Interrupt 278 */
    GPIO3_PIN21_IRQHandler,                   /*   279  Interrupt 279 */
    GPIO3_PIN22_IRQHandler,                   /*   280  Interrupt 280 */
    GPIO3_PIN23_IRQHandler,                   /*   281  Interrupt 281 */
    GPIO3_PIN24_IRQHandler,                   /*   282  Interrupt 282 */
    GPIO3_PIN25_IRQHandler,                   /*   283  Interrupt 283 */
    GPIO3_PIN26_IRQHandler,                   /*   284  Interrupt 284 */
    GPIO3_PIN27_IRQHandler,                   /*   285  Interrupt 285 */
    GPIO3_PIN28_IRQHandler,                   /*   286  Interrupt 286 */
    GPIO3_PIN29_IRQHandler,                   /*   287  Interrupt 287 */
    GPIO3_PIN30_IRQHandler,                   /*   288  Interrupt 288 */
    GPIO3_PIN31_IRQHandler,                   /*   289  Interrupt 289 */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*  290 -  303 */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*  304 -  319 */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*  320 -  335 */
    0,                                              /*  336 -  336 */
    LPTIMER_CHANNEL0_IRQHandler,              /*   337  Interrupt 337 */
    LPTIMER_CHANNEL1_IRQHandler,              /*   338  Interrupt 338 */
    LPTIMER_CHANNEL2_IRQHandler,              /*   339  Interrupt 339 */
    LPTIMER_CHANNEL3_IRQHandler,              /*   340  Interrupt 340 */
    RTC0_IRQHandler                           /*   341  Interrupt 341 */
};

/**
 * SysTick initialisation
 */
int Init_SysTick(void)
{
    const uint32_t ticks_1ms = GetSystemCoreClock()/1000;
    int err = 0;

    /* Reset CPU cycle count value. */
    cpu_cycle_count = 0;

    /* Changing configuration for sys tick => guard from being
     * interrupted. */
    NVIC_DisableIRQ(SysTick_IRQn);

    /* SysTick init - this will enable interrupt too. */
    err = SysTick_Config(ticks_1ms);

    /* Enable interrupt again. */
    NVIC_EnableIRQ(SysTick_IRQn);

    /* Wait for SysTick to kick off */
    while (!err && !SysTick->VAL) {
        __NOP();
    }

    return err;
}

/* Reset handler - starting point of our application. */
__attribute__((used)) void Reset_Handler(void)
{
    /* Initialise system. */
    SystemInit();

    /* Configure the system tick. */
    Init_SysTick();

    /* cmsis supplied entry point. */
    __PROGRAM_START();
}

#ifdef __cplusplus
}
#endif
