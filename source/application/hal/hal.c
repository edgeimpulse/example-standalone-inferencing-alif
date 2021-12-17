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
#include "hal.h"            /* API */

#include "hal_config.h"     /* HAL configuration */
#include "system_init.h"

#include <stdio.h>
#include <assert.h>

#if defined(ARM_NPU)

#include "ethosu_driver.h"              /* Arm Ethos-U driver header */
#include "timing_adapter.h"             /* Arm Ethos-U timing adapter driver header */
#include "timing_adapter_settings.h"    /* Arm Ethos-U timing adapter settings */

extern struct ethosu_driver ethosu_drv; /* Default Ethos-U device driver */

#endif /* ARM_NPU */


#if defined(ARM_NPU)
/**
 * @brief   Defines the Ethos-U interrupt handler: just a wrapper around the default
 *          implementation.
 **/
static void arm_npu_irq_handler(void)
{
    /* Call the default interrupt handler from the NPU driver */
    ethosu_irq_handler(&ethosu_drv);
}

/**
 * @brief  Initialises the NPU IRQ
 **/
static void arm_npu_irq_init(void)
{
    const IRQn_Type ethosu_irqnum = (IRQn_Type)EthosU_IRQn;

    /* Register the EthosU IRQ handler in our vector table.
     * Note, this handler comes from the EthosU driver */
    NVIC_ClearPendingIRQ(336);
    NVIC_SetVector(ethosu_irqnum, (uint32_t)arm_npu_irq_handler);

    /* Enable the IRQ */
    NVIC_EnableIRQ(ethosu_irqnum);

    debug("EthosU IRQ#: %u, Handler: 0x%p\n",
            ethosu_irqnum, arm_npu_irq_handler);
}

static int _arm_npu_timing_adapter_init(void)
{
#if defined (TA0_BASE)
    struct timing_adapter ta_0;
    struct timing_adapter_settings ta_0_settings = {
        .maxr = TA0_MAXR,
        .maxw = TA0_MAXW,
        .maxrw = TA0_MAXRW,
        .rlatency = TA0_RLATENCY,
        .wlatency = TA0_WLATENCY,
        .pulse_on = TA0_PULSE_ON,
        .pulse_off = TA0_PULSE_OFF,
        .bwcap = TA0_BWCAP,
        .perfctrl = TA0_PERFCTRL,
        .perfcnt = TA0_PERFCNT,
        .mode = TA0_MODE,
        .maxpending = 0, /* This is a read-only parameter */
        .histbin = TA0_HISTBIN,
        .histcnt = TA0_HISTCNT
    };

    if (0 != ta_init(&ta_0, TA0_BASE)) {
        printf_err("TA0 initialisation failed\n");
        return 1;
    }

    ta_set_all(&ta_0, &ta_0_settings);
#endif /* defined (TA0_BASE) */

#if defined (TA1_BASE)
    struct timing_adapter ta_1;
    struct timing_adapter_settings ta_1_settings = {
        .maxr = TA1_MAXR,
        .maxw = TA1_MAXW,
        .maxrw = TA1_MAXRW,
        .rlatency = TA1_RLATENCY,
        .wlatency = TA1_WLATENCY,
        .pulse_on = TA1_PULSE_ON,
        .pulse_off = TA1_PULSE_OFF,
        .bwcap = TA1_BWCAP,
        .perfctrl = TA1_PERFCTRL,
        .perfcnt = TA1_PERFCNT,
        .mode = TA1_MODE,
        .maxpending = 0, /* This is a read-only parameter */
        .histbin = TA1_HISTBIN,
        .histcnt = TA1_HISTCNT
    };

    if (0 != ta_init(&ta_1, TA1_BASE)) {
        printf_err("TA1 initialisation failed\n");
        return 1;
    }

    ta_set_all(&ta_1, &ta_1_settings);
#endif /* defined (TA1_BASE) */

    return 0;
}

int arm_npu_init(void)
{
    int err = 0;

    /* If the platform has timing adapter blocks along with Ethos-U core
     * block, initialise them here. */
    if (0 != (err = _arm_npu_timing_adapter_init())) {
        return err;
    }

    /* Initialise the IRQ */
    arm_npu_irq_init();

    /* Initialise Ethos-U device */
    const void * ethosu_base_address = (void *)(ETHOS_U_NPU_BASE);

    if (0 != (err = ethosu_init(
                        &ethosu_drv,            /* Ethos-U driver device pointer */
                        ethosu_base_address,    /* Ethos-U NPU's base address. */
                        NULL,                   /* Pointer to fast mem area - NULL for U55. */
                        0,                      /* Fast mem region size. */
                        1,                      /* Security enable. */
                        1))) {                  /* Privilege enable. */
        printf_err("failed to initalise Ethos-U device\n");
        return err;
    }

    info("Ethos-U device initialised\n");

    /* Get Ethos-U version */
    struct ethosu_version version;
    if (0 != (err = ethosu_get_version(&ethosu_drv, &version))) {
        printf_err("failed to fetch Ethos-U version info\n");
        return err;
    }

    info("Ethos-U version info:\n");
    info("\tArch:       v%u.%u.%u\n", version.id.arch_major_rev,
                                    version.id.arch_minor_rev,
                                    version.id.arch_patch_rev);
    info("\tDriver:     v%u.%u.%u\n", version.id.driver_major_rev,
                                    version.id.driver_minor_rev,
                                    version.id.driver_patch_rev);
    info("\tMACs/cc:    %u\n", (1 << version.cfg.macs_per_cc));
    info("\tCmd stream: v%u\n", version.cfg.cmd_stream_version);
    info("\tSHRAM size: %u\n", version.cfg.shram_size);

    return 0;
}
#endif /* ARM_NPU */
