/*
 * Copyright (c) 2019-2020 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "uart_stdout.h"

void UartStdOutInit(void) {}

unsigned char UartPutc(unsigned char c) {
    (void)c;
    return 0;
}

unsigned char UartGetc(void) {
    return 0;
}

unsigned int GetLine(char *lp, unsigned int len) {
    (void)lp;
    (void)len;
    return 0;
}
