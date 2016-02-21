/*
 * Copyright (c) 2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __BOSCH_BMA2X2_H__
#define __BOSCH_BMA2X2_H__

#include "mbed-drivers/mbed.h"

#include "gpio-switch/DigitalOutEx.h"
#include "gpio-switch/InterruptInEx.h"
#include "wrd-utilities/I2CEx.h"
#include "wrd-utilities/SharedModules.h"

#define MAX_READ (2*3*32)
#define MAX_WRITE 4

using namespace mbed::util;

class BMA2X2
{
public:
    typedef enum {
        RANGE_2G    = 0x03,
        RANGE_4G    = 0x05,
        RANGE_8G    = 0x08,
        RANGE_16G   = 0x0C
    } range_t;

    typedef enum {
        BANDWIDTH_7_81HZ    = 0x08,
        BANDWIDTH_15_63HZ   = 0x09,
        BANDWIDTH_31_25HZ   = 0x0A,
        BANDWIDTH_62_5HZ    = 0x0B,
        BANDWIDTH_125HZ     = 0x0C,
        BANDWIDTH_250HZ     = 0x0D,
        BANDWIDTH_500HZ     = 0x0E,
        BANDWIDTH_1000HZ    = 0x0F
    } bandwidth_t;

    typedef enum {
        FIFO_BYPASS         = 0x00,
        FIFO_FIFO           = (1 << 6),
        FIFO_STREAM         = (2 << 6)
    } fifo_t;

    BMA2X2(I2CEx& _i2c, DigitalOutEx& _enable, InterruptInEx& _irq);
    ~BMA2X2(void);

    void powerOn(FunctionPointer0<void> callback);
    void setRange(range_t range, FunctionPointer0<void> callback);
    void setBandwidth(bandwidth_t bandwidth, FunctionPointer0<void> callback);
    void setFifo(fifo_t fifo, FunctionPointer0<void> callback);

    void getFifo(uint8_t* buffer, uint8_t length, FunctionPointer0<void> callback);

private:
    typedef enum {
        REG_PMU_RANGE       = 0x0F,
        REG_PMU_BW          = 0x10,
        REG_PMU_LPW         = 0x11,
        REG_PMU_LOW_POWER   = 0x12,
        REG_FIFO_STATUS     = 0x0E,
        REG_FIFO_CONFIG_1   = 0x3E,
        REG_FIFO_DATA       = 0x3F
    } register_t;

    typedef enum {
        ADDRESS         = 0x30
    } address_t;

    void init(void);
    void powerOnDone(void);
    void deviceReady(void);
    void setRangeDone(void);
    void setBandwidthDone(void);
    void setFifoDone(void);

    I2CEx& i2c;
    DigitalOutEx enable;
    InterruptInEx irq;
    FunctionPointer0<void> setDone;

    char memoryWrite[MAX_WRITE];
    char memoryRead[MAX_READ];
};

#endif // __BOSCH_BMA2X2_H__
