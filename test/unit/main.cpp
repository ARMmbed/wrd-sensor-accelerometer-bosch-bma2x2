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

#include "mbed-drivers/mbed.h"

#include "wrd-utilities/I2CEx.h"
#include "gpio-pcal64/PCAL64.h"
#include "gpio-switch/InterruptInEx.h"

#include "wrd-sensor-accelerometer/BMA2X2.h"

InterruptInEx irq(YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_SENSOR_ACCELEROMETER_IRQ_PIN,
                  YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_SENSOR_ACCELEROMETER_IRQ_LOCATION);

I2CEx i2c(YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_SENSOR_ACCELEROMETER_I2C_SDA,
          YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_SENSOR_ACCELEROMETER_I2C_SCL);

BMA2X2 acc(i2c, irq);

PCAL64 ioexpander2(YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_EXTERNAL_GPIO_I2C_SDA,
                   YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_EXTERNAL_GPIO_I2C_SCL,
                   PCAL64::SECONDARY_ADDRESS,
                   YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_EXTERNAL_GPIO_PIN_IRQ2);


void fifoStatus()
{
    acc.getRegister(BMA2X2::REG_FIFO_STATUS);
}

void setup()
{
    acc.setRegister(BMA2X2::REG_FIFO_CONFIG_1, BMA2X2::FIFO_MODE_STREAM);
}

void read()
{
    minar::Scheduler::postCallback(setup)
        .delay(minar::milliseconds(500));

    minar::Scheduler::postCallback(fifoStatus)
        .delay(minar::milliseconds(1000))
        .period(minar::milliseconds(50));
}

void app_start(int, char *[])
{
//    printf("hello world\r\n");

    ioexpander2.set(PCAL64::P0_5, PCAL64::Output)
               .set(PCAL64::P0_5, PCAL64::High)
               .callback(read);
}
