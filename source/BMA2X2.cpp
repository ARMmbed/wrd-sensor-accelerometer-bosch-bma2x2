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

#include "wrd-sensor-accelerometer/BMA2X2.h"

BMA2X2::BMA2X2(I2CEx& _i2c, DigitalOutEx& _enable, InterruptInEx& _irq)
    :   i2c(_i2c),
        enable(_enable),
        irq(_irq)
{
    minar::Scheduler::postCallback(this, &BMA2X2::init)
        .delay(minar::milliseconds(100))
        .tolerance(1);
}

BMA2X2::~BMA2X2()
{

}

void BMA2X2::init()
{
    i2c.frequency(400000);

    FunctionPointer0<void> fp(this, &BMA2X2::deviceReady);
    powerOn(fp);
}

void BMA2X2::powerOn(FunctionPointer0<void> callback)
{
    setDone = callback;
    enable.write(1, this, &BMA2X2::powerOnDone);
}

void BMA2X2::powerOnDone()
{
    if (setDone)
    {
        minar::Scheduler::postCallback(this, &BMA2X2::deviceReady)
            .delay(minar::milliseconds(10))
            .tolerance(1);

        setDone.clear();
    }
}

void BMA2X2::deviceReady()
{
    FunctionPointer0<void> fp(this, &BMA2X2::setRangeDone);
    setRange(BMA2X2::RANGE_2G, fp);
}

void BMA2X2::setRangeDone()
{
    FunctionPointer0<void> fp(this, &BMA2X2::setBandwidthDone);
    setBandwidth(BMA2X2::BANDWIDTH_62_5HZ, fp);
}

void BMA2X2::setBandwidthDone()
{
    FunctionPointer0<void> fp(this, &BMA2X2::setFifoDone);
    setFifo(BMA2X2::FIFO_STREAM, fp);
}

void BMA2X2::setFifoDone()
{

}


void BMA2X2::setRange(range_t range, FunctionPointer0<void> callback)
{
    memoryWrite[0] = range;
    i2c.write(BMA2X2::ADDRESS, REG_PMU_RANGE, memoryWrite, 1, callback);
}

void BMA2X2::setBandwidth(bandwidth_t bandwidth, FunctionPointer0<void> callback)
{
    memoryWrite[0] = bandwidth;
    i2c.write(BMA2X2::ADDRESS, REG_PMU_BW, memoryWrite, 1, callback);
}

void BMA2X2::setFifo(fifo_t fifo, FunctionPointer0<void> callback)
{
    memoryWrite[0] = fifo;
    i2c.write(BMA2X2::ADDRESS, REG_FIFO_CONFIG_1, memoryWrite, 1, callback);
}

void BMA2X2::getFifo(uint8_t* buffer, uint8_t length, FunctionPointer0<void> callback)
{
    i2c.read(BMA2X2::ADDRESS, REG_FIFO_DATA, (char*) buffer, length, callback);
}

