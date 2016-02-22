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
        irq(_irq),
        currentResolution(98),
        currentInterval(1)
{
    minar::Scheduler::postCallback(this, &BMA2X2::init)
        .delay(minar::milliseconds(10))
        .tolerance(1);
}

BMA2X2::~BMA2X2()
{

}

void BMA2X2::init()
{
    i2c.frequency(400000);
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
        minar::Scheduler::postCallback(setDone)
            .delay(minar::milliseconds(100))
            .tolerance(1);

        setDone.clear();
    }
}

void BMA2X2::setRange(range_t range, FunctionPointer0<void> callback)
{
    switch (range)
    {
        case RANGE_2G:
                        currentResolution = 98;
                        break;
        case RANGE_4G:
                        currentResolution = 195;
                        break;
        case RANGE_8G:
                        currentResolution = 391;
                        break;
        case RANGE_16G:
                        currentResolution = 781;
                        break;
        default:
                        currentResolution = 98;
                        break;
    }

    memoryWrite[0] = range;
    i2c.write(BMA2X2::ADDRESS, REG_PMU_RANGE, memoryWrite, 1, callback);
}

void BMA2X2::setBandwidth(bandwidth_t bandwidth, FunctionPointer0<void> callback)
{
    switch (bandwidth)
    {
        case BANDWIDTH_7_81HZ:
                                currentInterval = 128;
                                break;
        case BANDWIDTH_15_63HZ:
                                currentInterval = 64;
                                break;
        case BANDWIDTH_31_25HZ:
                                currentInterval = 32;
                                break;
        case BANDWIDTH_62_5HZ:
                                currentInterval = 16;
                                break;
        case BANDWIDTH_125HZ:
                                currentInterval = 8;
                                break;
        case BANDWIDTH_250HZ:
                                currentInterval = 4;
                                break;
        case BANDWIDTH_500HZ:
                                currentInterval = 2;
                                break;
        case BANDWIDTH_1000HZ:
        default:
                                currentInterval = 1;
                                break;
    }

    memoryWrite[0] = bandwidth;
    i2c.write(BMA2X2::ADDRESS, REG_PMU_BW, memoryWrite, 1, callback);
}

void BMA2X2::setFifo(fifo_t fifo, FunctionPointer0<void> callback)
{
    memoryWrite[0] = fifo;
    i2c.write(BMA2X2::ADDRESS, REG_FIFO_CONFIG_1, memoryWrite, 1, callback);
}

void BMA2X2::getFifoStatus(FunctionPointer1<void, uint8_t> callback)
{
    getCallback = callback;
    i2c.read(BMA2X2::ADDRESS, REG_FIFO_STATUS, memoryRead, 1, this, &BMA2X2::getFifoStatusDone);
}

void BMA2X2::getFifoStatusDone()
{
    if (getCallback)
    {
        minar::Scheduler::postCallback(getCallback.bind(memoryRead[0]));
        getCallback.clear();
    }
}

void BMA2X2::getFifo(uint8_t* buffer, uint32_t length, FunctionPointer0<void> callback)
{
    i2c.read(BMA2X2::ADDRESS, REG_FIFO_DATA, (char*) buffer, length, callback);
}

void BMA2X2::getRawBuffer(uint8_t* buffer, uint32_t length, FunctionPointer1<void, uint8_t> callback)
{
    getRawBufferPointer = buffer;
    getRawBufferLength = length;
    getRawCallback = callback;

    FunctionPointer1<void, uint8_t> fp(this, &BMA2X2::getRawBuffer1);
    getFifoStatus(fp);
}

void BMA2X2::getRawBuffer1(uint8_t status)
{
    getRawFrames = status & 0x7F;

    /* calculate how many frames to retrieve */
    if (getRawFrames * FRAME_SIZE > getRawBufferLength)
    {
        getRawFrames = getRawBufferLength / FRAME_SIZE;
    }

    /* get frames if possible */
    if (getRawFrames > 0)
    {
        FunctionPointer0<void> fp(this, &BMA2X2::getRawBuffer2);
        getFifo(getRawBufferPointer, getRawFrames * FRAME_SIZE, fp);
    }
    else
    {
        /* signal get done */
        getRawBuffer2();
    }
}

void BMA2X2::getRawBuffer2()
{
    if (getRawCallback)
    {
        minar::Scheduler::postCallback(getRawCallback.bind(getRawFrames));
        getRawCallback.clear();
    }
}

void BMA2X2::getSampleFromBuffer(const uint8_t* buffer, uint8_t index, acceleration_t& measurement)
{
    int16_t x;
    int16_t y;
    int16_t z;

    /*  read frame based on index */
    x =             buffer[index * 6 + 1];
    x = (x << 8) | (buffer[index * 6    ] & 0xF0);
    x = (x >> 4);

    y =             buffer[index * 6 + 3];
    y = (y << 8) | (buffer[index * 6 + 2] & 0xF0);
    y = (y >> 4);

    z =             buffer[index * 6 + 5];
    z = (z << 8) | (buffer[index * 6 + 4] & 0xF0);
    z = (z >> 4);

    /*  convert raw values to mg */
    measurement.x = (x * currentResolution) / 100;
    measurement.y = (y * currentResolution) / 100;
    measurement.z = (z * currentResolution) / 100;

    /*  the offset is calculated with an added 1 so that multiple
        buffers can be chained together.
    */
    measurement.offset = (index + 1) * currentInterval;
}
