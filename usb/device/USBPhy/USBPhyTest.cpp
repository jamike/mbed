/* mbed Microcontroller Library
 * Copyright (c) 2018-2018 ARM Limited
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

#include <cstddef>
#include "USBPhyTest.h"
#include "USBPhyEvents.h"
#include "Semaphore.h"
#include "mbed_events.h"
#include "mbed_critical.h"

#include <stdio.h>

#include "ISRSerial.h"

ISRSerial serial(USBTX, 2048);

class USBPhyEventsTest : public USBPhyEvents {
public:
    USBPhyEventsTest()
        :   events(NULL), queue(mbed::mbed_event_queue()),
            enabled(false), task(0)
    {

    }

    virtual ~USBPhyEventsTest()
    {

    }

    void enable_events(bool enable)
    {
        rtos::Semaphore sem(0);
        int ret = queue->call(this, &USBPhyEventsTest::_enable_events, &sem, enable);
        MBED_ASSERT(ret != 0);
        sem.wait();
        MBED_ASSERT(enabled == enable);
    }

    void _enable_events(rtos::Semaphore *sem, bool enable)
    {
        core_util_critical_section_enter();
        if (!enable && task) {
            queue->cancel(task);
        }
        enabled = enable;
        core_util_critical_section_exit();

        sem->release();
    }

    virtual void reset()
    {
        events->reset();
    }
    virtual void ep0_setup()
    {
        events->ep0_setup();
    }
    virtual void ep0_out()
    {
        events->ep0_out();
    }

    virtual void ep0_in()
    {
        events->ep0_in();
    }

    virtual void power(bool powered)
    {
        events->power(powered);
    }

    virtual void suspend(bool suspended)
    {
        events->suspend(suspended);
    }

    virtual void sof(int frame_number)
    {
        events->sof(frame_number);
    }

    virtual void out(usb_ep_t endpoint)
    {
        events->out(endpoint);
    }

    virtual void in(usb_ep_t endpoint)
    {
        events->in(endpoint);
    }

    virtual void start_process()
    {
        core_util_critical_section_enter();

        MBED_ASSERT(enabled);
        if (enabled) {
            int ret = task = queue->call_in(1, this, &USBPhyEventsTest::_start_process);
            MBED_ASSERT(ret != 0);
        }

        core_util_critical_section_exit();
    }

    void _start_process()
    {
        MBED_ASSERT(enabled);
        task = 0;
        events->start_process();
    }

    USBPhyEvents *events;
    EventQueue *queue;
    bool enabled;
    int task;

};

USBPhyTest::USBPhyTest(USBPhy *phy): phy(phy), events(new USBPhyEventsTest())
{

}

USBPhyTest::~USBPhyTest()
{
    delete events;
}

void USBPhyTest::init(USBPhyEvents *events)
{
    this->events->events = events;
    this->events->enable_events(true);
    phy->init(this->events);
}

void USBPhyTest::deinit()
{
    phy->deinit();
    this->events->enable_events(false);
}

bool USBPhyTest::powered()
{
    return phy->powered();
}

void USBPhyTest::connect()
{
    phy->connect();
}

void USBPhyTest::disconnect()
{
    phy->disconnect();
}

void USBPhyTest::configure()
{
    phy->configure();
}

void USBPhyTest::unconfigure()
{
    phy->unconfigure();
}

void USBPhyTest::sof_enable()
{
    phy->sof_enable();
}

void USBPhyTest::sof_disable()
{
    phy->sof_disable();
}

void USBPhyTest::set_address(uint8_t address)
{
    phy->set_address(address);
}

void USBPhyTest::remote_wakeup()
{
    phy->remote_wakeup();
}

const usb_ep_table_t* USBPhyTest::endpoint_table()
{
    return phy->endpoint_table();
}

uint32_t USBPhyTest::ep0_set_max_packet(uint32_t max_packet)
{
    return phy->ep0_set_max_packet(max_packet);
}

void USBPhyTest::ep0_setup_read_result(uint8_t *buffer, uint32_t size)
{
    phy->ep0_setup_read_result(buffer, size);
}

void USBPhyTest::ep0_read(uint8_t *data, uint32_t size)
{
    phy->ep0_read(data, size);
}

uint32_t USBPhyTest::ep0_read_result()
{
    return phy->ep0_read_result();
}

void USBPhyTest::ep0_write(uint8_t *buffer, uint32_t size)
{
    phy->ep0_write(buffer, size);
}

void USBPhyTest::ep0_stall()
{
    phy->ep0_stall();
}

bool USBPhyTest::endpoint_add(usb_ep_t endpoint, uint32_t max_packet, usb_ep_type_t type)
{
    return phy->endpoint_add(endpoint, max_packet, type);
}

void USBPhyTest::endpoint_remove(usb_ep_t endpoint)
{
    phy->endpoint_remove(endpoint);
}

void USBPhyTest::endpoint_stall(usb_ep_t endpoint)
{
    phy->endpoint_stall(endpoint);
}

void USBPhyTest::endpoint_unstall(usb_ep_t endpoint)
{
    phy->endpoint_unstall(endpoint);
}

bool USBPhyTest::endpoint_read(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    return phy->endpoint_read(endpoint, data, size);
}

uint32_t USBPhyTest::endpoint_read_result(usb_ep_t endpoint)
{
    return phy->endpoint_read_result(endpoint);
}

bool USBPhyTest::endpoint_write(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    return phy->endpoint_write(endpoint, data, size);
}

void USBPhyTest::endpoint_abort(usb_ep_t endpoint)
{
    phy->endpoint_abort(endpoint);
}

void USBPhyTest::process()
{
    phy->process();
}
