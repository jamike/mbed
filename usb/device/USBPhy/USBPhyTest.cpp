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
#include <cstdarg>

#include "Mutex.h"

extern rtos::Mutex mut;

#define STRING_STACK_LIMIT    120
class BufferablePrintf {
public:
    BufferablePrintf(rtos::Mutex *mutex, uint32_t size): _mutex(mutex), _buf(new char[size]), _size(size)
    {
        _head = 0;
        _tail = 0;
        _buffer_mode = false;
        _overflow = false;
    }


    void buffer_mode(bool enable)
    {
        enable = false;
        lock();
        if (_buffer_mode == enable) {
            unlock();
            return;
        }

        if (!enable) {
            while (_head != _tail) {
                putchar(_buf[_head]);
                _head = (_head + 1) % _size;
            }
            if (_overflow) {
                puts("\n[overflow]\n");
            }
        }
        _buffer_mode = enable;
        unlock();

    }

    void lock()
    {
        if (_mutex) {
            _mutex->lock();
        }
    }

    void unlock()
    {
        if (_mutex) {
            _mutex->unlock();
        }
    }

    int printf(const char *format, ...)
    {
        lock();
        std::va_list arg;
        va_start(arg, format);
        // ARMCC microlib does not properly handle a size of 0.
        // As a workaround supply a dummy buffer with a size of 1.
        char dummy_buf[1];
        int len = vsnprintf(dummy_buf, sizeof(dummy_buf), format, arg);
        if (len < STRING_STACK_LIMIT) {
            char temp[STRING_STACK_LIMIT];
            vsprintf(temp, format, arg);
            _puts(temp);
        } else {
            char *temp = new char[len + 1];
            vsprintf(temp, format, arg);
            _puts(temp);
            delete[] temp;
        }
        va_end(arg);
        unlock();
        return len;
    }

private:
    void _puts(const char *str)
    {
        while (*str) {
            _putc(*str);
            str++;
        }
    }

    void _putc(char val)
    {
        if (!_buffer_mode) {
            putchar(val);
            return;
        }
        if ((_tail + 1) % _size == _head) {
            _overflow = true;
            return;
        }
        _buf[_tail] = val;
        _tail = (_tail + 1) % _size;
    }

    rtos::Mutex * const _mutex;
    char *const _buf;
    const uint32_t _size;
    uint32_t _head;
    uint32_t _tail;
    bool _buffer_mode;
    bool _overflow;
};

class USBPhyEventsTest : public USBPhyEvents {
public:
    USBPhyEventsTest(USBPhyTest *phy)
        :   phy(phy), events(NULL), queue(mbed::mbed_event_queue()),
            enabled(false), task(0), io(phy->io)
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
        phy->pre_addr = false;
        phy->addressed = false;
        io->buffer_mode(true);

        io->printf("events->reset\r\n");
        events->reset();
    }
    virtual void ep0_setup()
    {
        if (phy->pre_addr) {
            phy->addressed = true;
            io->buffer_mode(false);
        }

        io->printf("events->setup\r\n");
        events->ep0_setup();
    }
    virtual void ep0_out()
    {
        io->printf("events->out\r\n");
        events->ep0_out();
    }

    virtual void ep0_in()
    {
        io->printf("events->in\r\n");
        events->ep0_in();
    }

    virtual void power(bool powered)
    {
        io->printf("events->power\r\n");
        events->power(powered);
    }

    virtual void suspend(bool suspended)
    {
        io->printf("events->suspend\r\n");
        events->suspend(suspended);
    }

    virtual void sof(int frame_number)
    {
        // Too much data
        events->sof(frame_number);
    }

    virtual void out(usb_ep_t endpoint)
    {
        io->printf("events->out\r\n");
        events->out(endpoint);
    }

    virtual void in(usb_ep_t endpoint)
    {
        io->printf("events->in\r\n");
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

    USBPhyTest *phy;
    USBPhyEvents *events;
    EventQueue *queue;
    bool enabled;
    int task;
    BufferablePrintf *io;

};

USBPhyTest::USBPhyTest(USBPhy *phy): phy(phy), io(new BufferablePrintf(&mut, 4096)), events(new USBPhyEventsTest(this)), pre_addr(false), addressed(false)
{

}

USBPhyTest::~USBPhyTest()
{
    delete events;
    delete io;
}

void USBPhyTest::init(USBPhyEvents *events)
{
    printf("phy->init\r\n");
    this->events->events = events;
    this->events->enable_events(true);
    phy->init(this->events);
}

void USBPhyTest::deinit()
{
    printf("phy->deinit\r\n");
    pre_addr = false;
    addressed = false;
    io->buffer_mode(true);

    phy->deinit();
    this->events->enable_events(false);
}

bool USBPhyTest::powered()
{
    io->printf("phy->powered\r\n");
    return phy->powered();
}

void USBPhyTest::connect()
{
    io->printf("phy->connect\r\n");
    phy->connect();
}

void USBPhyTest::disconnect()
{
    io->printf("phy->disconnect\r\n");
    pre_addr = false;
    addressed = false;
    io->buffer_mode(true);

    phy->disconnect();
}

void USBPhyTest::configure()
{
    io->printf("phy->configure\r\n");
    phy->configure();
}

void USBPhyTest::unconfigure()
{
    io->printf("phy->unconfigure\r\n");
    pre_addr = false;
    addressed = false;
    io->buffer_mode(true);

    phy->unconfigure();
}

void USBPhyTest::sof_enable()
{
    io->printf("phy->sof_enable\r\n");
    phy->sof_enable();
}

void USBPhyTest::sof_disable()
{
    io->printf("phy->sof_disable\r\n");
    phy->sof_disable();
}

void USBPhyTest::set_address(uint8_t address)
{
    pre_addr = true;
    io->printf("phy->set_address\r\n");
    phy->set_address(address);
}

void USBPhyTest::remote_wakeup()
{
    io->printf("phy->remote_wakeup\r\n");
    phy->remote_wakeup();
}

const usb_ep_table_t* USBPhyTest::endpoint_table()
{
    io->printf("phy->endpoint_table\r\n");
    return phy->endpoint_table();
}

uint32_t USBPhyTest::ep0_set_max_packet(uint32_t max_packet)
{
    io->printf("phy->ep0_set_max_packet\r\n");
    return phy->ep0_set_max_packet(max_packet);
}

void USBPhyTest::ep0_setup_read_result(uint8_t *buffer, uint32_t size)
{
    io->printf("phy->ep0_setup_read_result\r\n");
    phy->ep0_setup_read_result(buffer, size);
}

void USBPhyTest::ep0_read(uint8_t *data, uint32_t size)
{
    io->printf("phy->ep0_read\r\n");
    phy->ep0_read(data, size);
}

uint32_t USBPhyTest::ep0_read_result()
{
    io->printf("phy->ep0_read_result\r\n");
    return phy->ep0_read_result();
}

void USBPhyTest::ep0_write(uint8_t *buffer, uint32_t size)
{
    io->printf("phy->ep0_write\r\n");
    phy->ep0_write(buffer, size);
}

void USBPhyTest::ep0_stall()
{
    io->printf("phy->ep0_stall\r\n");
    phy->ep0_stall();
}

bool USBPhyTest::endpoint_add(usb_ep_t endpoint, uint32_t max_packet, usb_ep_type_t type)
{
    io->printf("phy->endpoint_add\r\n");
    return phy->endpoint_add(endpoint, max_packet, type);
}

void USBPhyTest::endpoint_remove(usb_ep_t endpoint)
{
    io->printf("phy->endpoint_remove\r\n");
    phy->endpoint_remove(endpoint);
}

void USBPhyTest::endpoint_stall(usb_ep_t endpoint)
{
    io->printf("phy->endpoint_stall\r\n");
    phy->endpoint_stall(endpoint);
}

void USBPhyTest::endpoint_unstall(usb_ep_t endpoint)
{
    io->printf("phy->endpoint_unstall\r\n");
    phy->endpoint_unstall(endpoint);
}

bool USBPhyTest::endpoint_read(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    io->printf("phy->endpoint_read\r\n");
    return phy->endpoint_read(endpoint, data, size);
}

uint32_t USBPhyTest::endpoint_read_result(usb_ep_t endpoint)
{
    io->printf("phy->endpoint_read_result\r\n");
    return phy->endpoint_read_result(endpoint);
}

bool USBPhyTest::endpoint_write(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    io->printf("phy->endpoint_write\r\n");
    return phy->endpoint_write(endpoint, data, size);
}

void USBPhyTest::endpoint_abort(usb_ep_t endpoint)
{
    io->printf("phy->endpoint_abort\r\n");
    phy->endpoint_abort(endpoint);
}

void USBPhyTest::process()
{
    phy->process();
}
