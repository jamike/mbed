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

#ifndef USBPHY_TEST_H
#define USBPHY_TEST_H

#include "USBPhy.h"

class USBPhyEventsTest;

class USBPhyTest : public USBPhy {
public:
    USBPhyTest(USBPhy *phy);
    virtual ~USBPhyTest();

    /**
     * Initialize this USBPhy instance
     *
     * This function must be called before calling
     * any other functions of this class, unless specifically
     * noted.
     *
     * @param events Callback class to handle USB events
     */
    virtual void init(USBPhyEvents *events);

    /**
     * Power down this USBPhy instance
     *
     * Disable interrupts and stop sending events.
     */
    virtual void deinit();

    /**
     * Check if USB power is present
     *
     * Devices which don't support checking the USB power state
     * must always return true.
     *
     * @return true if USB power is present, false otherwise
     */
    virtual bool powered();

    /**
     * Make the USB phy visible to the USB host
     *
     * Enable either the D+ or D-  pullup so the host can detect
     * the presence of this device.
     */
    virtual void connect();

    /**
     * Detach the USB phy
     *
     * Disable the D+ and D- pullup and stop responding to
     * USB traffic.
     */
    virtual void disconnect();

    /**
     * Set this device to the configured state
     *
     * Enable added endpoints if they are not enabled
     * already.
     */
    virtual void configure();

    /**
     * Leave the configured state
     *
     * This is a notification to the USBPhy indicating that the device
     * is leaving the configured state. The USBPhy can disable all
     * endpoints other than endpoint 0.
     *
     */
    virtual void unconfigure();

    /**
     * Enable the start of frame interrupt
     *
     * Call USBPhyEvents::sof on every frame.
     */
    virtual void sof_enable();

    /**
     * Disable the start of frame interrupt
     *
     * Stop calling USBPhyEvents::sof.
     */
    virtual void sof_disable();

    /**
     * Set the USBPhy's address
     *
     * @param address This device's USB address
     */
    virtual void set_address(uint8_t address);

    /**
     * Wake upstream devices
     */
    virtual void remote_wakeup();

    /**
     * Get the endpoint table
     *
     * This function returns a table which describes the endpoints
     * can be used, the functionality of those endpoints and the
     * resource cost.
     */
    virtual const usb_ep_table_t* endpoint_table();

    /**
     * Set wMaxPacketSize of endpoint 0
     *
     * @param max_packet The wMaxPacketSize value for endpoint 0
     * @return The actual size of endpoint 0
     */
    virtual uint32_t ep0_set_max_packet(uint32_t max_packet);

    /**
     * Read the contents of the SETUP packet
     *
     * @param buffer Buffer to fill with data
     * @param size Size of buffer passed in
     */
    virtual void ep0_setup_read_result(uint8_t *buffer, uint32_t size);

    /**
     * Start receiving a packet of up to wMaxPacketSize on endpoint 0
     *
     * @param data Buffer to fill with the data read
     * @param size Size of buffer
     */
    virtual void ep0_read(uint8_t *data, uint32_t size);

    /**
     * Read the contents of a received packet
     *
     * @return Size of data read
     */
    virtual uint32_t ep0_read_result();

    /**
     * Write a packet on endpoint 0
     *
     * @param buffer Buffer fill with data to send
     * @param size Size of data to send
     */
    virtual void ep0_write(uint8_t *buffer, uint32_t size);

    /**
     * Protocol stall on endpoint 0
     *
     * Stall all IN and OUT packets on endpoint 0 until a setup packet
     * is received.
     * @note The stall is cleared automatically when a setup packet is received
     */
    virtual void ep0_stall();

    /**
     * Configure and enable an endpoint
     *
     * @param endpoint Endpoint to configure and enable
     * @param max_packet The maximum packet size that can be sent or received
     * @param type The type of endpoint this should be configured as -
     *  USB_EP_TYPE_BULK, USB_EP_TYPE_INT or USB_EP_TYPE_ISO
     * @note This function cannot be used to configure endpoint 0. That must be done
     * with ep0_set_max_packet
     */
    virtual bool endpoint_add(usb_ep_t endpoint, uint32_t max_packet, usb_ep_type_t type);

    /**
     * Disable an endpoint
     *
     * @param endpoint Endpoint to disable
     */
    virtual void endpoint_remove(usb_ep_t endpoint);

    /**
     * Perform a functional stall on the given endpoint
     *
     * Set the HALT feature for this endpoint so that all further
     * communication is aborted.
     *
     * @param endpoint Endpoint to stall
     */
    virtual void endpoint_stall(usb_ep_t endpoint);

    /**
     * Unstall the endpoint
     *
     * Clear the HALT feature on this endpoint so communication can
     * resume.
     *
     * @param endpoint Endpoint to stall
     */
    virtual void endpoint_unstall(usb_ep_t endpoint);

    /**
     * Start a read on the given endpoint
     *
     * @param endpoint Endpoint to start the read on
     * @param data Buffer to fill with data
     * @param size Size of the read buffer. This must be at least
     *     the max packet size for this endpoint.
     * @return true if the read was successfully started, false otherwise
     */
    virtual bool endpoint_read(usb_ep_t endpoint, uint8_t *data, uint32_t size);

    /**
     * Finish a read on the given endpoint
     *
     * @param endpoint Endpoint to check
     * @return true if data was read false otherwise
     */
    virtual uint32_t endpoint_read_result(usb_ep_t endpoint);

    /**
     * Start a write on the given endpoint
     *
     * @param endpoint Endpoint to write to
     * @param data Buffer to write
     * @param size Size of data to write
     * @return true if the data was prepared for transmit, false otherwise
     */
    virtual bool endpoint_write(usb_ep_t endpoint, uint8_t *data, uint32_t size);

    /**
     * Abort the current transfer if it has not yet been sent
     *
     * @param endpoint Endpoint to abort the transfer on. It is implementation defined
     * if this function has an effect on receive endpoints.
     */
    virtual void endpoint_abort(usb_ep_t endpoint);

    /**
     * Callback used for performing USB processing
     *
     * USBPhy processing should be triggered by calling USBPhyEvents::start_process
     * and done inside process. All USBPhyEvents callbacks aside from
     * USBPhyEvents::start_process must be called in the context of process
     */
    virtual void process();

    USBPhy *phy;
    USBPhyEventsTest *events;
};

#endif
