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


#if defined(TARGET_RZ_A1XX)
/*
  This class can use the pipe1, pipe3 and pipe6 only. You should
  re-program this class if you wanted to use other pipe.
 */

/*************************************************************************/
extern "C"
{
#include "r_typedefs.h"
#include "iodefine.h"
}
#include "USBPhyHw.h"
#include "devdrv_usb_function_api.h"
#include "usb_iobitmask.h"
#include "rza_io_regrw.h"
#include "USBDevice_Types.h"
#include "usb_function_setting.h"
#include "USBEndpoints_RZ_A1.h"
#include "greentea-client/test_env.h"

/*************************************************************************/
/* constants */
const struct PIPECFGREC {
    uint16_t    endpoint;
    uint16_t    pipesel;
    uint16_t    pipecfg;
    uint16_t    pipebuf;
    uint16_t    pipemaxp;
    uint16_t    pipeperi;
} def_pipecfg[] = {
    /*EP0OUT and EP0IN are configured by USB IP*/
    {
        EP1OUT, /*EP1: Host -> Func, INT*/
        6 | USB_FUNCTION_D0FIFO_USE,
        USB_FUNCTION_INTERRUPT |
        USB_FUNCTION_BFREOFF   |
        USB_FUNCTION_DBLBOFF   |
        USB_FUNCTION_CNTMDON   |
        USB_FUNCTION_SHTNAKOFF |
        USB_FUNCTION_DIR_P_OUT |
        USB_FUNCTION_EP1,
        ( ( (  64) / 64 - 1 ) << 10 ) | 0x04u,
        MAX_PACKET_SIZE_EP1,
        DEVDRV_USBF_OFF |
        ( 3 << USB_PIPEPERI_IITV_SHIFT ),
    },
    {
        EP1IN,  /*EP1: Host <- Func, INT*/
        7 | USB_FUNCTION_D1FIFO_USE,
        USB_FUNCTION_INTERRUPT |
        USB_FUNCTION_BFREOFF   |
        USB_FUNCTION_DBLBOFF   |
        USB_FUNCTION_CNTMDOFF  |
        USB_FUNCTION_SHTNAKOFF |
        USB_FUNCTION_DIR_P_IN  |
        USB_FUNCTION_EP1,
        ( ( (  64) / 64 - 1 ) << 10 ) | 0x05u,
        MAX_PACKET_SIZE_EP1,
        DEVDRV_USBF_OFF |
        ( 3 << USB_PIPEPERI_IITV_SHIFT ),
    },
    {
        EP2OUT, /*EP2: Host -> Func, BULK*/
        3 | USB_FUNCTION_D0FIFO_USE,
        USB_FUNCTION_BULK      |
        USB_FUNCTION_BFREOFF   |
        USB_FUNCTION_DBLBON    |
        USB_FUNCTION_CNTMDON   |
        USB_FUNCTION_SHTNAKON  |
        USB_FUNCTION_DIR_P_OUT |
        USB_FUNCTION_EP2,
        ( ( (2048) / 64 - 1 ) << 10 ) | 0x30u,
        MAX_PACKET_SIZE_EP2,
        DEVDRV_USBF_OFF |
        ( 0 << USB_PIPEPERI_IITV_SHIFT ),
    },
    {
        EP2IN,  /*EP2: Host <- Func, BULK*/
        4 | USB_FUNCTION_D1FIFO_USE,
        USB_FUNCTION_BULK      |
        USB_FUNCTION_BFREOFF   |
        USB_FUNCTION_DBLBOFF   |
        USB_FUNCTION_CNTMDON   |
        USB_FUNCTION_SHTNAKOFF |
        USB_FUNCTION_DIR_P_IN  |
        USB_FUNCTION_EP2,
        ( ( (2048) / 64 - 1 ) << 10 ) | 0x50u,
        MAX_PACKET_SIZE_EP2,
        DEVDRV_USBF_OFF |
        ( 0 << USB_PIPEPERI_IITV_SHIFT ),
    },
    {
        EP3OUT, /*EP3: Host -> Func, ISO*/
        1 | USB_FUNCTION_D0FIFO_USE,
        USB_FUNCTION_ISO       |
        USB_FUNCTION_BFREOFF   |
        USB_FUNCTION_DBLBON    |
        USB_FUNCTION_CNTMDOFF  |
        USB_FUNCTION_SHTNAKON  |
        USB_FUNCTION_DIR_P_OUT |
        USB_FUNCTION_EP3,
        ( ( ( 512) / 64 - 1 ) << 10 ) | 0x10u,
        MAX_PACKET_SIZE_EP3,
        DEVDRV_USBF_OFF |
        ( 0 << USB_PIPEPERI_IITV_SHIFT ),
    },
    {
        EP3IN,  /*EP3: Host <- Func, ISO*/
        2 | USB_FUNCTION_D1FIFO_USE,
        USB_FUNCTION_ISO       |
        USB_FUNCTION_BFREOFF   |
        USB_FUNCTION_DBLBON    |
        USB_FUNCTION_CNTMDOFF  |
        USB_FUNCTION_SHTNAKOFF |
        USB_FUNCTION_DIR_P_IN  |
        USB_FUNCTION_EP3,
        ( ( ( 512) / 64 - 1 ) << 10 ) | 0x20u,
        MAX_PACKET_SIZE_EP3,
        DEVDRV_USBF_OFF |
        ( 0 << USB_PIPEPERI_IITV_SHIFT ),
    },
    { /*terminator*/
        0, 0, 0, 0, 0,
    },
};


/*************************************************************************/
/* workareas */
static int write_wait = 0;
static uint16_t     EP0_read_status;
static uint16_t     EPx_read_status;
static uint16_t setup_buffer[MAX_PACKET_SIZE_EP0 / 2];
volatile static uint16_t    recv_error;


/*************************************************************************/
/* macros */

/******************************************************************************
 * Function Name: EP2PIPE
 * Description  : Converts from endpoint to pipe
 * Arguments    : number of endpoint
 * Return Value : number of pipe
 *****************************************************************************/
/*EP2PIPE converter is for pipe1, pipe3 and pipe6 only.*/
#define EP2PIPE(endpoint)   ((uint32_t)usbx_function_EpToPipe(endpoint))

/******************************************************************************
 * Function Name: usbx_function_save_request
 * Description  : Retains the USB request information in variables.
 * Arguments    : none
 * Return Value : none
 *****************************************************************************/
#define  usbx_function_save_request()                       \
    {                                                       \
        uint16_t *bufO = &setup_buffer[0];                  \
                                                            \
        USB20X.INTSTS0 = (uint16_t)~USB_FUNCTION_BITVALID;  \
        /*data[0] <= bmRequest, data[1] <= bmRequestType */ \
        *bufO++ = USB20X.USBREQ;                            \
        /*data[2] data[3] <= wValue*/                       \
        *bufO++ = USB20X.USBVAL;                            \
        /*data[4] data[5] <= wIndex*/                       \
        *bufO++ = USB20X.USBINDX;                           \
        /*data[6] data[6] <= wIndex*/                       \
        *bufO++ = USB20X.USBLENG;                           \
    }

/*************************************************************************/
/*************************************************************************/


static USBPhyHw *instance;

USBPhy *get_usb_phy()
{
    static USBPhyHw usbphy;
    return &usbphy;
}

USBPhyHw::USBPhyHw(): events(NULL)
{

}

USBPhyHw::~USBPhyHw()
{

}

void USBPhyHw::init(USBPhyEvents *events)
{
    this->events = events;

    // Disable IRQ
    GIC_DisableIRQ(USBIX_IRQn);

    /* some constants */
    EP0_read_status = DEVDRV_USBF_WRITEEND;
    EPx_read_status = DEVDRV_USBF_PIPE_DONE;

    /* registers me */
    instance = this;

    /* Clear pipe table */
    usbx_function_clear_pipe_tbl();

    /* The clock of USB0 modules is permitted */
#if (USB_FUNCTION_CH == 0)
    CPG.STBCR7 &= ~(CPG_STBCR7_MSTP71);
#else
    CPG.STBCR7 &= ~(CPG_STBCR7_MSTP71 | CPG_STBCR7_MSTP70);
#endif
    volatile uint8_t    dummy8;
    dummy8 = CPG.STBCR7;
    (void)dummy8;

    /* reset USB module with setting tranciever and HSE=1 */
    usbx_function_reset_module(USBFCLOCK_X1_48MHZ);

    /* clear variables */
    usbx_function_init_status();

    /* select USB Function and Interrupt Enable */
    /* Detect USB Device to attach or detach    */
#if (USB_FUNCTION_HISPEED == 0)
    usbx_function_InitModule(USB_FUNCTION_FULL_SPEED);
#else
    usbx_function_InitModule(USB_FUNCTION_HIGH_SPEED);
#endif

    volatile uint16_t buf;
    buf  = USB20X.INTENB0;
    //buf |= USB_INTENB0_SOFE;
    USB20X.INTENB0 = buf;

    greentea_send_kv("USBPhyHw::init", 1);
    InterruptHandlerRegister(USBIX_IRQn, &_usbisr);
    GIC_SetPriority(USBIX_IRQn, 16);
    GIC_SetConfiguration(USBIX_IRQn, 1);
    GIC_EnableIRQ(USBIX_IRQn);
}

void USBPhyHw::deinit()
{
    // Disconnect and disable interrupt
    disconnect();
    GIC_DisableIRQ(USBIX_IRQn);
}

bool USBPhyHw::powered()
{
    // TODO - return true if powered false otherwise. Devices which don't support
    //    this should always return true
    return true;
}

void USBPhyHw::connect()
{
    // TODO - Enable endpoint interrupts

    // TODO - Enable pullup on D+
}

void USBPhyHw::disconnect()
{
    // TODO - Disable all endpoints

    // TODO - Clear all endpoint interrupts

    // TODO - Disable pullup on D+
}

void USBPhyHw::configure()
{
    // TODO - set device to configured. Most device will not need this
}

void USBPhyHw::unconfigure()
{
    // TODO - set device to unconfigured. Most device will not need this
}

void USBPhyHw::sof_enable()
{
    // TODO - Enable SOF interrupt
}

void USBPhyHw::sof_disable()
{
    // TODO - Disable SOF interrupt
}

void USBPhyHw::set_address(uint8_t address)
{
    greentea_send_kv("USBPhyHw::set_address =", address);
    if (address <= 127) {
        usbx_function_set_pid_buf(USB_FUNCTION_PIPE0);      /* OK */
    } else {
        usbx_function_set_pid_stall(USB_FUNCTION_PIPE0);    /* Not Spec */
    }
}

void USBPhyHw::remote_wakeup()
{
    // TODO - Sent remote wakeup over USB lines (if supported)
}

const usb_ep_table_t *USBPhyHw::endpoint_table()
{
    // TODO - Update the endpoint table for what your device supports

    static const usb_ep_table_t rza1_table = {
        1, // No cost per endpoint - everything allocated up front
        {
            {USB_EP_ATTR_ALLOW_CTRL | USB_EP_ATTR_DIR_IN_AND_OUT, 0, 0},
            {USB_EP_ATTR_ALLOW_INT  | USB_EP_ATTR_DIR_IN_AND_OUT, 0, 0},
            {USB_EP_ATTR_ALLOW_BULK | USB_EP_ATTR_DIR_IN_AND_OUT, 0, 0},
            {USB_EP_ATTR_ALLOW_ISO  | USB_EP_ATTR_DIR_IN_AND_OUT, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
        }
    };
    return &rza1_table;
}

uint32_t USBPhyHw::ep0_set_max_packet(uint32_t max_packet)
{
    return MAX_PACKET_SIZE_EP0;
}

// read setup packet
void USBPhyHw::ep0_setup_read_result(uint8_t *buffer, uint32_t size)
{
    memcpy(buffer, setup_buffer, size);
}

void USBPhyHw::ep0_read(uint8_t *data, uint32_t size)
{
    /* remain of last writing */
    while (EP0_read_status != DEVDRV_USBF_WRITEEND) {
        static uint8_t bbb[2] = { 255, 255 };
        ep0_write(&bbb[0], 0);
    }
    if (data != NULL) {
        usbx_api_function_CtrlWriteStart(size, data);
    } else {
        g_usbx_function_data_count[USB_FUNCTION_PIPE0]   = size;
        g_usbx_function_data_pointer[USB_FUNCTION_PIPE0] = data;
    }
}

uint32_t USBPhyHw::ep0_read_result()
{
    return g_usbx_function_PipeDataSize[USB_FUNCTION_PIPE0];
}

void USBPhyHw::ep0_write(uint8_t *buffer, uint32_t size)
{
    /* zero byte writing */
    if ((size == 0) && (EP0_read_status == DEVDRV_USBF_WRITEEND)) {
        return;
    }

    if (EP0_read_status == DEVDRV_USBF_WRITEEND) {
        /*1st block*/
        EP0_read_status = usbx_api_function_CtrlReadStart(size, buffer);
    } else {
        /* waits the last transmission */
        /*other blocks*/
        g_usbx_function_data_count[ USB_FUNCTION_PIPE0 ]    = size;
        g_usbx_function_data_pointer [ USB_FUNCTION_PIPE0 ] = buffer;
        EP0_read_status = usbx_function_write_buffer_c(USB_FUNCTION_PIPE0);
    }
    /*max size may be deblocking outside*/
    if (size == MAX_PACKET_SIZE_EP0) {
        EP0_read_status = DEVDRV_USBF_WRITING;
    }
}

void USBPhyHw::ep0_stall()
{
    uint16_t ret;
    
    greentea_send_kv("USBPhyHw::ep0_stall", 1);
    ret = usbx_function_clear_pid_stall(0);
    greentea_send_kv("USBPhyHw::ep0_stall", 2);
    greentea_send_kv("USBPhyHw::ep0_stall ret =", ret);
}

bool USBPhyHw::endpoint_add(usb_ep_t endpoint, uint32_t max_packet, usb_ep_type_t type)
{
    const struct PIPECFGREC *cfg;
    uint16_t pipe;
    uint16_t buf;

    if ((EP0OUT == endpoint) || (EP0IN  == endpoint)) {
        return true;
    }

    for (cfg = &def_pipecfg[0]; cfg->pipesel != 0; cfg++) {
        if (cfg->endpoint == endpoint) {
            break;
        }
    }
    if (cfg->pipesel == 0) {
        return false;
    }

    pipe = ((cfg->pipesel & USB_PIPESEL_PIPESEL) >> USB_PIPESEL_PIPESEL_SHIFT);

    g_usbx_function_PipeTbl[ pipe ] = (uint16_t)(endpoint | ((cfg->pipesel & USB_FUNCTION_FIFO_USE) << 0));

    /* There are maintenance routine of SHTNAK and BFRE bits
     * in original sample program. This sample is not
     * programmed. Do maintenance the "def_pipecfg" array if
     * you want it. */

    /* Interrupt Disable */
    buf  = USB20X.BRDYENB;
    buf &= (uint16_t)~g_usbx_function_bit_set[pipe];
    USB20X.BRDYENB = buf;
    buf  = USB20X.NRDYENB;
    buf &= (uint16_t)~g_usbx_function_bit_set[pipe];
    USB20X.NRDYENB = buf;
    buf  = USB20X.BEMPENB;
    buf &= (uint16_t)~g_usbx_function_bit_set[pipe];
    USB20X.BEMPENB = buf;

    usbx_function_set_pid_nak(pipe);

    /* CurrentPIPE Clear */
    if (RZA_IO_RegRead_16(&USB20X.CFIFOSEL, USB_CFIFOSEL_CURPIPE_SHIFT, USB_CFIFOSEL_CURPIPE) == pipe) {
        RZA_IO_RegWrite_16(&USB20X.CFIFOSEL, 0, USB_CFIFOSEL_CURPIPE_SHIFT, USB_CFIFOSEL_CURPIPE);
    }

    if (RZA_IO_RegRead_16(&USB20X.D0FIFOSEL, USB_DnFIFOSEL_CURPIPE_SHIFT, USB_DnFIFOSEL_CURPIPE) == pipe) {
        RZA_IO_RegWrite_16(&USB20X.D0FIFOSEL, 0, USB_DnFIFOSEL_CURPIPE_SHIFT, USB_DnFIFOSEL_CURPIPE);
    }

    if (RZA_IO_RegRead_16(&USB20X.D1FIFOSEL, USB_DnFIFOSEL_CURPIPE_SHIFT, USB_DnFIFOSEL_CURPIPE) == pipe) {
        RZA_IO_RegWrite_16(&USB20X.D1FIFOSEL, 0, USB_DnFIFOSEL_CURPIPE_SHIFT, USB_DnFIFOSEL_CURPIPE);
    }

    /* PIPE Configuration */
    USB20X.PIPESEL  = pipe;
    USB20X.PIPECFG  = cfg->pipecfg;
    USB20X.PIPEBUF  = cfg->pipebuf;
    USB20X.PIPEMAXP = cfg->pipemaxp;
    USB20X.PIPEPERI = cfg->pipeperi;

    g_usbx_function_pipecfg[pipe]  = cfg->pipecfg;
    g_usbx_function_pipebuf[pipe]  = cfg->pipebuf;
    g_usbx_function_pipemaxp[pipe] = cfg->pipemaxp;
    g_usbx_function_pipeperi[pipe] = cfg->pipeperi;

    /* Buffer Clear */
    usbx_function_set_sqclr(pipe);
    usbx_function_aclrm(pipe);

    /* init Global */
    g_usbx_function_pipe_status[pipe]  = DEVDRV_USBF_PIPE_IDLE;
    g_usbx_function_PipeDataSize[pipe] = 0;

    return true;
}

void USBPhyHw::endpoint_remove(usb_ep_t endpoint)
{
    // TODO - disable and remove this endpoint
}

void USBPhyHw::endpoint_stall(usb_ep_t endpoint)
{
    uint32_t pipe = EP2PIPE(endpoint);

    usbx_function_clear_pid_stall(pipe);
}

void USBPhyHw::endpoint_unstall(usb_ep_t endpoint)
{
    uint32_t pipe = EP2PIPE(endpoint);

    usbx_function_set_pid_stall(pipe);
}

bool USBPhyHw::endpoint_read(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    uint32_t    pipe = EP2PIPE(endpoint);
    uint32_t    pipe_size;
    uint16_t    pipe_status;
    bool status;

    g_usbx_function_data_count[pipe]   = size;
    g_usbx_function_data_pointer[pipe] = data;

    pipe_status = usbx_api_function_check_pipe_status(pipe, &pipe_size);

    switch (pipe_status) {
        case DEVDRV_USBF_PIPE_IDLE:
        case DEVDRV_USBF_PIPE_WAIT:
            usbx_api_function_set_pid_nak(pipe);
            usbx_api_function_clear_pipe_status(pipe);

            usbx_api_function_start_receive_transfer(pipe, size, data);
            status = true;
            break;

        default:
            status = false;
            break;
    }

    return status;
}

uint32_t USBPhyHw::endpoint_read_result(usb_ep_t endpoint)
{
    uint32_t pipe = EP2PIPE(endpoint);
    uint16_t pipe_status;
    uint16_t err;
    uint32_t bytesRead;

    if (EPx_read_status != DEVDRV_USBF_PIPE_WAIT) {
        return 0;
    }

    pipe_status = usbx_api_function_check_pipe_status(pipe, &g_usbx_function_data_count[pipe]);
    switch (pipe_status) {
        case DEVDRV_USBF_PIPE_WAIT:
            break;
        case DEVDRV_USBF_PIPE_IDLE:
        case DEVDRV_USBF_PIPE_DONE:
        default:
            return 0;
    }

    /* receives data from pipe */
    err = usbx_function_read_buffer(pipe);
    recv_error = err;
    switch (err) {
        case USB_FUNCTION_READEND:
        case USB_FUNCTION_READSHRT:
        case USB_FUNCTION_READOVER:
            bytesRead = g_usbx_function_PipeDataSize[pipe];
            break;

        case USB_FUNCTION_READING:
        case DEVDRV_USBF_FIFOERROR:
            break;
    }

    pipe_status = usbx_api_function_check_pipe_status(pipe, &bytesRead);
    switch (pipe_status) {
        case DEVDRV_USBF_PIPE_DONE:
            break;

        case DEVDRV_USBF_PIPE_IDLE:
        case DEVDRV_USBF_PIPE_NORES:
        case DEVDRV_USBF_PIPE_STALL:
        case DEVDRV_USBF_FIFOERROR:
        default:
            bytesRead = 0;
            break;
    }

    return bytesRead;


}

bool USBPhyHw::endpoint_write(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    uint32_t pipe = EP2PIPE(endpoint);
    uint32_t pipe_size;
    uint16_t pipe_status;
    uint16_t err;
    uint16_t count;
    bool status = true;

    pipe_status = usbx_api_function_check_pipe_status(pipe, &pipe_size);

    /* waits the last transmission */
    count = 30000;
    while ((pipe_status == DEVDRV_USBF_PIPE_WAIT) || (pipe_status == DEVDRV_USBF_PIPE_DONE)) {
        pipe_status = usbx_api_function_check_pipe_status(pipe, &pipe_size);
        if (--count == 0) {
            pipe_status = DEVDRV_USBF_PIPE_STALL;
            break;
        }
    }

    switch (pipe_status) {
        case DEVDRV_USBF_PIPE_IDLE:
            write_wait = 1;
            err = usbx_api_function_start_send_transfer(pipe, size, data);

            switch (err) {
                    /* finish to write */
                case DEVDRV_USBF_WRITEEND:
                    /* finish to write, but data is short */
                case DEVDRV_USBF_WRITESHRT:
                    /* continue to write */
                case DEVDRV_USBF_WRITING:
                    /* use DMA */
                case DEVDRV_USBF_WRITEDMA:
                    /* error */
                case DEVDRV_USBF_FIFOERROR:
                    status = true;
                    break;
            }
            break;

        case DEVDRV_USBF_PIPE_WAIT:
        case DEVDRV_USBF_PIPE_DONE:
            status = true;
            break;

        case DEVDRV_USBF_PIPE_NORES:
        case DEVDRV_USBF_PIPE_STALL:
        default:
            status = false;
            break;
    }

    return status;
}

void USBPhyHw::endpoint_abort(usb_ep_t endpoint)
{
    // TODO - stop the current transfer on this endpoint and don't call the IN or OUT callback
}

void USBPhyHw::process()
{
    uint16_t            int_sts0;
    uint16_t            int_sts1;
    uint16_t            int_sts2;
    uint16_t            int_sts3;
    uint16_t            int_enb0;
    uint16_t            int_enb2;
    uint16_t            int_enb3;
    uint16_t            int_enb4;
    volatile uint16_t   dumy_sts;


    int_sts0 = USB20X.INTSTS0;

    if (!(int_sts0 & (
                USB_FUNCTION_BITVBINT |
                USB_FUNCTION_BITRESM  |
                USB_FUNCTION_BITSOFR  |
                USB_FUNCTION_BITDVST  |
                USB_FUNCTION_BITCTRT  |
                USB_FUNCTION_BITBEMP  |
                USB_FUNCTION_BITNRDY  |
                USB_FUNCTION_BITBRDY ))) {
        // Re-enable interrupt
        GIC_ClearPendingIRQ(USBIX_IRQn);
        GIC_EnableIRQ(USBIX_IRQn);
        return;
    }

    int_sts1 = USB20X.BRDYSTS;
    int_sts2 = USB20X.NRDYSTS;
    int_sts3 = USB20X.BEMPSTS;
    int_enb0 = USB20X.INTENB0;
    int_enb2 = USB20X.BRDYENB;
    int_enb3 = USB20X.NRDYENB;
    int_enb4 = USB20X.BEMPENB;

    if ((int_sts0 & USB_FUNCTION_BITRESM) &&
            (int_enb0 & USB_FUNCTION_BITRSME)) {
        USB20X.INTSTS0 = (uint16_t)~USB_FUNCTION_BITRESM;
        RZA_IO_RegWrite_16(&USB20X.INTENB0, 0, USB_INTENB0_RSME_SHIFT, USB_INTENB0_RSME);
        /*usbx_function_USB_FUNCTION_Resume();*/
        events->suspend(true);
    } else if (
        (int_sts0 & USB_FUNCTION_BITVBINT) &&
        (int_enb0 & USB_FUNCTION_BITVBSE)) {
        USB20X.INTSTS0 = (uint16_t)~USB_FUNCTION_BITVBINT;

        if (usbx_function_CheckVBUStaus() == DEVDRV_USBF_ON) {
            usbx_function_USB_FUNCTION_Attach();
        } else {
            usbx_function_USB_FUNCTION_Detach();
        }
    } else if (
        (int_sts0 & USB_FUNCTION_BITSOFR) &&
        (int_enb0 & USB_FUNCTION_BITSOFE)) {
        USB20X.INTSTS0 = (uint16_t)~USB_FUNCTION_BITSOFR;
        events->sof((USB20X.FRMNUM & USB_FRMNUM_FRNM) >> USB_FRMNUM_FRNM_SHIFT);
    } else if (
        (int_sts0 & USB_FUNCTION_BITDVST) &&
        (int_enb0 & USB_FUNCTION_BITDVSE)) {
        USB20X.INTSTS0 = (uint16_t)~USB_FUNCTION_BITDVST;
        switch (int_sts0 & USB_FUNCTION_BITDVSQ) {
            case USB_FUNCTION_DS_POWR:
                break;

            case USB_FUNCTION_DS_DFLT:
                /*****************************************************************************
                 * Function Name: usbx_function_USB_FUNCTION_BusReset
                 * Description  : This function is executed when the USB device is transitioned
                 *              : to POWERD_STATE. Sets the device descriptor according to the
                 *              : connection speed determined by the USB reset hand shake.
                 * Arguments    : none
                 * Return Value : none
                 *****************************************************************************/
                usbx_function_init_status();            /* memory clear */

                /* Default Control PIPE reset */
                /*****************************************************************************
                 * Function Name: usbx_function_ResetDCP
                 * Description  : Initializes the default control pipe(DCP).
                 * Outline      : Reset default control pipe
                 * Arguments    : none
                 * Return Value : none
                 *****************************************************************************/
                USB20X.DCPCFG  = 0;
                USB20X.DCPMAXP = 64;    /*TODO: This value is copied from sample*/

                USB20X.CFIFOSEL  = (uint16_t)(USB_FUNCTION_BITMBW_8 | USB_FUNCTION_BITBYTE_LITTLE);
                USB20X.D0FIFOSEL = (uint16_t)(USB_FUNCTION_BITMBW_8 | USB_FUNCTION_BITBYTE_LITTLE);
                USB20X.D1FIFOSEL = (uint16_t)(USB_FUNCTION_BITMBW_8 | USB_FUNCTION_BITBYTE_LITTLE);

                events->reset();
                break;

            case USB_FUNCTION_DS_ADDS:
                break;

            case USB_FUNCTION_DS_CNFG:
                break;

            case USB_FUNCTION_DS_SPD_POWR:
            case USB_FUNCTION_DS_SPD_DFLT:
            case USB_FUNCTION_DS_SPD_ADDR:
            case USB_FUNCTION_DS_SPD_CNFG:
                events->suspend(false);
                /*usbx_function_USB_FUNCTION_Suspend();*/
                break;

            default:
                break;
        }
    } else if (
        (int_sts0 & USB_FUNCTION_BITBEMP) &&
        (int_enb0 & USB_FUNCTION_BITBEMP) &&
        ((int_sts3 & int_enb4) & g_usbx_function_bit_set[USB_FUNCTION_PIPE0])) {
        /* ==== BEMP PIPE0 ==== */
        USB20X.BEMPSTS =
            (uint16_t)~g_usbx_function_bit_set[USB_FUNCTION_PIPE0];
        RZA_IO_RegWrite_16(
            &USB20X.CFIFOSEL, USB_FUNCTION_PIPE0,
            USB_CFIFOSEL_CURPIPE_SHIFT, USB_CFIFOSEL_CURPIPE);

        /*usbx_function_write_buffer_c(USB_FUNCTION_PIPE0);*/
        events->ep0_in();

        /* Three dummy reads for clearing interrupt requests */
        dumy_sts = USB20X.BEMPSTS;
    } else if (
        (int_sts0 & USB_FUNCTION_BITBRDY) &&
        (int_enb0 & USB_FUNCTION_BITBRDY) &&
        ((int_sts1 & int_enb2) & g_usbx_function_bit_set[USB_FUNCTION_PIPE0])) {
        /* ==== BRDY PIPE0 ==== */
        USB20X.BRDYSTS =
            (uint16_t)~g_usbx_function_bit_set[USB_FUNCTION_PIPE0];
        RZA_IO_RegWrite_16(
            &USB20X.CFIFOSEL, USB_FUNCTION_PIPE0,
            USB_CFIFOSEL_CURPIPE_SHIFT, USB_CFIFOSEL_CURPIPE);

        g_usbx_function_PipeDataSize[USB_FUNCTION_PIPE0] =
            g_usbx_function_data_count[USB_FUNCTION_PIPE0];

        uint16_t read_status = usbx_function_read_buffer_c(USB_FUNCTION_PIPE0);

        g_usbx_function_PipeDataSize[USB_FUNCTION_PIPE0] -=
            g_usbx_function_data_count[USB_FUNCTION_PIPE0];

        switch (read_status) {
            case USB_FUNCTION_READING:      /* Continue of data read */
            case USB_FUNCTION_READEND:      /* End of data read */
                /* PID = BUF */
                usbx_function_set_pid_buf(USB_FUNCTION_PIPE0);

                /*callback*/
                events->ep0_out();
                break;

            case USB_FUNCTION_READSHRT:     /* End of data read */
                usbx_function_disable_brdy_int(USB_FUNCTION_PIPE0);
                /* PID = BUF */
                usbx_function_set_pid_buf(USB_FUNCTION_PIPE0);

                /*callback*/
                events->ep0_out();
                break;

            case USB_FUNCTION_READOVER:     /* FIFO access error */
                /* Buffer Clear */
                USB20X.CFIFOCTR = USB_FUNCTION_BITBCLR;
                usbx_function_disable_brdy_int(USB_FUNCTION_PIPE0);
                /* Req Error */
                usbx_function_set_pid_stall(USB_FUNCTION_PIPE0);

                /*callback*/
                events->ep0_out();
                break;

            case DEVDRV_USBF_FIFOERROR:     /* FIFO access error */
            default:
                usbx_function_disable_brdy_int(USB_FUNCTION_PIPE0);
                /* Req Error */
                usbx_function_set_pid_stall(USB_FUNCTION_PIPE0);
                break;
        }
        /* Three dummy reads for clearing interrupt requests */
        dumy_sts = USB20X.BRDYSTS;
    } else if (
        (int_sts0 & USB_FUNCTION_BITNRDY) &&
        (int_enb0 & USB_FUNCTION_BITNRDY) &&
        ((int_sts2 & int_enb3) & g_usbx_function_bit_set[USB_FUNCTION_PIPE0])) {
        /* ==== NRDY PIPE0 ==== */
        USB20X.NRDYSTS =
            (uint16_t)~g_usbx_function_bit_set[USB_FUNCTION_PIPE0];

        /* Three dummy reads for clearing interrupt requests */
        dumy_sts = USB20X.NRDYSTS;
    } else if (
        (int_sts0 & USB_FUNCTION_BITCTRT) && (int_enb0 & USB_FUNCTION_BITCTRE)) {
        int_sts0 = USB20X.INTSTS0;
        USB20X.INTSTS0 = (uint16_t)~USB_FUNCTION_BITCTRT;

        if (((int_sts0 & USB_FUNCTION_BITCTSQ) == USB_FUNCTION_CS_RDDS) ||
                ((int_sts0 & USB_FUNCTION_BITCTSQ) == USB_FUNCTION_CS_WRDS) ||
                ((int_sts0 & USB_FUNCTION_BITCTSQ) == USB_FUNCTION_CS_WRND)) {

            /* remake EP0 into buffer */
            usbx_function_save_request();
            if ((USB20X.INTSTS0 & USB_FUNCTION_BITVALID) && (
                        ((int_sts0 & USB_FUNCTION_BITCTSQ) == USB_FUNCTION_CS_RDDS) ||
                        ((int_sts0 & USB_FUNCTION_BITCTSQ) == USB_FUNCTION_CS_WRDS) ||
                        ((int_sts0 & USB_FUNCTION_BITCTSQ) == USB_FUNCTION_CS_WRND))) {
                /* New SETUP token received */
                /* Three dummy reads for cleearing interrupt requests */
                dumy_sts = USB20X.INTSTS0;
                dumy_sts = USB20X.INTSTS0;
                dumy_sts = USB20X.INTSTS0;
                (void)dumy_sts;
                return;
            }
        }

        greentea_send_kv("USBPhyHw::setup =", (int_sts0 & USB_FUNCTION_BITCTSQ));
        switch (int_sts0 & USB_FUNCTION_BITCTSQ) {
            case USB_FUNCTION_CS_IDST:
                if (g_usbx_function_TestModeFlag == DEVDRV_USBF_YES) {
                    /* ==== Test Mode ==== */
                    usbx_function_USB_FUNCTION_TestMode();
                }
                /* Needs not procedure in this state */
                break;

            case USB_FUNCTION_CS_RDDS:
                /* Reads a setup packet */
                events->ep0_setup();
                break;

            case USB_FUNCTION_CS_WRDS:
                /* Original code was the SetDescriptor was called */
                events->ep0_setup();
                break;

            case USB_FUNCTION_CS_WRND:
                events->ep0_setup();

                /*The events->ep0_setup should finish in successful */
                usbx_function_set_pid_buf(USB_FUNCTION_PIPE0);

                RZA_IO_RegWrite_16(&USB20X.DCPCTR, 1, USB_DCPCTR_CCPL_SHIFT, USB_DCPCTR_CCPL);
                break;

            case USB_FUNCTION_CS_RDSS:
                RZA_IO_RegWrite_16(&USB20X.DCPCTR, 1, USB_DCPCTR_CCPL_SHIFT, USB_DCPCTR_CCPL);
                break;

            case USB_FUNCTION_CS_WRSS:
                RZA_IO_RegWrite_16(&USB20X.DCPCTR, 1, USB_DCPCTR_CCPL_SHIFT, USB_DCPCTR_CCPL);
                break;

            case USB_FUNCTION_CS_SQER:
                usbx_function_set_pid_stall(USB_FUNCTION_PIPE0);
                break;

            default:
                usbx_function_set_pid_stall(USB_FUNCTION_PIPE0);
                break;
        }
    } else if (
        (int_sts0 & USB_FUNCTION_BITBEMP) &&
        (int_enb0 & USB_FUNCTION_BITBEMP) &&
        (int_sts3 & int_enb4) ) {
        /* ==== BEMP PIPEx ==== */
        /**************************************************************
         * Function Name: usbx_function_bemp_int
         * Description  : Executes BEMP interrupt(USB_FUNCTION_PIPE1-9).
         * Arguments    : uint16_t int_sts3       ; BEMPSTS Register Value
         *              : uint16_t int_enb4      ; BEMPENB Register Value
         * Return Value : none
         *************************************************************/
        /* copied from usbx_function_intrn.c */
        uint16_t pid;
        uint16_t pipe;
        uint16_t bitcheck;
        uint16_t inbuf;
        uint16_t ep;

        bitcheck = (uint16_t)(int_sts3 & int_enb4);

        USB20X.BEMPSTS = (uint16_t)~int_sts3;

        for (pipe = USB_FUNCTION_PIPE1; pipe <= USB_FUNCTION_MAX_PIPE_NO; pipe++) {
            if ((bitcheck&g_usbx_function_bit_set[pipe]) != g_usbx_function_bit_set[pipe]) {
                continue;
            }

            pid = usbx_function_get_pid(pipe);

            if ((pid == DEVDRV_USBF_PID_STALL) ||
                (pid == DEVDRV_USBF_PID_STALL2)) {
                greentea_send_kv("USBPhyHw::process 1 pid =", pid);
                g_usbx_function_pipe_status[pipe] = DEVDRV_USBF_PIPE_STALL;

            } else {
                inbuf = usbx_function_get_inbuf(pipe);

                if (inbuf == 0) {
                    usbx_function_disable_bemp_int(pipe);
                    usbx_function_set_pid_nak(pipe);
                    g_usbx_function_pipe_status[pipe] = DEVDRV_USBF_PIPE_DONE;

                    switch (g_usbx_function_PipeTbl[pipe] & USB_FUNCTION_FIFO_USE) {
                        case USB_FUNCTION_D0FIFO_DMA:
                            /*now, DMA is not supported*/
                            break;

                        case USB_FUNCTION_D1FIFO_DMA:
                            /*now, DMA is not supported*/
                            break;

                        default:
                            ep = g_usbx_function_PipeTbl[pipe] & 0xff;
                            if (RZA_IO_RegRead_16(
                                    &g_usbx_function_pipecfg[pipe], USB_PIPECFG_DIR_SHIFT, USB_PIPECFG_DIR) == 0) {
                                /* read */
                                __NOP();
                            } else {
                                /* write */
                                EPx_read_status = DEVDRV_USBF_PIPE_WAIT;
                                events->out(ep);
                                EPx_read_status = DEVDRV_USBF_PIPE_DONE;
                            }
                    }
                }
            }
        }

        /* Three dummy reads for clearing interrupt requests */
        dumy_sts = USB20X.BEMPSTS;

    } else if (
        (int_sts0 & USB_FUNCTION_BITBRDY) &&
        (int_enb0 & USB_FUNCTION_BITBRDY) &&
        (int_sts1 & int_enb2) ) {
        /* ==== BRDY PIPEx ==== */
        /**************************************************************
         * Function Name: usbx_function_brdy_int
         * Description  : Executes BRDY interrupt(USB_FUNCTION_PIPE1-9).
         *              : According to the pipe that interrupt is generated in,
         *              : reads/writes buffer allocated in the pipe.
         *              : This function is executed in the BRDY
         *              : interrupt handler.  This function
         *              : clears BRDY interrupt status and BEMP
         *              : interrupt status.
         * Arguments    : uint16_t int_sts1    ; BRDYSTS Register Value
         *              : uint16_t Int_enbl  ; BRDYENB Register Value
         * Return Value : none
         *************************************************************/
        /* copied from usbx_function_intrn.c */
        uint32_t int_sense = 0;
        uint16_t pipe;
        uint16_t pipebit;
        uint16_t ep;

        for (pipe = USB_FUNCTION_PIPE1; pipe <= USB_FUNCTION_MAX_PIPE_NO; pipe++) {
            pipebit = g_usbx_function_bit_set[pipe];

            if ((int_sts1 & pipebit) && (int_enb2 & pipebit)) {
                USB20X.BRDYSTS = (uint16_t)~pipebit;
                USB20X.BEMPSTS = (uint16_t)~pipebit;

                switch (g_usbx_function_PipeTbl[pipe] & USB_FUNCTION_FIFO_USE) {
                    case USB_FUNCTION_D0FIFO_DMA:
                        if (g_usbx_function_DmaStatus[USB_FUNCTION_D0FIFO] != USB_FUNCTION_DMA_READY) {
                            /*now, DMA is not supported*/
                            usbx_function_dma_interrupt_d0fifo(int_sense);
                        }

                        if (RZA_IO_RegRead_16(
                                &g_usbx_function_pipecfg[pipe], USB_PIPECFG_BFRE_SHIFT, USB_PIPECFG_BFRE) == 0) {
                            /*now, DMA is not supported*/
                            usbx_function_read_dma(pipe);
                            usbx_function_disable_brdy_int(pipe);
                        } else {
                            USB20X.D0FIFOCTR = USB_FUNCTION_BITBCLR;
                            g_usbx_function_pipe_status[pipe] = DEVDRV_USBF_PIPE_DONE;
                        }
                        break;

                    case USB_FUNCTION_D1FIFO_DMA:
                        if (g_usbx_function_DmaStatus[USB_FUNCTION_D1FIFO] != USB_FUNCTION_DMA_READY) {
                            /*now, DMA is not supported*/
                            usbx_function_dma_interrupt_d1fifo(int_sense);
                        }

                        if (RZA_IO_RegRead_16(
                                &g_usbx_function_pipecfg[pipe], USB_PIPECFG_BFRE_SHIFT, USB_PIPECFG_BFRE) == 0) {
                            /*now, DMA is not supported*/
                            usbx_function_read_dma(pipe);
                            usbx_function_disable_brdy_int(pipe);
                        } else {
                            USB20X.D1FIFOCTR = USB_FUNCTION_BITBCLR;
                            g_usbx_function_pipe_status[pipe] = DEVDRV_USBF_PIPE_DONE;
                        }
                        break;

                    default:
                        ep = g_usbx_function_PipeTbl[pipe] & 0xff;
                        if (RZA_IO_RegRead_16(
                                &g_usbx_function_pipecfg[pipe], USB_PIPECFG_DIR_SHIFT, USB_PIPECFG_DIR) == 0) {
                            /* read */
                            EPx_read_status = DEVDRV_USBF_PIPE_WAIT;
                            events->in(ep);
                            EPx_read_status = DEVDRV_USBF_PIPE_DONE;
                        } else {
                            /* write */
                            EPx_read_status = DEVDRV_USBF_PIPE_WAIT;
                            events->out(ep);
                            EPx_read_status = DEVDRV_USBF_PIPE_DONE;
                            usbx_function_write_buffer(pipe);
                        }
                }
            }
        }
        /* Three dummy reads for clearing interrupt requests */
        dumy_sts = USB20X.BRDYSTS;

    } else if (
        (int_sts0 & USB_FUNCTION_BITNRDY) &&
        (int_enb0 & USB_FUNCTION_BITNRDY) &&
        (int_sts2 & int_enb3)) {
        /* ==== NRDY PIPEx ==== */
        /**************************************************************
         * Function Name: usbx_function_nrdy_int
         * Description  : Executes NRDY interrupt(USB_FUNCTION_PIPE1-9).
         *              : Checks NRDY interrupt cause by PID. When the cause if STALL,
         *              : regards the pipe state as STALL and ends the processing.
         *              : Then the cause is not STALL, increments the error count to
         *              : communicate again. When the error count is 3, determines
         *              : the pipe state as DEVDRV_USBF_PIPE_NORES and ends the processing.
         *              : This function is executed in the NRDY interrupt handler.
         *              : This function clears NRDY interrupt status.
         * Arguments    : uint16_t int_sts2       ; NRDYSTS Register Value
         *              : uint16_t int_enb3      ; NRDYENB Register Value
         * Return Value : none
         *************************************************************/
        /* copied from usbx_function_intrn.c */
        uint16_t pid;
        uint16_t pipe;
        uint16_t bitcheck;

        bitcheck = (uint16_t)(int_sts2 & int_enb3);

        USB20X.NRDYSTS = (uint16_t)~int_sts2;

        if (RZA_IO_RegRead_16(&USB20X.SYSCFG0, USB_SYSCFG_DCFM_SHIFT, USB_SYSCFG_DCFM) == 1) {
            /* USB HOST */
            /* not support */

        } else {
            /* USB Function */
            for (pipe = USB_FUNCTION_PIPE1; pipe <= USB_FUNCTION_MAX_PIPE_NO; pipe++) {
                if ((bitcheck&g_usbx_function_bit_set[pipe]) != g_usbx_function_bit_set[pipe]) {
                    continue;
                }

                if (g_usbx_function_pipe_status[pipe] != DEVDRV_USBF_PIPE_WAIT) {
                    continue;
                }

                pid = usbx_function_get_pid(pipe);
                if ((pid == DEVDRV_USBF_PID_STALL) || (pid == DEVDRV_USBF_PID_STALL2)) {
                    greentea_send_kv("USBPhyHw::process 2 pid =", pid);
                    g_usbx_function_pipe_status[pipe] = DEVDRV_USBF_PIPE_STALL;
                } else {
                    usbx_function_set_pid_buf(pipe);
                }

                if (RZA_IO_RegRead_16(
                        &g_usbx_function_pipecfg[pipe], USB_PIPECFG_DIR_SHIFT, USB_PIPECFG_DIR) == 0) {
                    /* read */
                    __NOP();
                } else {
                    /* write */
                    __NOP();
                }
            }
        }

        /* Three dummy reads for clearing interrupt requests */
        dumy_sts = USB20X.NRDYSTS;

    } else {
        /* Do Nothing */
    }

    /* Three dummy reads for cleearing interrupt requests */
    dumy_sts = USB20X.INTSTS0;
    dumy_sts = USB20X.INTSTS1;
    (void)dumy_sts;

//    greentea_send_kv("USBPhyHw::enabling0 ", int_sts0);
//    greentea_send_kv("USBPhyHw::enabling1 ", int_sts1);
//    greentea_send_kv("USBPhyHw::e0 ", int_enb0);

    // Re-enable interrupt
    GIC_ClearPendingIRQ(USBIX_IRQn);
    GIC_EnableIRQ(USBIX_IRQn);

}

void USBPhyHw::_usbisr(void) {
    GIC_DisableIRQ(USBIX_IRQn);
    instance->events->start_process();
}
#endif
