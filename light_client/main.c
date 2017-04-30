/*
 * The MIT License (MIT)
 *
 * Copyright (c) [2017] [Marco Russi]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/




/* ------------------- Inclusions ------------------- */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "bsp_thread.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include <openthread/openthread.h>
#include <openthread/diag.h>
#include <openthread/coap.h>
#include <openthread/cli.h>
#include <openthread/platform/platform.h>
#include <openthread/platform/alarm.h>




/* ------------------- typedef enums ------------------- */

/* Device type enum */
typedef enum
{
    DEVICE_TYPE_REMOTE_CONTROL,
    DEVICE_TYPE_LIGHT
} device_type_t;

/* Light command enum */
typedef enum
{
    LIGHT_OFF = 0,
    LIGHT_ON,
    LIGHT_TOGGLE
} light_command_t;




/* ------------------- typedef struct ------------------- */

/* Structure to store application data */
typedef struct
{
    otInstance   * p_ot_instance;       	/**< A pointer to the OpenThread instance. */
    otIp6Address   peer_address;        	/**< An address of a related server node. */
	uint8_t        multicast_dim_value; 	/**< Information which multicast dimming value should be sent next. */
} application_t;




/* ------------------- local variables ------------------- */

/* Store application data */
application_t m_app =
{
    .p_ot_instance      	= NULL,
    .peer_address       	= { .mFields.m8 = { 0 } },
	.multicast_dim_value	= 0,
};

/* Provisioning enable request flag */
static bool provisioning_enable_req = false;




/* ------------------- local const variables ------------------- */

/* Store const unspecified IPv6 address */
static const otIp6Address m_unspecified_ipv6 = { .mFields.m8 = { 0 } };




/* ------------------- local functions implementation ------------------- */

void otTaskletsSignalPending(otInstance *aInstance)
{
    (void)aInstance;
}


/* Dimming response handler function */
void dim_response_handler(void                * p_context,
                          otCoapHeader        * p_header,
                          otMessage           * p_message,
                          const otMessageInfo * p_message_info,
                          ThreadError           result)
{
    (void)p_context;
    (void)p_header;
    (void)p_message;

    if (result == kThreadError_None)
    {
        NRF_LOG_INFO("Received dimming control response.\r\n");
    }
    else
    {
        NRF_LOG_INFO("Failed to receive response: %d\r\n", result);
        m_app.peer_address = m_unspecified_ipv6;
    }
}


/* Light response handler function */
void light_response_handler(void                * p_context,
                            otCoapHeader        * p_header,
                            otMessage           * p_message,
                            const otMessageInfo * p_message_info,
                            ThreadError           result)
{
    (void)p_context;
    (void)p_header;
    (void)p_message;

    if (result == kThreadError_None)
    {
        NRF_LOG_INFO("Received light control response.\r\n");
    }
    else
    {
        NRF_LOG_INFO("Failed to receive response: %d\r\n", result);
        m_app.peer_address = m_unspecified_ipv6;
    }
}


/* Function to send a dimming request to a peer device (unicast) */
void unicast_dim_request_send(otInstance * p_instance, uint8_t dim_value)
{
    ThreadError   error = kThreadError_None;
    otMessage   * p_message;
    otMessageInfo messageInfo;
    otCoapHeader  header;

    do
    {
        otCoapHeaderInit(&header, kCoapTypeConfirmable, kCoapRequestPut);
        otCoapHeaderGenerateToken(&header, 2);
        otCoapHeaderAppendUriPathOptions(&header, "dim");
        otCoapHeaderSetPayloadMarker(&header);

        p_message = otCoapNewMessage(p_instance, &header);
        if (p_message == NULL)
        {
            NRF_LOG_INFO("Failed to allocate message for CoAP Request\r\n");
            break;
        }

        error = otMessageAppend(p_message, &dim_value, sizeof(dim_value));
        if (error != kThreadError_None)
        {
            break;
        }

        memset(&messageInfo, 0, sizeof(messageInfo));
        messageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
        messageInfo.mPeerPort = OT_DEFAULT_COAP_PORT;
        memcpy(&messageInfo.mPeerAddr, &m_app.peer_address, sizeof(messageInfo.mPeerAddr));

        error = otCoapSendRequest(p_instance,
                                  p_message,
                                  &messageInfo,
                                  &dim_response_handler,
                                  p_instance);
    } while (false);

    if (error != kThreadError_None && p_message != NULL)
    {
        NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
        otMessageFree(p_message);
    }
}


/* Function to send a light request to a peer device (unicast) */
void unicast_light_request_send(otInstance * p_instance, uint8_t command)
{
    ThreadError   error = kThreadError_None;
    otMessage   * p_message;
    otMessageInfo messageInfo;
    otCoapHeader  header;

    do
    {
        otCoapHeaderInit(&header, kCoapTypeConfirmable, kCoapRequestPut);
        otCoapHeaderGenerateToken(&header, 2);
        otCoapHeaderAppendUriPathOptions(&header, "light");
        otCoapHeaderSetPayloadMarker(&header);

        p_message = otCoapNewMessage(p_instance, &header);
        if (p_message == NULL)
        {
            NRF_LOG_INFO("Failed to allocate message for CoAP Request\r\n");
            break;
        }

        error = otMessageAppend(p_message, &command, sizeof(command));
        if (error != kThreadError_None)
        {
            break;
        }

        memset(&messageInfo, 0, sizeof(messageInfo));
        messageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
        messageInfo.mPeerPort = OT_DEFAULT_COAP_PORT;
        memcpy(&messageInfo.mPeerAddr, &m_app.peer_address, sizeof(messageInfo.mPeerAddr));

        error = otCoapSendRequest(p_instance,
                                  p_message,
                                  &messageInfo,
                                  &light_response_handler,
                                  p_instance);
    } while (false);

    if (error != kThreadError_None && p_message != NULL)
    {
        NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
        otMessageFree(p_message);
    }
}


/* Function to send a dimming request to any device (multicast) */
void multicast_dim_request_send(otInstance * p_instance, uint8_t dim_value)
{
    ThreadError   error = kThreadError_None;
    otMessage   * p_message;
    otMessageInfo messageInfo;
    otCoapHeader  header;

    do
    {
        otCoapHeaderInit(&header, kCoapTypeNonConfirmable, kCoapRequestPut);
        otCoapHeaderAppendUriPathOptions(&header, "dim");
        otCoapHeaderSetPayloadMarker(&header);

        p_message = otCoapNewMessage(p_instance, &header);
        if (p_message == NULL)
        {
            NRF_LOG_INFO("Failed to allocate message for CoAP Request\r\n");
            break;
        }

        error = otMessageAppend(p_message, &dim_value, sizeof(dim_value));
        if (error != kThreadError_None)
        {
            break;
        }

        memset(&messageInfo, 0, sizeof(messageInfo));
        messageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
        messageInfo.mPeerPort = OT_DEFAULT_COAP_PORT;
        otIp6AddressFromString("FF03::1", &messageInfo.mPeerAddr);

        error = otCoapSendRequest(p_instance, 
								  p_message, 
								  &messageInfo, 
								  NULL, 
								  NULL);
    } while (false);

    if (error != kThreadError_None && p_message != NULL)
    {
        NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
        otMessageFree(p_message);
    }

	NRF_LOG_INFO("Sent dim value: %d\r\n", dim_value);
}


/* Function to send a light request to any device (multicast) */
void multicast_light_request_send(otInstance * p_instance, uint8_t command)
{
    ThreadError   error = kThreadError_None;
    otMessage   * p_message;
    otMessageInfo messageInfo;
    otCoapHeader  header;

    do
    {
        otCoapHeaderInit(&header, kCoapTypeNonConfirmable, kCoapRequestPut);
        otCoapHeaderAppendUriPathOptions(&header, "light");
        otCoapHeaderSetPayloadMarker(&header);

        p_message = otCoapNewMessage(p_instance, &header);
        if (p_message == NULL)
        {
            NRF_LOG_INFO("Failed to allocate message for CoAP Request\r\n");
            break;
        }

        error = otMessageAppend(p_message, &command, sizeof(command));
        if (error != kThreadError_None)
        {
            break;
        }

        memset(&messageInfo, 0, sizeof(messageInfo));
        messageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
        messageInfo.mPeerPort = OT_DEFAULT_COAP_PORT;
        otIp6AddressFromString("FF03::1", &messageInfo.mPeerAddr);

        error = otCoapSendRequest(p_instance, p_message, &messageInfo, NULL, NULL);
    } while (false);

    if (error != kThreadError_None && p_message != NULL)
    {
        NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
        otMessageFree(p_message);
    }
}


/* Provisioning response handler function */
void provisioning_response_handler(void                * p_context,
                                   otCoapHeader        * p_header,
                                   otMessage           * p_message,
                                   const otMessageInfo * p_message_info,
                                   ThreadError           result)
{
    (void)p_context;
    (void)p_header;

    uint8_t peer_type;

    if (result == kThreadError_None)
    {
        if ((otMessageRead(p_message, otMessageGetOffset(p_message), &peer_type, 1) == 1) &&
            (peer_type == DEVICE_TYPE_LIGHT))
        {
            otMessageRead(p_message,
                          otMessageGetOffset(p_message) + 1,
                          &m_app.peer_address,
                          sizeof(m_app.peer_address));
        }
    }
    else
    {
        NRF_LOG_INFO("Provisioning failed: %d\r\n", result);
    }
}


/* Function to send a provisioning request */
void provisioning_request_send(otInstance * p_instance)
{
    ThreadError   error = kThreadError_None;
    otCoapHeader  header;
    otMessage   * p_request;
    otMessageInfo aMessageInfo;

    do
    {
        otCoapHeaderInit(&header, kCoapTypeNonConfirmable, kCoapRequestGet);
        otCoapHeaderGenerateToken(&header, 2);
        otCoapHeaderAppendUriPathOptions(&header, "provisioning");

        p_request = otCoapNewMessage(p_instance, &header);
        if (p_request == NULL)
        {
            break;
        }

        memset(&aMessageInfo, 0, sizeof(aMessageInfo));
        aMessageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
        aMessageInfo.mPeerPort = OT_DEFAULT_COAP_PORT;
        otIp6AddressFromString("FF03::1", &aMessageInfo.mPeerAddr);

        error = otCoapSendRequest(p_instance,
                				  p_request,
                                  &aMessageInfo,
                                  provisioning_response_handler,
                                  p_instance);
    } while (false);

    if (error != kThreadError_None && p_request != NULL)
    {
        otMessageFree(p_request);
    }
}


/* Function to manage a role change */
static void handle_role_change(void * p_context, otDeviceRole role)
{
    switch(role)
    {
        case kDeviceRoleChild:
        case kDeviceRoleRouter:
        case kDeviceRoleLeader:
            break;

        case kDeviceRoleOffline:
        case kDeviceRoleDisabled:
        case kDeviceRoleDetached:
        default:
            m_app.peer_address = m_unspecified_ipv6;
            break;
    }
}


/* State changed callback function */
void state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_NET_ROLE)
    {
        handle_role_change(p_context, otThreadGetDeviceRole(p_context));
    }

    if (flags & OT_NET_PARTITION_ID)
    {
        m_app.peer_address = m_unspecified_ipv6;
    }

    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otThreadGetDeviceRole(p_context));
}


/* BSP event handler function (buttons) */
void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
		case BSP_EVENT_KEY_0:
			/* switch provisioning request flag value */
			if(true == provisioning_enable_req)
			{
				provisioning_enable_req = false;
				/* remove peer address */
				m_app.peer_address = m_unspecified_ipv6;
			}
			else
			{
				provisioning_enable_req = true;
				/* send provisioning request (always multicast) */
            	provisioning_request_send(m_app.p_ot_instance);
			}
            break;

        case BSP_EVENT_KEY_1:
			/* if peer address is valid */
            if (!otIp6IsAddressEqual(&m_app.peer_address, &m_unspecified_ipv6))
            {
				/* send unicast light toggle request */
                unicast_light_request_send(m_app.p_ot_instance, LIGHT_TOGGLE);
            }
			else
			{
				/* send multicast light toggle request */
		       	multicast_light_request_send(m_app.p_ot_instance, LIGHT_TOGGLE);
			}
            break;

        case BSP_EVENT_KEY_2:
			/* decrement dimming value */
			if(m_app.multicast_dim_value > 0)
			{
				m_app.multicast_dim_value -= 10;
			}
            /* if peer address is valid */
            if (!otIp6IsAddressEqual(&m_app.peer_address, &m_unspecified_ipv6))
            {
				/* send unicast dimming value */
                unicast_dim_request_send(m_app.p_ot_instance, m_app.multicast_dim_value);
            }
			else
			{
				/* send multicast dimming value */
		       	multicast_dim_request_send(m_app.p_ot_instance, m_app.multicast_dim_value);
			}
            break;

        case BSP_EVENT_KEY_3:
			/* increment dimming value */
			if(m_app.multicast_dim_value < 100)
			{
				m_app.multicast_dim_value += 10;
			}
			/* if peer address is valid */
			if (!otIp6IsAddressEqual(&m_app.peer_address, &m_unspecified_ipv6))
            {
				/* send unicast dimming value */
                unicast_dim_request_send(m_app.p_ot_instance, m_app.multicast_dim_value);
            }
			else
			{
				/* send multicast dimming value */
		       	multicast_dim_request_send(m_app.p_ot_instance, m_app.multicast_dim_value);
			}
            break;

        default:
            return; /* no implementation needed */
    }
}


/* Function to initialise Thread networking */
static otInstance * initialize_thread(void)
{
    otInstance *p_instance;

    p_instance = otInstanceInit();
    assert(p_instance);

    otCliUartInit(p_instance);

    NRF_LOG_INFO("Thread version: %s\r\n", (uint32_t)otGetVersionString());
    NRF_LOG_INFO("Network name:   %s\r\n", (uint32_t)otThreadGetNetworkName(p_instance));

    assert(otSetStateChangedCallback(p_instance, &state_changed_callback, p_instance) == kThreadError_None);

    assert(otLinkSetChannel(p_instance, THREAD_CHANNEL) == kThreadError_None);
    assert(otLinkSetPanId(p_instance, THREAD_PANID) == kThreadError_None);
    assert(otIp6SetEnabled(p_instance, true) == kThreadError_None);
    assert(otThreadSetEnabled(p_instance, true) == kThreadError_None);

    return p_instance;
}


/* Function to init timer */
void initialize_timer(void)
{
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/* Function to init LEDs */
void initialize_leds(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
}


/* Function to init BSP */
void initialize_bsp(void)
{
    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_thread_init(m_app.p_ot_instance);
    APP_ERROR_CHECK(err_code);
}


/* main function */
int main(int argc, char *argv[])
{
    NRF_LOG_INIT(NULL);

    PlatformInit(argc, argv);
    m_app.p_ot_instance = initialize_thread();

    initialize_timer();
    initialize_bsp();
    initialize_leds();

    while (true)
    {
        otTaskletsProcess(m_app.p_ot_instance);
        PlatformProcessDrivers(m_app.p_ot_instance);
    }

    return 0;
}




/* End of file */


