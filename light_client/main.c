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




/* ------------ Inclusions ---------------- */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "bsp_thread.h"
#include "app_timer.h"
#include "app_uart.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include <openthread/openthread.h>
#include <openthread/diag.h>
#include <openthread/coap.h>
#include <openthread/cli.h>
#include <openthread/thread_ftd.h>
#include <openthread/platform/platform.h>
#include <openthread/platform/alarm.h>




/* Enable or disable UART channel */
#define UART_CHANNEL_ENABLED




/* ---------------- local constants -----------------  */

#ifdef UART_CHANNEL_ENABLED
/* UART pins defines */
#define DATA_UART_RX_PIN_NUM 				30	
#define DATA_UART_TX_PIN_NUM 				31
#define DATA_UART_RTS_PIN_NUM 			28
#define DATA_UART_CTS_PIN_NUM 			29
/* UART buffers size */
#define UART_TX_BUF_SIZE 					256	/**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 					256	/**< UART RX buffer size. */
#endif




/* ---------------- local typedefs -----------------  */

/* device type */
typedef enum
{
    DEVICE_TYPE_REMOTE_CONTROL,
    DEVICE_TYPE_LIGHT
} device_type_t;

/* light commands */
typedef enum
{
    LIGHT_OFF = 0,
    LIGHT_ON,
    LIGHT_TOGGLE
} light_command_t;

/* application info */
typedef struct
{
	otInstance   * p_ot_instance;       	/**< A pointer to the OpenThread instance. */
	otIp6Address   peer_address;        	/**< An address of a related server node. */
	uint8_t        multicast_dim_value;		/**< Information which multicast dimming value should be sent next. */
} application_t;




/* ---------------- local variables -----------------  */

/* store application info */
static application_t m_app =
{
	.p_ot_instance      = NULL,
	.peer_address       = { .mFields.m8 = { 0 } },
	.multicast_dim_value	= 0,
};

/* IPv6 address */
static const otIp6Address m_unspecified_ipv6 = { .mFields.m8 = { 0 } };

/* Provisioning enable request flag */
static bool provisioning_enable_req = false;

#ifdef UART_CHANNEL_ENABLED
/* flag to set data are received */
static bool data_received = false;

/* UART data buffer */
static uint8_t data_buffer[100];

/* UART buffer depth */
static uint8_t buffer_depth;

/* UART buffer index */
static uint8_t buf_idx = 0;
#endif




/* ----------------------- local functions prototypes --------------------- */

static void light_response_handler			(void *, otCoapHeader *, otMessage *, const otMessageInfo *, otError);
static void dim_response_handler				(void *, otCoapHeader *, otMessage *, const otMessageInfo *, otError);
static void unicast_light_request_send		(otInstance *, uint8_t);
static void unicast_dim_request_send		(otInstance *, uint8_t);
static void multicast_light_request_send	(otInstance *, uint8_t);
static void multicast_dim_request_send		(otInstance *, uint8_t);
static void provisioning_response_handler	(void *, otCoapHeader *, otMessage *, const otMessageInfo *, otError);
static void provisioning_request_send		(otInstance *);
static void coap_default_handler				(void *, otCoapHeader *, otMessage *, const otMessageInfo *);
static void role_change_handler				(void *, otDeviceRole);
static void state_changed_callback			(uint32_t, void *);
static void bsp_event_handler					(bsp_event_t);
static void thread_init							(void);
static void coap_init							(void);
static void timer_init							(void);
static void leds_init							(void);
static void thread_bsp_init					(void);
#ifdef UART_CHANNEL_ENABLED
static void manageUART							(void);
static void uart_error_handle					(app_uart_evt_t *);
#endif




/* ------------------- local functions implementation ------------------ */

/* CoAP light response handler */
static void light_response_handler(void                * p_context,
                                   otCoapHeader        * p_header,
                                   otMessage           * p_message,
                                   const otMessageInfo * p_message_info,
                                   otError               result)
{
    (void)p_context;
    (void)p_header;
    (void)p_message;

    if (result == OT_ERROR_NONE)
    {
        NRF_LOG_INFO("Received light control response.\r\n");
    }
    else
    {
        NRF_LOG_INFO("Failed to receive response: %d\r\n", result);
        m_app.peer_address = m_unspecified_ipv6;
    }
}


/* Dimming response handler function */
static void dim_response_handler(void                * p_context,
                          			otCoapHeader        * p_header,
                          			otMessage           * p_message,
                          			const otMessageInfo * p_message_info,
                          			otError           	 result)
{
    (void)p_context;
    (void)p_header;
    (void)p_message;

    if (result == OT_ERROR_NONE)
    {
        NRF_LOG_INFO("Received dimming control response.\r\n");
    }
    else
    {
        NRF_LOG_INFO("Failed to receive response: %d\r\n", result);
        m_app.peer_address = m_unspecified_ipv6;
    }
}


/* CoAP unicast light request */
static void unicast_light_request_send(otInstance * p_instance, uint8_t command)
{
    otError       error = OT_ERROR_NONE;
    otMessage   * p_message;
    otMessageInfo messageInfo;
    otCoapHeader  header;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_CONFIRMABLE, OT_COAP_CODE_PUT);
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
        if (error != OT_ERROR_NONE)
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

    if (error != OT_ERROR_NONE && p_message != NULL)
    {
        NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
        otMessageFree(p_message);
    }
}


/* Function to send a dimming request to a peer device (unicast) */
static void unicast_dim_request_send(otInstance * p_instance, uint8_t dim_value)
{
    otError       error = OT_ERROR_NONE;
    otMessage   * p_message;
    otMessageInfo messageInfo;
    otCoapHeader  header;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_CONFIRMABLE, OT_COAP_CODE_PUT);
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
        if (error != OT_ERROR_NONE)
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

    if (error != OT_ERROR_NONE && p_message != NULL)
    {
        NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
        otMessageFree(p_message);
    }
}


/* CoAP multicast light request */
static void multicast_light_request_send(otInstance * p_instance, uint8_t command)
{
    otError       error = OT_ERROR_NONE;
    otMessage   * p_message;
    otMessageInfo messageInfo;
    otCoapHeader  header;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_CONFIRMABLE, OT_COAP_CODE_PUT);
        otCoapHeaderAppendUriPathOptions(&header, "light");
        otCoapHeaderSetPayloadMarker(&header);

        p_message = otCoapNewMessage(p_instance, &header);
        if (p_message == NULL)
        {
            NRF_LOG_INFO("Failed to allocate message for CoAP Request\r\n");
            break;
        }

        error = otMessageAppend(p_message, &command, sizeof(command));
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        memset(&messageInfo, 0, sizeof(messageInfo));
        messageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
        messageInfo.mPeerPort = OT_DEFAULT_COAP_PORT;
        otIp6AddressFromString("FF03::1", &messageInfo.mPeerAddr);

        error = otCoapSendRequest(p_instance, p_message, &messageInfo, NULL, NULL);
    } while (false);

    if (error != OT_ERROR_NONE && p_message != NULL)
    {
        NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
        otMessageFree(p_message);
    }
}


/* Function to send a dimming request to any device (multicast) */
static void multicast_dim_request_send(otInstance * p_instance, uint8_t dim_value)
{
    otError       error = OT_ERROR_NONE;
    otMessage   * p_message;
    otMessageInfo messageInfo;
    otCoapHeader  header;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_CONFIRMABLE, OT_COAP_CODE_PUT);
        otCoapHeaderAppendUriPathOptions(&header, "dim");
        otCoapHeaderSetPayloadMarker(&header);

        p_message = otCoapNewMessage(p_instance, &header);
        if (p_message == NULL)
        {
            NRF_LOG_INFO("Failed to allocate message for CoAP Request\r\n");
            break;
        }

        error = otMessageAppend(p_message, &dim_value, sizeof(dim_value));
        if (error != OT_ERROR_NONE)
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

    if (error != OT_ERROR_NONE && p_message != NULL)
    {
        NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
        otMessageFree(p_message);
    }

	NRF_LOG_INFO("Sent dim value: %d\r\n", dim_value);
}


/* CoAP provisioning response handler */
static void provisioning_response_handler(void                * p_context,
                                          otCoapHeader        * p_header,
                                          otMessage           * p_message,
                                          const otMessageInfo * p_message_info,
                                          otError               result)
{
    (void)p_context;
    (void)p_header;

    uint8_t peer_type;

    if (result == OT_ERROR_NONE)
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


/* CoAP send provisioning request */
static void provisioning_request_send(otInstance * p_instance)
{
    otError       error = OT_ERROR_NONE;
    otCoapHeader  header;
    otMessage   * p_request;
    otMessageInfo aMessageInfo;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_GET);
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

    if (error != OT_ERROR_NONE && p_request != NULL)
    {
        otMessageFree(p_request);
    }
}


/* Default CoAP Handler */
static void coap_default_handler(void *p_context, otCoapHeader *p_header, otMessage *p_message,
                                 const otMessageInfo *p_message_info)
{
	(void)p_context;
	(void)p_header;
	(void)p_message;
	(void)p_message_info;

	NRF_LOG_INFO("Received CoAP message that does not match any request or resource\r\n");
}


/* State change handling */
static void role_change_handler(void * p_context, otDeviceRole role)
{
    switch(role)
    {
        case OT_DEVICE_ROLE_CHILD:
        case OT_DEVICE_ROLE_ROUTER:
        case OT_DEVICE_ROLE_LEADER:
            break;

        case OT_DEVICE_ROLE_DISABLED:
        case OT_DEVICE_ROLE_DETACHED:
        default:
            m_app.peer_address = m_unspecified_ipv6;
            break;
    }
}


/* state change callback */
static void state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_CHANGED_THREAD_ROLE)
    {
        role_change_handler(p_context, otThreadGetDeviceRole(p_context));
    }

    if (flags & OT_CHANGED_THREAD_PARTITION_ID)
    {
        m_app.peer_address = m_unspecified_ipv6;
    }

    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otThreadGetDeviceRole(p_context));
}


/* Buttons event handler */
static void bsp_event_handler(bsp_event_t event)
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
            return; // no implementation needed
    }
}


/* Thread Initialization */
static void thread_init(void)
{
    otInstance *p_instance;

    PlatformInit(0, NULL);

    p_instance = otInstanceInit();
    assert(p_instance);

    otCliUartInit(p_instance);

    NRF_LOG_INFO("Thread version: %s\r\n", (uint32_t)otGetVersionString());
    NRF_LOG_INFO("Network name:   %s\r\n", (uint32_t)otThreadGetNetworkName(p_instance));

    assert(otSetStateChangedCallback(p_instance, &state_changed_callback, p_instance) == OT_ERROR_NONE);

    if (!otDatasetIsCommissioned(p_instance))
    {
        assert(otLinkSetChannel(p_instance, THREAD_CHANNEL) == OT_ERROR_NONE);
        assert(otLinkSetPanId(p_instance, THREAD_PANID) == OT_ERROR_NONE);
    }

    assert(otIp6SetEnabled(p_instance, true) == OT_ERROR_NONE);
    assert(otThreadSetEnabled(p_instance, true) == OT_ERROR_NONE);

    m_app.p_ot_instance = p_instance;
}


/* CoAp init */
static void coap_init(void)
{
    assert(otCoapStart(m_app.p_ot_instance, OT_DEFAULT_COAP_PORT) == OT_ERROR_NONE);
    otCoapSetDefaultHandler(m_app.p_ot_instance, coap_default_handler, NULL);
}


/* timer init */
static void timer_init(void)
{
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/* LEDs init */
static void leds_init(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
}


/* Thread BSP init */
static void thread_bsp_init(void)
{
    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_thread_init(m_app.p_ot_instance);
    APP_ERROR_CHECK(err_code);
}

#ifdef UART_CHANNEL_ENABLED
/* function to manage UART data */
static void manageUART( void )
{
	/* if data are received */
	if(true == data_received)
	{
		/* clear data received flag */
		data_received = false;

		/* ATTENTION: brutal compare of JSON string... temporary code... */
		if(0 == memcmp(data_buffer, "{\"command\":[{\"light\":\"on\"}]}", buffer_depth))
		{
			/* send a multi light request to turn lights on */
			multicast_light_request_send(m_app.p_ot_instance, LIGHT_ON);	
		}
		else if(0 == memcmp(data_buffer, "{\"command\":[{\"light\":\"off\"}]}", buffer_depth))
		{
			/* send a multi light request to turn lights off */
			multicast_light_request_send(m_app.p_ot_instance, LIGHT_OFF);	
		}
		else
		{
			/* do nothing */
		}

		/* signal on the board that a message has been received and managed */
		LEDS_INVERT(BSP_LED_1_MASK);

		/* clear buffer depth */
		buffer_depth = 0;
	}
}


/* UART error handler function */
static void uart_error_handle(app_uart_evt_t * p_event)
{
	if(p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
	{
		/* flush the FIFO buffers */
		app_uart_flush();
		APP_ERROR_HANDLER(p_event->data.error_communication);
	}
	else if(p_event->evt_type == APP_UART_FIFO_ERROR)
	{
		/* flush the FIFO buffers */
		app_uart_flush();
		APP_ERROR_HANDLER(p_event->data.error_code);
	}
	else if(p_event->evt_type == APP_UART_DATA_READY)
	{
		/* store received byte in the buffer */
		app_uart_get(&data_buffer[buf_idx]);
		/* if termination character has been received */
		if(data_buffer[buf_idx] == '.')
		{
			/* store buffer depth */
			buffer_depth = buf_idx;
			/* clear buffer index */
			buf_idx = 0;
			/* set flag */
			data_received = true;
		}
		else
		{
			/* increment buffer index */
			buf_idx++;
		}
	}
}
#endif




/* ----------------- Main loop --------------------- */
int main(int argc, char *argv[])
{
	NRF_LOG_INIT(NULL);
#ifdef UART_CHANNEL_ENABLED
	uint32_t err_code;
	
	const app_uart_comm_params_t comm_params =
   {
       DATA_UART_RX_PIN_NUM, 
       DATA_UART_TX_PIN_NUM, 
       DATA_UART_RTS_PIN_NUM, 
       DATA_UART_CTS_PIN_NUM,
       APP_UART_FLOW_CONTROL_DISABLED,
       false,
       UART_BAUDRATE_BAUDRATE_Baud115200
   };

	/* init UART 1 module */
	APP_UART_FIFO_INIT(&comm_params,
		        			 UART_RX_BUF_SIZE,
		                UART_TX_BUF_SIZE,
		                uart_error_handle,
		                APP_IRQ_PRIORITY_LOWEST,
		                err_code);

	APP_ERROR_CHECK(err_code);
#endif
	thread_init();
	coap_init();

	timer_init();
	thread_bsp_init();
	leds_init();

	/* infinite loop */
	while (true)
	{
		otTaskletsProcess(m_app.p_ot_instance);
		PlatformProcessDrivers(m_app.p_ot_instance);
#ifdef UART_CHANNEL_ENABLED
		/* call function to manage UART data */
		manageUART();
#endif
	}
}




/* End of file */




