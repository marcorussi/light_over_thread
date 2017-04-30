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
#include "app_pwm.h"

#include <openthread/openthread.h>
#include <openthread/diag.h>
#include <openthread/coap.h>
#include <openthread/cli.h>
#include <openthread/platform/platform.h>
#include <openthread/platform/alarm.h>




/* ------------------- local defines ------------------- */

#define PROVISIONING_LED			BSP_LED_1_MASK

#define LED_INTERVAL             	100

#define PWM_CH_PIN_NUM				16

#define PROVISIONING_EXPIRY_TIME 	5000




/* ------------------- local enums ------------------- */

typedef enum
{
    DEVICE_TYPE_REMOTE_CONTROL,	/* not used */
    DEVICE_TYPE_LIGHT
} device_type_t;

typedef enum
{
    LIGHT_OFF = 0,		/* not used */
    LIGHT_ON,			/* not used */
    LIGHT_TOGGLE
} light_command_t;




/* ------------------- local structs ------------------- */

typedef struct
{
    otInstance     * p_ot_instance;         /**< A pointer to the OpenThread instance. */
    bool             enable_provisioning;   /**< Information if provisioning is enabled. */
    uint32_t         provisioning_expiry;   /**< Provisioning timeout time. */
    otCoapResource   provisioning_resource; /**< CoAP provisioning resource. */
    otCoapResource   light_resource;        /**< CoAP light resource. */
	otCoapResource   dim_resource;        	/**< CoAP light dimming resource. */
} application_t;




/* ------------------- local macros ------------------- */

/* Timers instances */
APP_TIMER_DEF(m_provisioning_timer);
APP_TIMER_DEF(m_led_timer);

/* Create the instance "PWM1" using TIMER1. */
APP_PWM_INSTANCE(PWM1,1);	




/* ------------------- local functions prototypes ------------------- */		

static void dim_request_handler(void                * p_context,
                                otCoapHeader        * p_header,
                                otMessage           * p_message,
                                const otMessageInfo * p_message_info);

static void light_request_handler(void                * p_context,
                                  otCoapHeader        * p_header,
                                  otMessage           * p_message,
                                  const otMessageInfo * p_message_info);

static void provisioning_request_handler(void                * p_context,
                                         otCoapHeader        * p_header,
                                         otMessage           * p_message,
                                         const otMessageInfo * p_message_info);




/* ------------------- local variables ------------------- */		

/* A flag indicating PWM status. */
static volatile bool ready_flag = true;           

/* Store last received and applied light dimming value */
static uint8_t last_dim_value = true;       

/* Store last received and applied light state */
static uint8_t last_light_state = false; 

/* Store application data */
application_t m_app =
{
    .p_ot_instance         = NULL,
    .enable_provisioning   = false,
    .provisioning_expiry   = 0,
    .provisioning_resource = {"provisioning", provisioning_request_handler, NULL, NULL},
    .light_resource        = {"light", light_request_handler, NULL, NULL},
	.dim_resource          = {"dim", dim_request_handler, NULL, NULL},
};    




/* ------------------- local functions implementation ------------------- */	

void otTaskletsSignalPending(otInstance *aInstance)
{
    (void)aInstance;
}


/* PWM callback function */
void pwm_ready_callback(uint32_t pwm_id)    
{
    ready_flag = true;
}


/* Disable provisioning service */
static void provisioning_disable(otInstance * p_instance)
{
    m_app.enable_provisioning = false;
    m_app.provisioning_expiry = 0;
    otCoapServerRemoveResource(p_instance, &m_app.provisioning_resource);
    app_timer_stop(m_provisioning_timer);
}


/* Enable provisioning service */
static void provisioning_enable(otInstance * p_instance)
{
    m_app.enable_provisioning = true;
    m_app.provisioning_expiry = otPlatAlarmGetNow() + PROVISIONING_EXPIRY_TIME;
    otCoapServerAddResource(p_instance, &m_app.provisioning_resource);
    app_timer_start(m_provisioning_timer,
                    APP_TIMER_TICKS(PROVISIONING_EXPIRY_TIME),
                    p_instance);
}


/* Send dimming response message */
static void dim_response_send(void                * p_context,
                              otCoapHeader        * p_request_header,
                              const otMessageInfo * p_message_info)
{
    ThreadError  error = kThreadError_None;
    otCoapHeader header;
    otMessage  * p_response;

    do
    {
        otCoapHeaderInit(&header, kCoapTypeAcknowledgment, kCoapResponseChanged);
        otCoapHeaderSetMessageId(&header, otCoapHeaderGetMessageId(p_request_header));
        otCoapHeaderSetToken(&header,
                             otCoapHeaderGetToken(p_request_header),
                             otCoapHeaderGetTokenLength(p_request_header));

        p_response = otCoapNewMessage(p_context, &header);
        if (p_response == NULL)
        {
            break;
        }


        error = otCoapSendResponse(p_context, p_response, p_message_info);

    } while (false);

    if (error != kThreadError_None && p_response != NULL)
    {
        otMessageFree(p_response);
    }
}


/* Send light response message */
static void light_response_send(void                * p_context,
                                otCoapHeader        * p_request_header,
                                const otMessageInfo * p_message_info)
{
    ThreadError  error = kThreadError_None;
    otCoapHeader header;
    otMessage  * p_response;

    do
    {
        otCoapHeaderInit(&header, kCoapTypeAcknowledgment, kCoapResponseChanged);
        otCoapHeaderSetMessageId(&header, otCoapHeaderGetMessageId(p_request_header));
        otCoapHeaderSetToken(&header,
                             otCoapHeaderGetToken(p_request_header),
                             otCoapHeaderGetTokenLength(p_request_header));

        p_response = otCoapNewMessage(p_context, &header);
        if (p_response == NULL)
        {
            break;
        }

        error = otCoapSendResponse(p_context, p_response, p_message_info);

    } while (false);

    if (error != kThreadError_None && p_response != NULL)
    {
        otMessageFree(p_response);
    }
}


/* Dimming request handler function */
static void dim_request_handler(void                * p_context,
                                otCoapHeader        * p_header,
                                otMessage           * p_message,
                                const otMessageInfo * p_message_info)
{
    (void)p_message;
    uint8_t dim_value;

    do
    {
        if (otCoapHeaderGetType(p_header) != kCoapTypeConfirmable &&
            otCoapHeaderGetType(p_header) != kCoapTypeNonConfirmable)
        {
            break;
        }

        if (otCoapHeaderGetCode(p_header) != kCoapRequestPut)
        {
            break;
        }

        if (otMessageRead(p_message, otMessageGetOffset(p_message), &dim_value, 1) != 1)
        {
            NRF_LOG_INFO("dim handler - missing command\r\n");
        }

		/* store dimming value */
		last_dim_value = dim_value;

		/* set light state to true */
		last_light_state = true;

		/* set PWM value */
		while (false == ready_flag);
		ready_flag = false;
        APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 0, last_dim_value));

        if (last_dim_value <= 100)
        {
            NRF_LOG_INFO("dim value: %d\r\n", last_dim_value);
        }
		else
		{
			NRF_LOG_INFO("Invalid dim value\r\n");
		}

        if (otCoapHeaderGetType(p_header) == kCoapTypeConfirmable)
        {
            dim_response_send(p_context, p_header, p_message_info);
        }

    } while (false);
}


/* Manage light request */
static void light_request_handler(void                * p_context,
                                  otCoapHeader        * p_header,
                                  otMessage           * p_message,
                                  const otMessageInfo * p_message_info)
{
    (void)p_message;
    uint8_t command;

    do
    {
        if (otCoapHeaderGetType(p_header) != kCoapTypeConfirmable &&
            otCoapHeaderGetType(p_header) != kCoapTypeNonConfirmable)
        {
            break;
        }

        if (otCoapHeaderGetCode(p_header) != kCoapRequestPut)
        {
            break;
        }

        if (otMessageRead(p_message, otMessageGetOffset(p_message), &command, 1) != 1)
        {
            NRF_LOG_INFO("light handler - missing command\r\n");
        }

        switch (command)
        {
            case LIGHT_TOGGLE:
				/* switch light state */
				if(true == last_light_state)
				{
					last_light_state = false;

					/* set PWM value to 0 */
					while (false == ready_flag);
					ready_flag = false;
					APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 0, 0));
				}
				else
				{
					last_light_state = true;

					/* set PWM value to the last received one */
					while (false == ready_flag);
					ready_flag = false;
					APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 0, last_dim_value));
				}
				
				NRF_LOG_INFO("light handler - command TOGGLE\r\n");
                break;

            default:
				/* not supported command: do nothing */
                break;
        }

        if (otCoapHeaderGetType(p_header) == kCoapTypeConfirmable)
        {
            light_response_send(p_context, p_header, p_message_info);
        }

    } while (false);
}


/* Send provisioning response message */
static ThreadError provisioning_response_send(void                * p_context,
                                              otCoapHeader        * p_request_header,
                                              uint8_t               device_type,
                                              const otMessageInfo * p_message_info)
{
    ThreadError  error = kThreadError_NoBufs;
    otCoapHeader header;
    otMessage  * p_response;

    do
    {
        otCoapHeaderInit(&header, kCoapTypeNonConfirmable, kCoapResponseContent);
        otCoapHeaderSetToken(&header,
                             otCoapHeaderGetToken(p_request_header),
                             otCoapHeaderGetTokenLength(p_request_header));
        otCoapHeaderSetPayloadMarker(&header);

        p_response = otCoapNewMessage(p_context, &header);
        if (p_response == NULL)
        {
            break;
        }

        error = otMessageAppend(p_response, &device_type, 1);
        if (error != kThreadError_None)
        {
            break;
        }

        error = otMessageAppend(p_response, otThreadGetMeshLocalEid(p_context), sizeof(otIp6Address));
        if (error != kThreadError_None)
        {
            break;
        }

        error = otCoapSendResponse(p_context, p_response, p_message_info);

    } while (false);

    if (error != kThreadError_None && p_response != NULL)
    {
        otMessageFree(p_response);
    }

    return error;
}


/* Provisioning request handler function */
static void provisioning_request_handler(void                * p_context,
                                         otCoapHeader        * p_header,
                                         otMessage           * p_message,
                                         const otMessageInfo * p_message_info)
{
    (void)p_message;
    otMessageInfo message_info;

    if (otCoapHeaderGetType(p_header) == kCoapTypeNonConfirmable &&
        otCoapHeaderGetCode(p_header) == kCoapRequestGet)
    {
        message_info = *p_message_info;
        memset(&message_info.mSockAddr, 0, sizeof(message_info.mSockAddr));
        if (provisioning_response_send(p_context, p_header, DEVICE_TYPE_LIGHT, &message_info) ==
                kThreadError_None)
        {
            provisioning_disable(p_context);
        }
    }
}


/* Thread role changed handler function */
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
            provisioning_disable(p_context);
            break;
    }
}


/* Thread state changed callback function */
static void state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_NET_ROLE)
    {
        handle_role_change(p_context, otThreadGetDeviceRole(p_context));
    }

    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otThreadGetDeviceRole(p_context));
}


/* BSP buttons event handler function */
static void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
			provisioning_enable(m_app.p_ot_instance);
            break;

        case BSP_EVENT_KEY_1:
            break;

        case BSP_EVENT_KEY_2:
            break;

        case BSP_EVENT_KEY_3:
            break;

        default:
            return;
    }
}


/* Provisioning time-out handler function */
static void provisioning_timer_handler(void * p_context)
{
    provisioning_disable(p_context);
}


/* Provisioning LED timer handler function */
static void led_timer_handler(void * p_context)
{
    (void)p_context;

    if (m_app.enable_provisioning)
    {
        LEDS_INVERT(PROVISIONING_LED);
    }
    else
    {
        LEDS_OFF(PROVISIONING_LED);
    }
}


/* Initialise Thread networking */
static otInstance * initialize_thread(void)
{
    otInstance * p_instance;

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


/* Initialise CoAp service */
static void initialize_coap(otInstance * p_instance)
{
	m_app.dim_resource.mContext = p_instance;
    m_app.light_resource.mContext = p_instance;
    m_app.provisioning_resource.mContext = p_instance;

    assert(otCoapServerStart(p_instance) == kThreadError_None);
    assert(otCoapServerAddResource(p_instance, &m_app.light_resource) == kThreadError_None);
	assert(otCoapServerAddResource(p_instance, &m_app.dim_resource) == kThreadError_None);
}


/* Initialise timers */
static void initialize_timer(void)
{
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    app_timer_create(&m_provisioning_timer, APP_TIMER_MODE_SINGLE_SHOT, provisioning_timer_handler);
    app_timer_create(&m_led_timer, APP_TIMER_MODE_REPEATED, led_timer_handler);
}


/* Initialise BSP */
static void initialize_bsp(void)
{
    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_thread_init(m_app.p_ot_instance);
    APP_ERROR_CHECK(err_code);
}


/* Initialise LEDs */
static void initialize_leds(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);

    app_timer_start(m_led_timer, APP_TIMER_TICKS(LED_INTERVAL), NULL);
}


/* main loop function */
int main(int argc, char *argv[])
{
	ret_code_t err_code;

    NRF_LOG_INIT(NULL);

    PlatformInit(argc, argv);
    m_app.p_ot_instance = initialize_thread();
    initialize_coap(m_app.p_ot_instance);

    initialize_timer();
    initialize_bsp();
    initialize_leds();

	
	/* 1-channel PWM, 2000Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(500L, PWM_CH_PIN_NUM);

    /* Switch the polarity of the channel. */
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_LOW;

    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg, pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);

	/* set initial light state */
	last_light_state = false; 
	/* set initial PWM value */     
	while (false == ready_flag);
	ready_flag = false;
    APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 0, 0));


    while (true)
    {
        otTaskletsProcess(m_app.p_ot_instance);
        PlatformProcessDrivers(m_app.p_ot_instance);
    }

    return 0;
}




/* End of file */


