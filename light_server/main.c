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




/* ------------------- Inclusions --------------------- */

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
#include <openthread/thread_ftd.h>
#include <openthread/platform/alarm.h>
#include <openthread/platform/platform.h>




/* ------------------- Local constants --------------------- */

/* Provisioning LED mask */
#define PROVISIONING_LED					BSP_LED_1_MASK

/* LED interval period in ms */
#define LED_INTERVAL          			100

/* PWM channel pin number */
#define PWM_CH_PIN_NUM						16

/* Provisioning expiry time in ms */
#define PROVISIONING_EXPIRY_TIME 		5000




/* ------------------- Local typedefs --------------------- */

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

/* application info structure */
typedef struct
{
	otInstance     * p_ot_instance;         	/**< A pointer to the OpenThread instance. */
	bool             enable_provisioning;   	/**< Information if provisioning is enabled. */
	uint32_t         provisioning_expiry;   	/**< Provisioning timeout time. */
	otCoapResource   provisioning_resource;	/**< CoAP provisioning resource. */
	otCoapResource   light_resource;        	/**< CoAP light resource. */
	otCoapResource   dim_resource;        		/**< CoAP light dimming resource. */
} application_t;




/* ------------------- local macros --------------------- */

/* timers */
APP_TIMER_DEF(m_provisioning_timer);
APP_TIMER_DEF(m_led_timer);


/* Create the instance "PWM1" using TIMER1. */
APP_PWM_INSTANCE(PWM1,1);




/* ------------------- local variables part 1 --------------------- */

/* A flag indicating PWM status. */
static volatile bool ready_flag = true;           

/* Store last received and applied light dimming value */
static uint8_t last_dim_value = 0;       

/* Store last received and applied light state */
static uint8_t last_light_state = false; 




/* ------------------- local functions prototypes --------------------- */

static void 	pwm_ready_callback					(uint32_t);
static void 	light_on									(void);
static void 	light_off								(void);
static void 	light_toggle							(void);
static void 	provisioning_disable					(otInstance *);
static void 	provisioning_enable					(otInstance *);
static void 	light_response_send					(void *, otCoapHeader *, const otMessageInfo *);
static void 	dim_response_send						(void *, otCoapHeader *, const otMessageInfo *);
static void 	light_request_handler				(void *, otCoapHeader *, otMessage *, const otMessageInfo *);
static void 	dim_request_handler					(void *, otCoapHeader *, otMessage *, const otMessageInfo *);
static otError	provisioning_response_send			(void *, otCoapHeader *, uint8_t, const otMessageInfo *);
static void 	provisioning_request_handler		(void *, otCoapHeader *, otMessage *, const otMessageInfo *);
static void 	role_change_handler					(void *, otDeviceRole);
static void 	state_changed_callback				(uint32_t, void *);
static void 	bsp_event_handler						(bsp_event_t);
static void 	provisioning_timer_handler			(void *);
static void 	led_timer_handler						(void *);
static void 	thread_init								(void);
static void 	coap_init								(void);
static void 	timer_init								(void);
static void 	thread_bsp_init						(void);
static void 	leds_init								(void);




/* ------------------- local variables part 2 --------------------- */

/* Store application info */
static application_t m_app =
{
	.p_ot_instance         = NULL,
	.enable_provisioning   = false,
	.provisioning_expiry   = 0,
	.provisioning_resource = {"provisioning", provisioning_request_handler, NULL, NULL},
	.light_resource        = {"light", light_request_handler, NULL, NULL},
	.dim_resource          = {"dim", dim_request_handler, NULL, NULL},
};




/* ------------------- local functions implementation --------------------- */

/* PWM callback function */
void pwm_ready_callback(uint32_t pwm_id)    
{
    ready_flag = true;
}

/* Function to turn lights on */
static void light_on(void)
{
	last_light_state = true;

	/* set PWM value to the last received one */
	while (false == ready_flag);
	ready_flag = false;
	APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 0, last_dim_value));
}


/* Function to turn lights off */
static void light_off(void)
{
	last_light_state = false;

	/* set PWM value to 0 */
	while (false == ready_flag);
	ready_flag = false;
	APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 0, 0));
}


/* Function to toggle lights */
static void light_toggle(void)
{
	/* switch light state */
	if(true == last_light_state)
	{
		light_off();	
	}
	else
	{
		light_on();
	}

	NRF_LOG_INFO("light handler - command TOGGLE\r\n");
}


/* Function to disable provisioning */
static void provisioning_disable(otInstance * p_instance)
{
	m_app.enable_provisioning = false;
	m_app.provisioning_expiry = 0;

	otCoapRemoveResource(p_instance, &m_app.provisioning_resource);

	app_timer_stop(m_provisioning_timer);
}


/* Function to enable provisioning */
static void provisioning_enable(otInstance * p_instance)
{
	m_app.enable_provisioning = true;
	m_app.provisioning_expiry = otPlatAlarmGetNow() + PROVISIONING_EXPIRY_TIME;

	assert(otCoapAddResource(m_app.p_ot_instance, &m_app.provisioning_resource) == OT_ERROR_NONE);

	app_timer_start(m_provisioning_timer,
		         	APP_TIMER_TICKS(PROVISIONING_EXPIRY_TIME),
		           	p_instance);
}


/* Send dimming response message */
static void dim_response_send(void                * p_context,
                              otCoapHeader        * p_request_header,
                              const otMessageInfo * p_message_info)
{
    otError      error = OT_ERROR_NONE;
    otCoapHeader header;
    otMessage  * p_response;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CHANGED);
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

    if (error != OT_ERROR_NONE && p_response != NULL)
    {
        otMessageFree(p_response);
    }
}


/* Function to send light response */
static void light_response_send(void                * p_context,
                                otCoapHeader        * p_request_header,
                                const otMessageInfo * p_message_info)
{
    otError      error = OT_ERROR_NONE;
    otCoapHeader header;
    otMessage  * p_response;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CHANGED);
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

    if (error != OT_ERROR_NONE && p_response != NULL)
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
		if (otCoapHeaderGetType(p_header) != OT_COAP_TYPE_CONFIRMABLE &&
			otCoapHeaderGetType(p_header) != OT_COAP_TYPE_NON_CONFIRMABLE)
		{
			break;
		}

		if (otCoapHeaderGetCode(p_header) != OT_COAP_CODE_PUT)
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

		if (otCoapHeaderGetType(p_header) == OT_COAP_TYPE_CONFIRMABLE)
		{
			dim_response_send(p_context, p_header, p_message_info);
		}

	} while (false);
}


/* Function to handle light request */
static void light_request_handler(void                * p_context,
                                  otCoapHeader        * p_header,
                                  otMessage           * p_message,
                                  const otMessageInfo * p_message_info)
{
	(void)p_message;
	uint8_t command;

	do
	{
		if (otCoapHeaderGetType(p_header) != OT_COAP_TYPE_CONFIRMABLE &&
			otCoapHeaderGetType(p_header) != OT_COAP_TYPE_NON_CONFIRMABLE)
		{
			break;
		}

		if (otCoapHeaderGetCode(p_header) != OT_COAP_CODE_PUT)
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
				light_toggle();
         	break;
			case LIGHT_ON:
				light_on();
				break;
			case LIGHT_OFF:
				light_off(); 
				break;
         default:
				/* not supported command: do nothing */
         	break;
		}

		if (otCoapHeaderGetType(p_header) == OT_COAP_TYPE_CONFIRMABLE)
		{
			light_response_send(p_context, p_header, p_message_info);
		}

	} while (false);
}


/* Function to send provisioning response */
static otError provisioning_response_send(void                * p_context,
                                          otCoapHeader        * p_request_header,
                                          uint8_t               device_type,
                                          const otMessageInfo * p_message_info)
{
    otError      error = OT_ERROR_NO_BUFS;
    otCoapHeader header;
    otMessage  * p_response;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_CONTENT);
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
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        error = otMessageAppend(p_response, otThreadGetMeshLocalEid(p_context), sizeof(otIp6Address));
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        error = otCoapSendResponse(p_context, p_response, p_message_info);

    } while (false);

    if (error != OT_ERROR_NONE && p_response != NULL)
    {
        otMessageFree(p_response);
    }

    return error;
}


/* Function to request provisioning */
static void provisioning_request_handler(void                * p_context,
                                         otCoapHeader        * p_header,
                                         otMessage           * p_message,
                                         const otMessageInfo * p_message_info)
{
    (void)p_message;
    otMessageInfo message_info;

    if (!m_app.enable_provisioning)
    {
        return;
    }

    if (otCoapHeaderGetType(p_header) == OT_COAP_TYPE_NON_CONFIRMABLE &&
        otCoapHeaderGetCode(p_header) == OT_COAP_CODE_GET)
    {
        message_info = *p_message_info;
        memset(&message_info.mSockAddr, 0, sizeof(message_info.mSockAddr));
        if (provisioning_response_send(p_context, p_header, DEVICE_TYPE_LIGHT, &message_info) ==
            OT_ERROR_NONE)
        {
            provisioning_disable(p_context);
        }
    }
}


/* State change handler */
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
            provisioning_disable(p_context);
            break;
    }
}


/* State changed callack function */
static void state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_CHANGED_THREAD_ROLE)
    {
        role_change_handler(p_context, otThreadGetDeviceRole(p_context));
    }

    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otThreadGetDeviceRole(p_context));
}


/* Buttons events handler */
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


/* Provisioning timer handler */
static void provisioning_timer_handler(void * p_context)
{
    provisioning_disable(p_context);
}


/* LED timer handler */
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


/* Thread Initialization */
static void thread_init(void)
{
    otInstance * p_instance;

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


/* Init CoAp */
static void coap_init()
{
	m_app.light_resource.mContext = m_app.p_ot_instance;
	m_app.dim_resource.mContext = m_app.p_ot_instance;
	m_app.provisioning_resource.mContext = m_app.p_ot_instance;

	assert(otCoapStart(m_app.p_ot_instance, OT_DEFAULT_COAP_PORT) == OT_ERROR_NONE);
	assert(otCoapAddResource(m_app.p_ot_instance, &m_app.light_resource) == OT_ERROR_NONE);
	assert(otCoapAddResource(m_app.p_ot_instance, &m_app.dim_resource) == OT_ERROR_NONE);
}


/* Init timers */
static void timer_init(void)
{
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    app_timer_create(&m_provisioning_timer, APP_TIMER_MODE_SINGLE_SHOT, provisioning_timer_handler);
    app_timer_create(&m_led_timer, APP_TIMER_MODE_REPEATED, led_timer_handler);
}


/* Thread BSP init */
static void thread_bsp_init(void)
{
    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_thread_init(m_app.p_ot_instance);
    APP_ERROR_CHECK(err_code);
}


/* init LEDs */
static void leds_init(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);

    app_timer_start(m_led_timer, APP_TIMER_TICKS(LED_INTERVAL), NULL);
}




/*-------------- Main loop -------------------- */
int main(int argc, char *argv[])
{
	ret_code_t err_code;

	NRF_LOG_INIT(NULL);

	thread_init();
	coap_init();

	timer_init();
	thread_bsp_init();
	leds_init();

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

	/* infinite loop */
	while (true)
	{
		otTaskletsProcess(m_app.p_ot_instance);
		PlatformProcessDrivers(m_app.p_ot_instance);
	}
}




/* End of file */





