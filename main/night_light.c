/*
 * SPDX-FileCopyrightText: 2020-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "lwip/err.h"
#include "esp_sleep.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
#include "time.h"
#include "driver/ledc.h"

#define GPIO_OUTPUT_PIN_SEL  (1ULL<<  CONFIG_MAIN_LIGHT_OUT)
#define GPIO_INPUT_PIN_SEL  (1ULL << CONFIG_PIR_SENSOR_IN)

#define ESP_INTR_FLAG_DEFAULT 0

int cnt = 0;

const ledc_timer_config_t pwmTmrConf = \
	{.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT, \
	.timer_num = LEDC_TIMER_0, .freq_hz = 16000, .clk_cfg = LEDC_AUTO_CLK, \
	.deconfigure = 0};

const ledc_channel_config_t ledc_conf = \
		{.gpio_num = CONFIG_MAIN_LIGHT_OUT, .channel = LEDC_CHANNEL_0, \
		.timer_sel = pwmTmrConf.timer_num, .speed_mode = pwmTmrConf.speed_mode, \
		.intr_type = GPIO_INTR_DISABLE, .duty = 0, .hpoint = 0, \
		.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD, .flags.output_invert = 0};


typedef enum
{
	none_e = 0,
	evt_pon = 1,
	evt_pin = 2,
	evt_pwm = 4,
	evt_pir = 8
}evts_e;

typedef struct
{
	uint16_t	mgstype;
	uint16_t	data0;
	uint32_t	data1;		
}msg_t;

typedef enum
{
	reset_e,
	post_reset_e,
	waitTrig_e,
	gotTrig_e,
	fadeIn_e,
	ledOn_e,
	fadeOut_e,
	goSleep_e	
}states_e;

typedef struct
{
	ledc_channel_config_t* out;
	int16_t sval;
	int16_t tval;
	int16_t cval;
	uint16_t ctim;
	uint16_t ttim;
	uint16_t  done:1, upd:1, sleep:1, dc:13;	
}interpolate_t;

typedef struct
{
	ledc_timer_config_t*   ledTmrCfg;
	ledc_channel_config_t* ledChanCfg;
	int16_t		fadeIn;
	int16_t		fadeOut;
	int16_t		wait;
	int16_t		maxInt;
}cfg_t;

typedef struct
{
	states_e		state;
	QueueHandle_t	evt_queue;
	interpolate_t 	interpolator;
	cfg_t		  	cfg;
}sys_t;

sys_t sys_inst;
sys_t* sys = &sys_inst;

const msg_t pir_msg = {.mgstype = evt_pir, .data0 = CONFIG_PIR_SENSOR_IN, .data1 = 0};

static IRAM_ATTR bool cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg)
{
    BaseType_t taskAwoken = pdFALSE;
//	const msg_t msg = {.mgstype = evt_pwm, .data0 = param->event, .data1 = param->duty};
//	xQueueSendFromISR(evt_queue, &msg, &taskAwoken);
    return (taskAwoken == pdTRUE);
}

void procInterpolate(interpolate_t* ipol)
{
	int32_t vdif = ipol->tval - ipol->sval;
	if(vdif == 0)
	{
		ipol->ctim = ipol->ttim;
		ipol->cval = ipol->tval;
		ipol->done = 1;
		return;
	}
	if(ipol->ctim == ipol->ttim)
	{
		ipol->cval = ipol->tval;
		ipol->done = 1;
		ipol->upd = 1;
		return;
	}
	const int32_t tdif = ipol->ctim * vdif;
	ipol->cval = (tdif / ipol->ttim) + ipol->sval;
	ipol->done = 0;
	ipol->upd = 1;
	ipol->ctim++;
}

void dumpInterpolator(const interpolate_t* ipol)
{
	printf(" %5d |%5d |%5d |%5u |%5u |%5d |%5d\n", \
	ipol->sval, ipol->cval, ipol->tval, ipol->ctim, ipol->ttim, ipol->done, ipol->upd);
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	if((sys) && sys->evt_queue)	xQueueSendFromISR(sys->evt_queue, &pir_msg, NULL);
}

static void interpolator(void* arg)
{
	interpolate_t* ipol =(interpolate_t*)arg;
	//printf("interpolator started\n");
	procInterpolate(ipol);
	while(1)
	{
		if(ipol->done < 1)
		{
			procInterpolate(ipol);
			//dumpInterpolator(ipol);			
			if(ipol->upd)
			{
				ledc_set_duty(ipol->out->speed_mode, ipol->out->channel, (uint32_t)ipol->cval);
				ledc_update_duty(ipol->out->speed_mode, ipol->out->channel);
			}
			if(ipol->done)
			{
				const msg_t msg = {.mgstype = evt_pwm, .data0 = 0, .data1 = 0};
				if((sys) && sys->evt_queue)	xQueueSend(sys->evt_queue, &msg, portMAX_DELAY);
			}
			ipol->upd = 0;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
		if(ipol->sleep)
		{
			vTaskDelete(NULL);
			return;
		}
	}
}

void setOut(interpolate_t* ipol, int16_t val, int16_t milisec)
{
	ipol->ctim = 0;
	ipol->ttim = milisec / 10;
	ipol->done = 0;
	ipol->upd = 0;
	ipol->sval = ipol->cval;
	ipol->tval = val;
}

static void app_task(void* arg)
{
    //gpio_set_level(GPIO_OUTPUT_IO_0, 0);
    states_e state = reset_e;
    uint32_t tMax = (sys->cfg.maxInt * (1u << sys->cfg.ledTmrCfg->duty_resolution));
    uint16_t MAXDUTY = (uint16_t)(tMax / 100u);
    const cfg_t* cfg = &sys->cfg;
	interpolate_t* ipol = &sys->interpolator;
	memset((void*)(ipol), 0, sizeof(interpolate_t));
	ledc_set_duty_with_hpoint(ledc_conf.speed_mode, ledc_conf.channel, 0, ledc_conf.hpoint);
	ipol->out = sys->cfg.ledChanCfg;
    printf("appTask started, max_duty %u\n",MAXDUTY);
    dumpInterpolator(ipol);
    xTaskCreate(interpolator, "interpolator", 4096, ipol, 11, NULL);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    BaseType_t qres;
    msg_t msg;

    while(1)
    {
		msg.mgstype = none_e;
		qres = xQueueReceive(sys->evt_queue, &msg, pdMS_TO_TICKS(1000));
		if(qres == pdFAIL) cnt++;
		else
		{
			if(msg.mgstype == evt_pir) cnt = 0;
			if(msg.mgstype == evt_pin) state = gotTrig_e;
		}

		//printf("----------------------------------------------\nGot MSG %u, data0 %u, data1 %lu\n", msg.mgstype, msg.data0, msg.data1);

		switch(state)
		{
			case reset_e:
			{
				printf("reset_e : PWM start\n");
				setOut(ipol, MAXDUTY/4, 1500);
				state = post_reset_e;
				break;
			}
			case post_reset_e:
			{
				printf("Entered post_reset_e state, cnt %d\n", cnt);
				if(msg.mgstype == evt_pwm)
				{
					printf("post_reset_e : PWM done\n");
					setOut(ipol, 0, 1000);
					state = waitTrig_e;
					cnt = 0;
				}
				break;
			}
			case gotTrig_e:
			{
				//xQueueSend(evt_queue, &pir_msg, portMAX_DELAY);
				setOut(ipol, MAXDUTY, cfg->fadeIn);
				printf("Entered gotTrig_e state\n");
				//state = waitTrig_e;
				state = fadeIn_e;
				break;
			}
			case waitTrig_e:
			{
				printf("Entered waitTrig_e state, msg.mgstype %u\n",msg.mgstype);
				switch(msg.mgstype)
				{
					case evt_pir:
					{
						printf("waitTrig_e state PIR TRIG\n");
						setOut(ipol, MAXDUTY, cfg->fadeIn);
						state = fadeIn_e;
						break;
					}
					case none_e:
					{
						if(cnt > cfg->wait) state = goSleep_e;
						printf("waitTrig_e state CNT %d\n", cnt);
						break;
					}
					default:
					{
						printf("waitTrig_e state def\n");
						break;
					}
				}
				break;
			}

			case fadeIn_e:
			{
				if(msg.mgstype == evt_pwm)
				{
					printf("fadeIn_e : PWM done\n");
					state = ledOn_e;
				}
				cnt = 0;
				break;
			}
			case ledOn_e:
			{
				printf("Entered ledOn_e state, msg.mgstype %u\n",msg.mgstype);
				const int pinv = gpio_get_level(CONFIG_PIR_SENSOR_IN);
				if(pinv)
				{
					cnt = 0;
				}
				else
				{
					switch(msg.mgstype)
					{
						case evt_pir:
						{
							cnt = 0;
							printf("ledOn_e state PIR\n");
							break;
						}
						case none_e:
						{
							if(cnt > cfg->wait)
							{
								state = fadeOut_e;
								setOut(ipol, 0, cfg->fadeOut);
							}
							break;
						}
						default:
						{
							printf("ledOn_e state other state\n");
							break;
						}
					}
				}
				printf("ledOn_e state CNT %d\n", cnt);
				break;
			}
			case fadeOut_e:
			{
				printf("Entered fadeOut_e state, msg.mgstype %u\n",msg.mgstype);
				switch(msg.mgstype)
				{
					case evt_pwm:
					{
						//state = goSleep_e;
						state = waitTrig_e;
						cnt = 0;
						printf("fadeOut_e PWM DONE, wait trigger\n");
						break;
					}
					case evt_pir:
					{
						cnt = 0;
						state = fadeIn_e;
						//xQueueSend(evt_queue, &pir_msg, portMAX_DELAY);
						setOut(ipol, MAXDUTY, cfg->fadeIn);
						printf("fadeOut_e state PIR >>>>>>>>>>>>>>>>>>>> \n");						
						break;
					}
					default:
					{
						printf("fadeOut_e state other state\n");
						break;
					}
				}				
				break;
			}
			case goSleep_e:
			{
				setOut(ipol, 0, 100);
				ipol->sleep = 1;
				vTaskDelay(150 / portTICK_PERIOD_MS);
				printf("GOING TO SLEEP\n");
				esp_deep_sleep_start();
				vTaskDelete(NULL);
				return;
				break;
			}
			default:break;
		}
	}
}

void config_led_pwm(void)
{
	esp_err_t ec = ledc_timer_config(&pwmTmrConf);
	printf("ledc_timer_config returns(%d) %s\n", ec, esp_err_to_name(ec));
	ec = ledc_channel_config(&ledc_conf);
	printf("ledc_channel_config returns(%d) %s\n", ec, esp_err_to_name(ec));
	ledc_fade_func_install(0);
	ledc_cbs_t callbacks = {.fade_cb = cb_ledc_fade_end_event};
    ledc_cb_register(ledc_conf.speed_mode, ledc_conf.channel, &callbacks, NULL);
}

void config_pins(void)
{
	gpio_config_t io_conf = {};
	 //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin

        //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup((GPIO_INPUT_PIN_SEL), ESP_GPIO_WAKEUP_GPIO_HIGH));
	printf("Enabling GPIO wakeup on pins GPIO%d\n", 0);
	vTaskDelay(pdMS_TO_TICKS(10));
	gpio_isr_handler_add(CONFIG_PIR_SENSOR_IN, gpio_isr_handler, NULL);    
}

void config_peripherals(void)
{
	config_pins();
	printf("========================= PINS configured ====================================\n");
	config_led_pwm();
	printf("========================= LEDS configured ====================================\n");
	sys->cfg.ledChanCfg = &ledc_conf;
	sys->cfg.ledTmrCfg = &pwmTmrConf;
	sys->cfg.fadeIn = CONFIG_FADE_IN_MS;
	sys->cfg.fadeOut = CONFIG_FADE_OUT_MS;
	sys->cfg.wait = CONFIG_WAIT_ON_MS;
	sys->cfg.maxInt = CONFIG_MAX_INTEN;
}

esp_sleep_wakeup_cause_t getWakeData(void)
{
	const esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
	switch(wc)
	{
		case ESP_SLEEP_WAKEUP_GPIO:
		{
			uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
			if (wakeup_pin_mask != 0)
			{
			    int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
			    printf("Wake up from GPIO %d\n", pin);
			}
			else
			{
			    printf("Wake up from GPIO\n");
			}
			const msg_t msg = {.mgstype = evt_pin, .data0 = (uint16_t)wc, .data1 = wakeup_pin_mask};
			xQueueSend(sys->evt_queue, &msg, portMAX_DELAY);
			break;
		}
		default:
		{
			printf("Wake up from OTHER %u\n", wc);
			const msg_t msg = {.mgstype = evt_pon, .data0 = (uint16_t)wc, .data1 = 0};
			xQueueSend(sys->evt_queue, &msg, portMAX_DELAY);
		}
	}
	return(wc);
}
void app_main(void)
{   
    sys->evt_queue = xQueueCreate(16, sizeof(msg_t));
    vTaskDelay(20 / portTICK_PERIOD_MS);
    getWakeData();
	config_peripherals();
    //create a queue to handle gpio event from isr

    xTaskCreate(app_task, "gpio_task_example", 4096, (void*)sys, 10, NULL);    
	cnt = 0;
    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
