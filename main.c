/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 hirokuma
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
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdbool.h>
#include <string.h>

#include "app_timer.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "app_scheduler.h"

#include "nrf_gpio.h"
#include "nrf_sdm.h"
#include "softdevice_handler.h"
#include "ble_stack_handler_types.h"
//#include "app_error.h"
//#include "nrf51_bitfields.h"
#include "ble.h"

//SoftDeviceのスケジューラを使うかどうか。コメントアウトで未使用。
#define USE_SD_SCHEDULER


/** Include or not the service_changed characteristic.
 * if not enabled, the server's database cannot be changed for the lifetime of the device
 */
#define IS_SRVC_CHANGED_CHARACT_PRESENT		(0)

/** Value of the RTC1 PRESCALER register. */
#define APP_TIMER_PRESCALER			(0)
/** Maximum number of simultaneously created timers. */
#define APP_TIMER_MAX_TIMERS		(2)
/** Size of timer operation queues. */
#define APP_TIMER_OP_QUEUE_SIZE		(4)

/** Maximum number of users of the GPIOTE handler. */
#define APP_GPIOTE_MAX_USERS		(1)

/** Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */
#define BUTTON_DETECTION_DELAY		(APP_TIMER_TICKS(5, APP_TIMER_PRESCALER))

/** Maximum size of scheduler events.
 *     Note that scheduler BLE stack events do not contain any data, 
 *       as the events are being pulled from the stack in the event handler.
 */
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)
/** Maximum number of events in the scheduler queue. */
#define SCHED_QUEUE_SIZE			(10)


#define PIN_LED			(21)
#define	LED_ON			(0)
#define	LED_OFF			(1)

#define PIN_RFDET		(8)
#define PIN_SW			(9)
#define	PIN_IRQ			(11)
#define	PIN_SEL			(12)
#define	PIN_DATA		(13)
#define	PIN_SPICLK		(15)


/**@brief	エラーハンドラ
 *
 * @param[in] error_code	エラーコード
 * @param[in] line_num		行番号
 * @param[in] p_file_name	ファイル名
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
	// This call can be used for debug purposes during application development.
	// @note CAUTION: Activating this code will write the stack to flash on an error.
	//                This function should NOT be used in a final product.
	//                It is intended STRICTLY for development/debugging purposes.
	//                The flash write will happen EVEN if the radio is active, thus interrupting
	//                any communication.
	//                Use with care. Uncomment the line below to use.
	// ble_debug_assert_handler(error_code, line_num, p_file_name);

	// On assert, the system can only recover with a reset.
	NVIC_SystemReset();
}


/**@brief	SoftDeviceからのassertコールバック関数
 *
 * @warning		リセットしない限り正常に戻らない
 *
 * @param[in]   line_num	行番号
 * @param[in]   p_file_name	ファイル名
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
	app_error_handler(1, line_num, p_file_name);
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
//    ble_bondmngr_on_ble_evt(p_ble_evt);
//    ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
//    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
//    ble_conn_params_on_ble_evt(p_ble_evt);
//    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
//    pstorage_sys_event_handler(sys_evt);
}


static void init_softdevice(void)
{
	//外部16MHz xtalを動かす
	NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
	NRF_CLOCK->XTALFREQ = CLOCK_XTALFREQ_XTALFREQ_16MHz;	//16MHz xtal
	NRF_CLOCK->TASKS_HFCLKSTART = CLOCK_HFCLKRUN_STATUS_Triggered;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
	{
		//Do nothing.
	}
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

	uint32_t err_code;

	// SoftDeviceの初期化
	//   32kHz : 内蔵
#ifdef USE_SD_SCHEDULER
	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, true);
#else //USE_SD_SCHEDULER
	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, false);
#endif //USE_SD_SCHEDULER

	// Enable BLE stack
	ble_enable_params_t ble_enable_params;
	memset(&ble_enable_params, 0, sizeof(ble_enable_params));
	ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
	err_code = sd_ble_enable(&ble_enable_params);
	APP_ERROR_CHECK(err_code);

	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);

	// Register with the SoftDevice handler module for System (SOC) events.
	err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
	APP_ERROR_CHECK(err_code);

}


/**@brief	app_buttonのイベントハンドラ
 *
 * @param[in]   pin_no			アクションが発生したピン番号
 * @param[in]	button_action	APP_BUTTON_XXX
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
	switch (pin_no) {
	case PIN_RFDET:
		nrf_gpio_pin_write(PIN_LED, (button_action == APP_BUTTON_PUSH) ? LED_ON : LED_OFF);
		break;

	default:
		APP_ERROR_HANDLER(pin_no);
		break;
	}
}


/**@brief	GPIO関連の初期化
 *
 */
static void init_gpio(void)
{
	//LED
	//プルアップ
	nrf_gpio_cfg_output(PIN_LED);
	nrf_gpio_pin_write(PIN_LED, LED_OFF);

	//
	//FeliCa Plug
	//

	//nRFDET
	nrf_gpio_cfg_input(PIN_RFDET, NRF_GPIO_PIN_PULLUP);

	//SW
	nrf_gpio_cfg_output(PIN_SW);
	nrf_gpio_pin_write(PIN_SW, 0);

	//IRQ
	nrf_gpio_cfg_input(PIN_IRQ, NRF_GPIO_PIN_PULLDOWN);

	//SEL
	nrf_gpio_cfg_output(PIN_SEL);
	nrf_gpio_pin_write(PIN_SEL, 0);

	//DATA
	nrf_gpio_cfg_input(PIN_DATA, NRF_GPIO_PIN_PULLDOWN);

	//SPICLK
	nrf_gpio_cfg_output(PIN_SPICLK);
	nrf_gpio_pin_write(PIN_SPICLK, 0);

	//
	// app_button設定関連
	//
	
	//タイマ
#ifdef USE_SD_SCHEDULER
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
#else //USE_SD_SCHEDULER
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
#endif //USE_SD_SCHEDULER

	//GPIO Task Event
	APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);

	//app_button
	static app_button_cfg_t buttons[] = {
		{PIN_RFDET, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_event_handler},
	};
#ifdef USE_SD_SCHEDULER
	APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, true);
#else //USE_SD_SCHEDULER
	APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, false);
#endif //USE_SD_SCHEDULER
}


#ifdef USE_SD_SCHEDULER
/**@brief スケジューラの初期化
 */
static void init_scheduler(void)
{
	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}
#endif	//USE_SD_SCHEDULER


/**
 * @brief	main()
 */
int main(void)
{
	init_softdevice();
	init_gpio();
#ifdef USE_SD_SCHEDULER
	init_scheduler();
#endif	//USE_SD_SCHEDULER

	app_button_enable();

	while (true) {
#ifdef USE_SD_SCHEDULER
		app_sched_execute();
		sd_app_evt_wait();
#endif	//USE_SD_SCHEDULER
	}
}
/** @} */
