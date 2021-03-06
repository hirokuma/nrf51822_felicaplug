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
#include "app_scheduler.h"

#include "nrf_gpio.h"
#include "nrf_sdm.h"
#include "softdevice_handler.h"

#include "ble_stack_handler_types.h"
#include "ble.h"
#include "ble_advdata.h"


#include "board_settings.h"
#include "felica/felica_plug.h"


////////////////////////////////////////////////////
// definition
////////////////////////////////////////////////////

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

#define GPIOTE_MASK_IRQ				(1 << PIN_IRQ)
#define GPIOTE_MASK_RFDET			(1 << PIN_RFDET)

#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define APP_COMPANY_IDENTIFIER			(0x004c)		/**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */


////////////////////////////////////////////////////
// private variable
////////////////////////////////////////////////////
static app_gpiote_user_id_t	sGpioteUserId;		//< GPIOTE用

static ble_gap_adv_params_t	sAdvParams;		/**< Parameters to be passed to the stack when starting advertising. */
static ble_advdata_manuf_data_t	sAdvManuData;
static ble_advdata_t		sAdvData;
static uint8_t sAdvFlag = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
static uint8_t sBeaconInfo[] =				/**< Information advertised by the Beacon. */
{
	0x02,				//
	0x15,				//ここから下のLEN

	// iBeacon proximity UUID
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//major
	0x00, 0x00,
	//minor
	0x00, 0x00,
	//RSSI
	0xc5
};


////////////////////////////////////////////////////
// private method
////////////////////////////////////////////////////

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t *p_ble_evt)
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
	LED_ON(PIN_ERRLED);
	for(;;){}
}


static void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
	//立ち上がり
	if (event_pins_low_to_high & GPIOTE_MASK_IRQ) {
		fp_irq_assert();
	}
	if (event_pins_low_to_high & GPIOTE_MASK_RFDET) {
		LED_OFF(PIN_LED);
		fp_stop();
	}

	//立ち下がり
	if (event_pins_high_to_low & GPIOTE_MASK_RFDET) {
		LED_ON(PIN_LED);
		fp_rfdet_assert();
	}
}


static void init_softdevice(void)
{
	uint32_t err_code;

	//外部16MHz xtalを動かす
	NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
	NRF_CLOCK->XTALFREQ = CLOCK_XTALFREQ_XTALFREQ_16MHz;	//16MHz xtal
	NRF_CLOCK->TASKS_HFCLKSTART = CLOCK_HFCLKRUN_STATUS_Triggered;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
	{
		//Do nothing.
	}
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

	// SoftDeviceの初期化
	//   32kHz : 内蔵
	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, true);

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

	//エラーLED(pullup)
	nrf_gpio_cfg_output(PIN_ERRLED);
	LED_OFF(PIN_ERRLED);

	//デバッグLED(pullup)
	nrf_gpio_cfg_output(PIN_DBGLED);
	LED_OFF(PIN_DBGLED);

	//LED(pullup)
	nrf_gpio_cfg_output(PIN_LED);
	LED_OFF(PIN_LED);

	//タイマ
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);

	//GPIO Task Event
	APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
	err_code = app_gpiote_user_register(&sGpioteUserId,
										GPIOTE_MASK_IRQ | GPIOTE_MASK_RFDET,	//立ち上がり
										GPIOTE_MASK_RFDET,						//立ち下がり
										gpiote_event_handler);
	APP_ERROR_CHECK(err_code);

	//スケジューラ
	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


////////////////////////////////////////////////////
// BLE
////////////////////////////////////////////////////

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t        err_code;

    sAdvManuData.company_identifier = APP_COMPANY_IDENTIFIER;
    sAdvManuData.data.p_data = (uint8_t *)sBeaconInfo;
    sAdvManuData.data.size   = sizeof(sBeaconInfo);

    // Build and set advertising data.
    memset(&sAdvData, 0, sizeof(sAdvData));

    sAdvData.name_type               = BLE_ADVDATA_NO_NAME;
    sAdvData.flags.size              = sizeof(sAdvFlag);
    sAdvData.flags.p_data            = &sAdvFlag;
    sAdvData.p_manuf_specific_data   = &sAdvManuData;

    err_code = ble_advdata_set(&sAdvData, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&sAdvParams, 0, sizeof(sAdvParams));

    sAdvParams.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    sAdvParams.p_peer_addr = NULL;                             // Undirected advertisement.
    sAdvParams.fp          = BLE_GAP_ADV_FP_ANY;
    sAdvParams.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    sAdvParams.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


////////////////////////////////////////////////////
// FeliCa Plug call back
////////////////////////////////////////////////////

/**
 * Write w/o Encryptionの受信完了コールバック
 *
 * @param[in,out]	pBuf		受信データ/送信データ
 * @param[in]		rsize		受信サイズ
 * @param[in]		blocks		受信ブロック数
 * @note	pBufに書き込んだ先頭2バイトをR/Wに返す。
 *
 * Write w/o Encコマンド受信(これが最大)
 * [0] : コマンドコード
 * [1] : ブロック数
 * [2～2+2*[1]-1]: ブロックリスト(2*[1])
 * [2+2*[1]]     : ブロックデータ(16*[1])
 *
 * Write w/o Encレスポンス送信
 * 1 : ステータスフラグ1
 * 1 : ステータスフラグ2
 */
static void fp_write_req(uint8_t *pBuf, uint8_t rsize, uint8_t blocks)
{
	uint32_t err_code;

	if ((blocks == 1) && (pBuf[3] == 0x00)) {

		//ブロック0への書き込み要求
		memcpy(&sBeaconInfo[2], &pBuf[2 + 2*blocks], 16);	//UUID
		err_code = ble_advdata_set(&sAdvData, NULL);			//再設定
		APP_ERROR_CHECK(err_code);

		//Advertiting開始
		sd_ble_gap_adv_stop();
		err_code = sd_ble_gap_adv_start(&sAdvParams);
		APP_ERROR_CHECK(err_code);

		LED_ON(PIN_DBGLED);
	}
	else {
		//ここはエラーを見ない
		sd_ble_gap_adv_stop();
		//APP_ERROR_CHECK(err_code);

		LED_OFF(PIN_DBGLED);
	}

	//レスポンス
	pBuf[0] = FP_STATFLAG1_SUCCESS;
	pBuf[1] = FP_STATFLAG2_SUCCESS;
}

/**
 * Read w/o Encryptionの受信完了コールバック
 *
 * @param[in,out]	pBuf		受信データ/送信データ
 * @param[in]		rsize		受信サイズ
 * @param[in]		blocks		受信ブロック数
 * @note	pBufに書き込んだ先頭2バイト + FP_TAG_BLOCK * blocksをR/Wに返す。
 *
 * Read w/o Encコマンド受信
 * 1 : コマンドコード
 * 1 : ブロック数
 * 3*12 : ブロックリスト(3バイトタイプ)
 *
 * Read w/o Encレスポンス送信
 * 1 : ステータスフラグ1
 * 1 : ステータスフラグ2
 * 16*12 : 読み出したブロックデータ
 */
static void fp_read_req(uint8_t *pBuf, uint8_t rsize, uint8_t blocks)
{
	if ((blocks == 1) && (pBuf[3] == 0x00)) {
		//レスポンス
		pBuf[0] = FP_STATFLAG1_SUCCESS;
		pBuf[1] = FP_STATFLAG2_SUCCESS;
		//UUIDを返す
		memcpy(&pBuf[2], &sBeaconInfo[2], 16);
	}
	else {
		//レスポンス
		pBuf[0] = FP_STATFLAG1_ERROR;
		pBuf[1] = 0x80;
		memset(&pBuf[2], 0x00, blocks);
	}
}


////////////////////////////////////////////////////
// public method
////////////////////////////////////////////////////

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
	//NVIC_SystemReset();
	LED_ON(PIN_ERRLED);
	while(1);
}


/**@brief	SoftDeviceからのassertコールバック関数(外部から呼ばれる)
 *
 * @warning		リセットしない限り正常に戻らない
 *
 * @param[in]   line_num	行番号
 * @param[in]   p_file_name	ファイル名
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
	app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**
 * @brief	main()
 */
int main(void)
{
	uint32_t err_code;

	init_softdevice();
	fp_init(fp_write_req, fp_read_req);
	advertising_init();

	//
	err_code = app_gpiote_user_enable(sGpioteUserId);
	APP_ERROR_CHECK(err_code);

	while (true) {
		app_sched_execute();
		sd_app_evt_wait();
		fp_event_loop();
	}
}
/** @} */
