/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
* @brief Example template project.
* @defgroup nrf_templates_example Example Template
* @{
* @ingroup nrf_examples_nrf6310
*
*/

#include <stdbool.h>
#include <string.h>

#include "nrf_gpio.h"
#include "nrf_sdm.h"
#include "softdevice_handler.h"
#include "ble_stack_handler_types.h"
//#include "nrf.h"
//#include "app_error.h"
//#include "nrf51_bitfields.h"
#include "ble.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT     0  /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/


#define PIN_LED			(21)
#define	LED_ON			(0)
#define	LED_OFF			(1)

#define PIN_RFDET		(8)
#define PIN_SW			(9)
#define	PIN_IRQ			(11)
#define	PIN_SEL			(12)
#define	PIN_DATA		(13)
#define	PIN_SPICLK		(15)


/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
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


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
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

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, false);

#ifdef S110
    // Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
#endif

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);

}


static void init_gpio(void)
{
	//LED
	//プルアップ
	nrf_gpio_cfg_output(PIN_LED);
	nrf_gpio_pin_write(PIN_LED, LED_OFF);

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
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
	init_softdevice();
	init_gpio();

    while (true)
    {
        nrf_gpio_pin_write(PIN_LED, nrf_gpio_pin_read(PIN_RFDET));
    }
}
/** @} */
