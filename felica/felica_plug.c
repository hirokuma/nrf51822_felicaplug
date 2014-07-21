/**
 * @file	felicaplug.c
 * @brief	FeliCa Plug用
 */

#include <string.h>

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "spi_master.h"
#include "app_error.h"
#include "nrf_soc.h"
#include "nrf51_bitfields.h"
#include "app_util_platform.h"

#include "board_settings.h"
#include "felica_plug.h"


/*******************************************************************/

#define SZ_SPIBUF		(256)	/* 送受信バッファサイズ */

/*******************************************************************/

/* SW */
#define FP_SW_SET()		{ nrf_gpio_pin_set(PIN_SW); }
#define FP_SW_CLR()		{ nrf_gpio_pin_clear(PIN_SW); }

/* SEL */
#define FP_SEL_SET()	{ nrf_gpio_pin_set(PIN_SEL); }
#define FP_SEL_CLR()	{ nrf_gpio_pin_clear(PIN_SEL); }
#define FP_SEL_GET()	(nrf_gpio_pin_read(PIN_SEL))

/* 50usec待つ */
#define FP_RFDET_WAIT()	{ nrf_delay_us(50); }

/* 受信　IN=MISO / OUT=disconnect */
#define FP_MOSI_HIZ()	{				\
		NRF_SPI0->CONFIG = (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos) | (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos);	\
		NRF_SPI0->PSELMOSI = SPI_PIN_DISCONNECTED;	\
		NRF_SPI0->PSELMISO = PIN_DATA;		\
	}
/* 送信　IN=disconnect / OUT=MOSI */
#define FP_MISO_HIZ()	{				\
		NRF_SPI0->CONFIG = (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos) | (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos);	\
		NRF_SPI0->PSELMISO = SPI_PIN_DISCONNECTED;	\
		NRF_SPI0->PSELMOSI = PIN_DATA;		\
	}

/* 有線コマンド */
#define FPCMD_READ_WO_ENC		(0x06)		/* Read w/o Encryption */
#define FPCMD_WRITE_WO_ENC		(0x08)		/* Write w/o Encryption */


/*******************************************************************/

typedef void (*EVENT_FUNC_TYPE)(void);

/*******************************************************************/

/* 初期化パラメータ */
static const uint8_t k_InitVal[] = {
	/* 各種設定パラメータ */
	0x03,					/* データ転送設定 : FT転送設定		*/
							/* システムコード : 0x12FC			*/

	/* 最大応答時間パラメータ */
	0xff, 0xff,				/* Read w/o Enctyption  : 255		*/
							/* Write w/o Enctyption : 255		*/


	/* データフォーマットコード */
	0x00, 0x1c,				/* Switch Science社から購入の場合	*/

	/* ユーザー設定パラメータ */
	'k', 'u', 'm', 'a'
};


/*******************************************************************/

static uint8_t	sSpiBuf[SZ_SPIBUF];		//< SPI送受信バッファ
static FpStatus	sStatus;				//< ステータス

/*******************************************************************/

static void recv_rwe(void);
static void recv_wwe(void);
static void spi_event_handler(spi_master_evt_t spi_master_evt);


/*******************************************************************
 * 公開API
 *******************************************************************/

void fp_init(void)
{
	uint32_t err_code;

	//nRFDET
	nrf_gpio_cfg_input(PIN_RFDET, NRF_GPIO_PIN_PULLUP);

	//SW
	nrf_gpio_cfg_output(PIN_SW);
	FP_SW_CLR();

	//IRQ
	nrf_gpio_cfg_input(PIN_IRQ, NRF_GPIO_PIN_PULLDOWN);

	//SEL
	nrf_gpio_cfg_output(PIN_SEL);
	FP_SEL_CLR();

	//DATA
	nrf_gpio_cfg_input(PIN_DATA, NRF_GPIO_PIN_PULLDOWN);

	//SPICLK
	nrf_gpio_cfg_output(PIN_SPICLK);
	nrf_gpio_pin_set(PIN_SPICLK);


	/* SPI初期化 */
	spi_master_config_t spi_config = {
		SPI_FREQUENCY_FREQUENCY_M1,		/**< Serial clock frequency 1 Mbps. */
		PIN_SPICLK,						/**< SCK pin */
		PIN_DATA,						/**< MISO pin */
		SPI_PIN_DISCONNECTED,			/**< MOSI pin DISCONNECTED(受信). */
		PIN_SEL,						/**< Slave select pin. */
		APP_IRQ_PRIORITY_LOW,			/**< Interrupt priority LOW. */
		SPI_CONFIG_ORDER_MsbFirst,		/**< Bits order MSB. */
		SPI_CONFIG_CPOL_ActiveLow,		/**< Serial clock polarity ACTIVELOW. */
		SPI_CONFIG_CPHA_Trailing,		/**< 受信：Trailing  送信：Leading */
		0								/**< Don't disable all IRQs. */
	};

	err_code = spi_master_open(SPI_MASTER_0, &spi_config);
	APP_ERROR_CHECK(err_code);

	//Register event handler for SPI master.
	spi_master_evt_handler_reg(SPI_MASTER_0, spi_event_handler);

	sStatus = FPST_RFDET_WAIT;
}


/**
 * RFDET割込検出
 */
void fp_rfdet_assert(void)
{
	if (sStatus == FPST_RFDET_WAIT) {
		/* 搬送波待ち→FeliCa Plug初期化 */

		FP_SEL_CLR();
		FP_SW_SET();
		FP_RFDET_WAIT();

		/* 初期パラメータ転送開始 */
		FP_MISO_HIZ();
		memcpy(sSpiBuf, k_InitVal, sizeof(k_InitVal));	//constじゃだめなので・・・.
		uint32_t err_code = spi_master_send_recv(SPI_MASTER_0, sSpiBuf, sizeof(k_InitVal), NULL, 0);
		APP_ERROR_CHECK(err_code);
	} else {
		fp_stop();
	}
}


/**
 * IRQ割込検出によるDATA読み込み
 */
void fp_irq_assert(void)
{
	FP_SEL_SET();			/* SEL=H */
	FP_MOSI_HIZ();			/* MOSI→GPIO HiZ */

	/* 2byte読んで、全受信データを計算する */
	uint32_t err_code = spi_master_send_recv(SPI_MASTER_0, NULL, 0, sSpiBuf, 2);
	APP_ERROR_CHECK(err_code);
}


/**
 * 停止
 */
void fp_stop(void)
{
	FP_SEL_CLR();
	FP_SW_CLR();

	sStatus = FPST_RFDET_WAIT;
}


FpStatus fp_get_status(void)
{
	return sStatus;
}

/*******************************************************************
 * static関数
 *******************************************************************/

/**
 * データ受信完了：Read w/o Encryption
 */
static void recv_rwe(void)
{
	uint16_t size;
	int i;
	uint8_t num = sSpiBuf[1];	/* ブロック数 */

	FP_SEL_CLR();			/* SEL=L */
	FP_MISO_HIZ();			/* MOSI→MOSI */

	size = 0;
	sSpiBuf[size++] = 0x00;		/* ステータスフラグ1 */
	sSpiBuf[size++] = 0x00;		/* ステータスフラグ2 */
	for(i=0; i<16*num; i++) {
		sSpiBuf[size++] = i;
	}
	FP_MISO_HIZ();
	uint32_t err_code = spi_master_send_recv(SPI_MASTER_0, sSpiBuf, size, NULL, 0);
	APP_ERROR_CHECK(err_code);
}


/**
 * データ受信完了：Write w/o Encryption
 */
static void recv_wwe(void)
{
	uint16_t size;

	FP_SEL_CLR();			/* SEL=L */
	FP_MISO_HIZ();			/* MOSI→MOSI */

	size = 2;
	sSpiBuf[0] = 0x00;
	sSpiBuf[1] = 0x00;
	FP_MISO_HIZ();
	uint32_t err_code = spi_master_send_recv(SPI_MASTER_0, sSpiBuf, size, NULL, 0);
	APP_ERROR_CHECK(err_code);
}


/**@brief Handler for SPI0 master events.
 *
 * @param[in] spi_master_evt    SPI master event.
 */
static void spi_event_handler(spi_master_evt_t spi_master_evt)
{
	uint32_t err_code;

	switch (spi_master_evt.evt_type) {
	case SPI_MASTER_EVT_TRANSFER_COMPLETED:
		switch (sStatus) {
		case FPST_RFDET_WAIT:	//初期パラメータ転送完了後
		case FPST_SPI_SEND:		//SPI送信後
			FP_SEL_SET();				/* SEL=H */
			sStatus = FPST_IRQ_WAIT;
			break;

		case FPST_IRQ_WAIT:		//R/Wからの先頭2byte受信後
			{
				uint16_t size;
				if(sSpiBuf[0] == FPCMD_READ_WO_ENC) {
					/* Read w/o Enc */
					size = 2 * sSpiBuf[1];
				} else if(sSpiBuf[0] == FPCMD_WRITE_WO_ENC) {
					/* Write w/o Enc */
					size = 2 * sSpiBuf[1] + 16 * sSpiBuf[1];
				} else {
					/* ?? */
				}
				//残りを受信
				err_code = spi_master_send_recv(SPI_MASTER_0, NULL, 0, sSpiBuf + 2, size);
				APP_ERROR_CHECK(err_code);

				sStatus = FPST_SPI_RECV;
			}
			break;

		case FPST_SPI_RECV:		//R/Wからの残りデータ受信後
			if(sSpiBuf[0] == FPCMD_READ_WO_ENC) {
				/* Read w/o Enc */
				recv_rwe();
			} else if(sSpiBuf[0] == FPCMD_WRITE_WO_ENC) {
				/* Write w/o Enc */
				recv_wwe();
			} else {
				fp_stop();
			}
			sStatus = FPST_SPI_SEND;
			break;

		default:
			fp_stop();
			break;
		}
		break;

	default:
		break;
	}
}

