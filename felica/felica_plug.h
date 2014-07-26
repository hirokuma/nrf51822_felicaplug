/**
 * @file	felica_plug.h
 * @brief	FeliCa Plug制御
 */
#ifndef FELICA_PLUG_H
#define FELICA_PLUG_H

/*******************************************************************/
#define FP_TAG_BLOCK		(16)		/**< 1ブロック16byte */

#define FP_STATFLAG1_SUCCESS	(0x00)
#define FP_STATFLAG1_ERROR		(0xff)

#define FP_STATFLAG2_SUCCESS	(0x00)
#define FP_STATFLAG2_DATA		(0x70)


/*******************************************************************/

/**
 * @enum FpStatus
 */
typedef enum FpStatus {
	FPST_INIT,			/**< 初期状態 */
	FPST_RFDET_WAIT,	/**< RFDET割込み待ち */
	FPST_IRQ_WAIT,		/**< IRQ割込み待ち */
	FPST_SPI_RECV,		/**< SPI受信中 */
	FPST_SPI_SEND,		/**< SPI送信中 */
} FpStatus;


typedef void (*FpWriteReq)(uint8_t *pBuf, uint8_t size, uint8_t blocks);
typedef void (*FpReadReq)(uint8_t *pBuf, uint8_t size, uint8_t blocks);

/*******************************************************************/

void fp_init(FpWriteReq writeFunc, FpReadReq readFunc);
void fp_rfdet_assert(void);
void fp_irq_assert(void);
void fp_stop(void);
void fp_event_loop(void);
FpStatus fp_get_status(void);

/*******************************************************************/

#endif /* FELICA_PLUG_H */
