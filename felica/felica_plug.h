/**
 * @file	felica_plug.h
 * @brief	FeliCa Plug制御
 */
#ifndef FELICA_PLUG_H
#define FELICA_PLUG_H

/**@enum FpStatus
 */
typedef enum FpStatus {
	FPST_INIT,			/**< 初期状態 */
	FPST_RFDET_WAIT,	/**< RFDET割込み待ち */
	FPST_IRQ_WAIT,		/**< IRQ割込み待ち */
	FPST_SPI_RECV,		/**< SPI受信中 */
	FPST_SPI_SEND,		/**< SPI送信中 */
} FpStatus;


/*******************************************************************/

void fp_init(void);
void fp_rfdet_assert(void);
void fp_irq_assert(void);
void fp_stop(void);
void fp_event_loop(void);
FpStatus fp_get_status(void);

#endif /* FELICA_PLUG_H */
