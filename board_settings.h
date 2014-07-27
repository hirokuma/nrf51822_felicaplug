#ifndef BOARD_SETTINGS_H
#define BOARD_SETTINGS_H

#define PIN_LED			(21)
#define PIN_DBGLED		(26)
#define PIN_ERRLED		(5)
#define	LED_ON(pin)		nrf_gpio_pin_clear(pin)
#define	LED_OFF(pin)	nrf_gpio_pin_set(pin)

#define PIN_RFDET		(8)
#define PIN_SW			(9)
#define	PIN_IRQ			(11)
#define	PIN_SEL			(12)
#define	PIN_DATA		(13)
#define	PIN_SPICLK		(15)


#endif /* BOARD_SETTINGS_H */
